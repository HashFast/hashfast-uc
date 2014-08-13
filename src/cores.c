/** @file cores.c
 * @brief Controls cores under GWQ.
 *
 * @copyright
 * Copyright (c) 2014, HashFast Technologies LLC
 * All rights reserved.
 *
 * @page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   1.  Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *   2.  Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *   3.  Neither the name of HashFast Technologies LLC nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL HASHFAST TECHNOLOGIES LLC BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "main.h"

uint16_t core_good[MAX_CORES/16];
uint16_t core_good_persist[MAX_CORES/16];
uint16_t core_good_slow[MAX_CORES/16];

/**
 * Marks a core as enabled in the softmap.
 * Optionally persists this change to the hardmap.
 * @param core_index
 * @param persist
 * @return success
 */
int core_map_set_enabled(int core_index, int persist) {
    if(persist) {
        /* TODO currently takes a very long time
        int i, j, temp_module, temp_core_index;
        uint16_t temp_core_map[6];
        temp_module = core_index / 96;
        temp_core_index = core_index % 96;
        hf_nvram_read_bad_core_bitmap(temp_module, &temp_core_map[0]);
        hf_nvram_write_bad_core_bitmap(temp_module, 0x8000);
        temp_core_map[(temp_core_index) >> 4] |= ((uint16_t)0x0001 << ((temp_core_index) & 0xF));
        for(i = 0; i < 6; i++) {
            for(j = 0; j < 16; j++) {
                if( (temp_core_map[i] & ((uint16_t)0x0001 << ((j) & 0xF))) ? 0x00 : 0x01 ) {
                    temp_core_index = 6*i + j;
                    hf_nvram_write_bad_core_bitmap(temp_module, (0x4000 | temp_core_index));
                }
            }
        }
        core_map_refresh();
        */
    }
    /* Test is to protect shed_amount from changing unnecessarily */
    if(CORE_BAD(core_index)) {
        /* set core_good bit */
        core_good[((core_index) >> 4)] |= ((uint16_t)0x0001 << ((core_index) & 0xF));
        /* Shed amount reflects inflight target (2 per core) */
        ucinfo.shed_amount -= 2;
        return 1;
    } else {
        return 0;
    }
}

/**
 * Marks a core as disabled in the softmap.
 * Optionally persists this change to the hardmap.
 * @param core_index
 * @param persist
 * @return success
 */
int core_map_set_disabled(int core_index, int persist) {
    if(persist) {
        hf_nvram_write_bad_core_bitmap(core_index / 96, (0x4000 | (core_index % 96) ));
        core_map_refresh();
    }
    /* Test is to protect shed_amount from growing */
    if(CORE_GOOD(core_index)) {
        /* clear core_good bit */
        core_good[((core_index) >> 4)] &= ~((uint16_t)0x0001 << ((core_index) & 0xF));
        /*
         * Unless this is done, work overruns can occur, and eventually either
         * the world blows up, or on the next work restart the gap won't be
         * closed so we'll time out and have to be restarted.
         * Shed amount reflects inflight target (2 per core)
         */
        ucinfo.shed_amount += 2;
        return 1;
    } else {
        return 0;
    }
}

/**
 * Completely resets the softmap and enables all cores.
 * Optionally persists this change to the hardmap.
 * @param persist
 * @return success
 */
int core_map_reset(int persist) {
    int i;
    if(persist) {
        for(i = 0; i <= ucinfo.num_slaves; i++) {
            hf_nvram_write_bad_core_bitmap(i, 0x8000);
        }
        core_map_refresh();
    }
    for(i = 0; i < MAX_CORES/16; i++) {
        core_good[i] = (uint16_t) 0xFFFF;
    }
    ucinfo.shed_amount = 0;
    return 1;
}

/**
 * Checks when each core was disabled and attempts to reactive it.
 * If the core is in the disabled hardmap or has been deactivated
 * too often, then it is left disabled.
 * @return success
 */
int core_map_try_enable(void) {
    return 0;
}

/**
 * Refresh the softmap from NVRAM
 */
void core_map_refresh(void) {
    int i;
    for(i = 0; i <= ucinfo.num_slaves; i++) {
        hf_nvram_read_bad_core_bitmap(i, &core_good_persist[i*6]);
    }
}

#ifdef FEATURE_CORE_STATISTICS

extern uint32_t core_ranges[MAX_CORES];
extern uint32_t core_nonces[MAX_CORES];
extern uint64_t die_hashes[MAX_DIE];
extern uint64_t die_nonces[MAX_DIE];
extern uint64_t asic_hashes[MAX_SLAVES];
extern uint64_t asic_nonces[MAX_SLAVES];

#endif /* FEATURE_CORE_STATISTICS */
