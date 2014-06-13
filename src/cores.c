
#include "main.h"

uint16_t core_good[MAX_CORES/16];
uint16_t core_good_persist[MAX_CORES/16];
uint16_t core_good_slow[MAX_CORES/16];

/**
 * Marks a core as enabled in the softmap.
 * Optionally persists this change to the hardmap.
 */
int core_map_set_enabled(int core_index, int persist) {
    if(persist) {
        /*
         * TODO: currently takes a very long time
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
        // set core_good bit
        core_good[((core_index) >> 4)] |= ((uint16_t)0x0001 << ((core_index) & 0xF));
        // TODO not sure why this is a two
        ucinfo.shed_amount -= 2;
        return 1;
    } else {
        return 0;
    }
}

/**
 * Marks a core as disabled in the softmap.
 * Optionally persists this change to the hardmap.
 */
int core_map_set_disabled(int core_index, int persist) {
    if(persist) {
        hf_nvram_write_bad_core_bitmap(core_index / 96, (0x4000 | (core_index % 96) ));
        core_map_refresh();
    }
    /* Test is to protect shed_amount from growing */
    if(CORE_GOOD(core_index)) {
        // clear core_good bit
        core_good[((core_index) >> 4)] &= ~((uint16_t)0x0001 << ((core_index) & 0xF));
        /*
         * Unless this is done, work overruns can occur, and eventually either the world blows up,
         * or on the next work restart the gap won't be closed so we'll time out and have to be restarted
         */
        // TODO not sure why this is a two
        ucinfo.shed_amount += 2;
        return 1;
    } else {
        return 0;
    }
}

/**
 * Completely resets the softmap and enables all cores.
 * Optionally persists this change to the hardmap.
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
 */
int core_map_try_enable(void) {
    return 0;
}

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
