/** @file cores.h
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

#ifndef _CORE_MAP_H
#define _CORE_MAP_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Some cores may be broken/disabled. Here's a bitmap to track this.
 * This is set up by startup diagnostics. A '1' means the core is
 * good, a '0' means it is bad.
 */
extern uint16_t core_good[MAX_CORES/16];            /* 240 bytes */
/**
 * This is the broken/disabled core map from nvram.
 */
extern uint16_t core_good_persist[MAX_CORES/16];    /* 240 bytes */
/**
 * If a high speed test is done, these are the results
 */
extern uint16_t core_good_slow[MAX_CORES/16];       /* 240 bytes */

#define CORE_GOOD(x) (core_good[((x) >> 4)] & ((uint16_t)0x0001 << ((x) & 0xF)))
#define CORE_BAD(x)  (CORE_GOOD(x) ? 0x00 : 0x01)

int core_map_set_enabled(int core_index, int persist);
int core_map_set_disabled(int core_index, int persist);
int core_map_reset(int persist);

int core_map_try_enable(void);

void core_map_refresh(void);

#ifdef __cplusplus
}
#endif

#endif /* _CORE_MAP_H */
