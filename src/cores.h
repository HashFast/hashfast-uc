/* core_map.h */

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
