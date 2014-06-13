/* trap.h */

#ifndef _trap_h
#define _trap_h

#ifdef __cplusplus
extern "C" {
#endif


void trap(uint32_t *sp, uint32_t addr, uint32_t cause);


#ifdef __cplusplus
}
#endif

#endif /* _trap_h */

