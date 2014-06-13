/* mpu.h */

#ifndef _mpu_h
#define _mpu_h

#ifdef __cplusplus
extern "C" {
#endif


void mpuSetup(int protectHeap);

int mpuDump(int first, int parmCount, uint32_t *parms);

void mpuException(uint32_t *sp, uint32_t addr, uint32_t cause);


#ifdef __cplusplus
}
#endif

#endif /* _mpu_h */

