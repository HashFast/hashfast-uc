/* flash.h */

#ifndef _flash_h
#define _flash_h

#ifdef __cplusplus
extern "C" {
#endif


int flashEraseAllFuses(void);

int flashWriteAllFuses(uint32_t fuses);

int flashEraseWrite(void *dest, const void *source, int bytes);


#ifdef __cplusplus
}
#endif

#endif /* _flash_h */


