/* flash.h */

#ifndef _flash_h
#define _flash_h

#ifdef __cplusplus
extern "C" {
#endif


void (*flashVerifyApp(void))(void);

void flashReset(void);

int flashSubmitFWBuffer(unsigned char *buffer, int length);

uint32_t flashSize(void);

#ifdef __cplusplus
}
#endif

#endif /* _flash_h */


