/* utils.h */

#ifndef _utils_h
#define _utils_h

#ifdef __cplusplus
extern "C" {
#endif


#define UTILS_REBOOT_MODE_APP         0
#define UTILS_REBOOT_MODE_LOADER      1
#define UTILS_REBOOT_MODE_NOTSET      2


void utilsReboot(unsigned int when, int mode);

int utilsRebootMode(void);



#ifdef __cplusplus
}
#endif

#endif /* _utils_h */

