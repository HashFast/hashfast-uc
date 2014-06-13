/* usbdev.h */

#ifndef _usbdev_h
#define _usbdev_h

#ifdef __cplusplus
extern "C" {
#endif


#define USBDEV_CONTROL_EP_SIZE          64

extern uint8_t usbdevDebugBuffer[USBDEV_CONTROL_EP_SIZE];


void usbdevInit(void);

void usbdevTask(void);


/* callbacks from ASF stack: */

int usbdevVendorEnable(void);

void usbdevVendorDisable(void);

int usbdevSetup(void);


#ifdef __cplusplus
}
#endif

#endif /* _usbdev_h */

