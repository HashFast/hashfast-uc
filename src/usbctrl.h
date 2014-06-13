/* usbctrl.h */

#ifndef _usbctrl_h
#define _usbctrl_h

#ifdef __cplusplus
extern "C" {
#endif


extern uint8_t usbctrlDebugBuffer[64];


int usbctrlDebugStreamWriteStr(const char *);

int usbctrlDebugStreamPrintf(const char *, ...);

int usbctrlDebugMonitorRead(void);

int usbctrlDebugMonitorWrite(char c);

int usbctrlSetupPacket(void);

void usbctrlTask(void);


#ifdef __cplusplus
}
#endif

#endif /* _usbctrl_h */

