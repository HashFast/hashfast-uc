/* profile.h */

#ifndef _profile_h
#define _profile_h

#ifdef __cplusplus
extern "C" {
#endif


#define PROFILE_CHANNELS                64

/* some fixed channels */
#define PROFILE_CHANNEL_TC_ISR           0
#define PROFILE_CHANNEL_GPIO_ISR         1
#define PROFILE_CHANNEL_SPIDMA_ISR       2
#define PROFILE_CHANNEL_TWI_ISR          3
#define PROFILE_CHANNEL_UART_ISR         4
#define PROFILE_CHANNEL_UARTTXDMA_ISR    5
#define PROFILE_CHANNEL_UARTRXDMA_ISR    6
#define PROFILE_CHANNEL_USB_ISR          7
#define PROFILE_CHANNEL_MAINLOOP         8



#define PROFILE_FLAGS_ISR             0x01


#ifdef FEATURE_PROFILE

void profileInit(void);

void profileReset(void);

void profileConfigure(unsigned int channel, uint8_t flags);

void profileEnter(unsigned int channel);

void profileExit(unsigned int channel);

int profileCLI(int first, int parmCount, uint32_t *parms);

#else /* FEATURE_PROFILE */

#define profileInit()
#define profileReset()
#define profileConfigure(channel, flags)
#define profileEnter(channel)
#define profileExit(channel)

#endif /* FEATURE_PROFILE */


#ifdef __cplusplus
}
#endif

#endif /* _profile_h */

