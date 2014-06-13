/* timers.h */

#ifndef _timers_h
#define _timers_h

#ifdef __cplusplus
extern "C" {
#endif


#define TIMERS_CHANNEL_TICK                0
#define TIMERS_CHANNEL_TICK_IRQ            AVR32_TC_IRQ0


#define TIMERS_TICK_HZ                     1000


extern unsigned int timersTick;


void timersInit(void);


#ifdef __cplusplus
}
#endif

#endif /* _timers_h */

