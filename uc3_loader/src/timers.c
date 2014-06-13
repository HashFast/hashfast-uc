/* timers.c */

/*
    Copyright (c) 2013 HashFast Technologies LLC
*/

#include <stdint.h>
#include <avr32/io.h>

#include "config.h"
#include "interrupts.h"
#include "timers.h"


unsigned int timersTick;


__attribute__((__interrupt__)) static void tickIntHandler(void) {

    timersTick++;
    AVR32_TC.channel[TIMERS_CHANNEL_TICK].sr;
}

void timersInit(void) {

    timersTick = 0;

    interruptsRegister(tickIntHandler, TIMERS_CHANNEL_TICK_IRQ,
                       CONFIG_INT_LEVEL_TICK);
    /* PBa / 8 */
    AVR32_TC.channel[TIMERS_CHANNEL_TICK].cmr = AVR32_TC_WAVE_MASK |
                                                (AVR32_TC_WAVSEL_UP_AUTO <<
                                                 AVR32_TC_WAVSEL_OFFSET) |
                                                (AVR32_TC_TCCLKS_TIMER_CLOCK3 <<
                                                 AVR32_TC_TCCLKS_OFFSET);
    AVR32_TC.channel[TIMERS_CHANNEL_TICK].rc = (CONFIG_PBA_FREQ / 8 +
                                                TIMERS_TICK_HZ / 2) /
                                               TIMERS_TICK_HZ;
    AVR32_TC.channel[TIMERS_CHANNEL_TICK].sr;
    AVR32_TC.channel[TIMERS_CHANNEL_TICK].ier = AVR32_TC_CPCS_MASK;
    /* start */
    AVR32_TC.channel[TIMERS_CHANNEL_TICK].ccr = AVR32_TC_SWTRG_MASK |
                                                AVR32_TC_CLKEN_MASK;
}


