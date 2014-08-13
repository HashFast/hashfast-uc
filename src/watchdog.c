/** @file watchdog.c
 * @brief Watchdog support
 *
 * @copyright
 * Copyright (c) 2014, HashFast Technologies LLC
 * All rights reserved.
 *
 * @page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   1.  Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *   2.  Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *   3.  Neither the name of HashFast Technologies LLC nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL HASHFAST TECHNOLOGIES LLC BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "main.h"

#define WDT_USEC (uint64_t)3000000              // 2 seconds

/**
 * Watchdog options.
 * To specify which current Watchdog value.
 */
static wdt_opt_t opt = {
    .us_timeout_period = WDT_USEC               //!< TimeOut Value
};

/**
 * Inititalize the hardware Watchdog.
 */
void wdt_init(void) {
#ifdef FEATURE_HW_WATCHDOG
    volatile avr32_pm_t* pm = &AVR32_PM;

    /*
     * If Reset Cause is due to a Watchdog reset just relaunch Watchdog.
     */
    if (pm->RCAUSE.wdt) {
        opt.us_timeout_period = WDT_USEC;
        wdt_enable(&opt);
    }
    /*
     * If Reset Cause is due to a Power On reset, enable Watchdog with default
     * value.
     */
    else if (pm->RCAUSE.por) {
        opt.us_timeout_period = WDT_USEC;

        /* Save current value in GPLP register. */
        //pm_write_gplp(pm, 0, opt.us_timeout_period);
        wdt_enable(&opt);
    }
#if 0
    /*
     * If Reset Cause is due to an External reset, increment
     * opt.us_timeout_period .
     */
    else if (pm->RCAUSE.ext) {
        /* Reload current value stored in GPLP register. */
        opt.us_timeout_period = pm_read_gplp(pm, 0);
        opt.us_timeout_period += WDT_CTRL_STEP_US;

        if (opt.us_timeout_period >= WDT_MAX_VALUE_US)
            opt.us_timeout_period = WDT_MIN_VALUE_US;

        wdt_enable(&opt);

        /* Save new value in GPLP register. */
        pm_write_gplp(pm, 0, opt.us_timeout_period);
    }
#endif
    /* else
     * Relaunch Watchdog and toggle GPIO to let user know that a new reset has
     * occurred.
     */
    else {
        opt.us_timeout_period = WDT_USEC;

        /* Save start value of watchdog in GPLP register. */
        pm_write_gplp(pm, 0, opt.us_timeout_period);
        wdt_enable(&opt);
    }
#endif /* FEATURE_HW_WATCHDOG */
}

/**
 * Reset self.
 */
void self_reset(void) {
    volatile avr32_pm_t* pm = &AVR32_PM;

    Disable_global_interrupt();

    wdt_disable();
    pm_switch_to_osc0(pm, FOSC0, OSC0_STARTUP);

    opt.us_timeout_period = 200000;             // 200 msec
    pm_write_gplp(pm, 0, opt.us_timeout_period);
    wdt_enable(&opt);
    for (;;);
}
