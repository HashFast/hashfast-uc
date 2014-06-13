//
// Watchdog support
//

#include "main.h"

#define WDT_USEC (uint64_t)3000000                      // 2 seconds


// To specify which current Watchdog value
static wdt_opt_t opt = {
    .us_timeout_period = WDT_USEC                       // TimeOut Value
    };

void wdt_init(void)
    {
#ifdef FEATURE_HW_WATCHDOG
    volatile avr32_pm_t* pm = &AVR32_PM;
 
    // If Reset Cause is due to a Watchdog reset just relaunch Watchdog
    if (pm->RCAUSE.wdt)
        {
        opt.us_timeout_period = WDT_USEC ;
        wdt_enable(&opt);
        }
    // If Reset Cause is due to a Power On reset, enable Watchdog with default value
    else if (pm->RCAUSE.por)
        {
        opt.us_timeout_period = WDT_USEC ;
 
        // Save current value in GPLP register
        //pm_write_gplp(pm, 0, opt.us_timeout_period);
        wdt_enable(&opt);
        }
#if 0
    // If Reset Cause is due to an External reset, increment opt.us_timeout_period
    else if (pm->RCAUSE.ext)
        {
        // Reload current value stored in GPLP register
        opt.us_timeout_period = pm_read_gplp(pm, 0);
        opt.us_timeout_period += WDT_CTRL_STEP_US;
 
        if (opt.us_timeout_period >= WDT_MAX_VALUE_US)
            opt.us_timeout_period = WDT_MIN_VALUE_US;
 
        wdt_enable(&opt);
 
        // Save new value in GPLP register
        pm_write_gplp(pm,0,opt.us_timeout_period);
        }
#endif
     // Else relaunch Watchdog and toggle GPIO to let user know that a new reset has occured
     else
        {
        opt.us_timeout_period = WDT_USEC;
 
        // Save start value of watchdog in GPLP register
        pm_write_gplp(pm, 0, opt.us_timeout_period);
        wdt_enable(&opt);
        }
#endif // FEATURE_HW_WATCHDOG
    }

void self_reset(void)
    {
    volatile avr32_pm_t* pm = &AVR32_PM;

    Disable_global_interrupt();

    wdt_disable();
    pm_switch_to_osc0(pm, FOSC0, OSC0_STARTUP);

    opt.us_timeout_period = 200000;       // 200 msec
    pm_write_gplp(pm, 0, opt.us_timeout_period);
    wdt_enable(&opt);
    for (;;);
    }

