/* main.c */

/*
    Copyright (c) 2013 HashFast Technologies LLC
*/

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <avr32/io.h>

#include "config.h"
#include "interrupts.h"
#include "flash.h"
#include "gpio.h"
#include "timers.h"
#include "twi.h"
#include "twicomms.h"
#include "chain.h"
#include "utils.h"
#include "flash.h"
#include "usbdev.h"
#include "main.h"

#if CONFIG_BOARD_OSC0_STARTUP_US == 0
#  define OSC0_STARTUP_VALUE    AVR32_PM_OSCCTRL0_STARTUP_0_RCOSC
#  define OSC0_STARTUP_TIMEOUT  8
#elif CONFIG_BOARD_OSC0_STARTUP_US <= 560
#  define OSC0_STARTUP_VALUE    AVR32_PM_OSCCTRL0_STARTUP_64_RCOSC
#  define OSC0_STARTUP_TIMEOUT  80
#elif CONFIG_BOARD_OSC0_STARTUP_US <= 1100
#  define OSC0_STARTUP_VALUE    AVR32_PM_OSCCTRL0_STARTUP_128_RCOSC
#  define OSC0_STARTUP_TIMEOUT  160
#elif CONFIG_BOARD_OSC0_STARTUP_US <= 18000
#  define OSC0_STARTUP_VALUE    AVR32_PM_OSCCTRL0_STARTUP_2048_RCOSC
#  define OSC0_STARTUP_TIMEOUT  2560
#elif CONFIG_BOARD_OSC0_STARTUP_US <= 36000
#  define OSC0_STARTUP_VALUE    AVR32_PM_OSCCTRL0_STARTUP_4096_RCOSC
#  define OSC0_STARTUP_TIMEOUT  5120
#elif CONFIG_BOARD_OSC0_STARTUP_US <= 71000
#  define OSC0_STARTUP_VALUE    AVR32_PM_OSCCTRL0_STARTUP_8192_RCOSC
#  define OSC0_STARTUP_TIMEOUT  10240
#elif CONFIG_BOARD_OSC0_STARTUP_US <= 142000
#  define OSC0_STARTUP_VALUE    AVR32_PM_OSCCTRL0_STARTUP_16384_RCOSC
#  define OSC0_STARTUP_TIMEOUT  20480
#else
#  error
#endif

#if CONFIG_BOARD_OSC0_IS_XTAL
#  if CONFIG_BOARD_OSC0_HZ < 900000
#    define OSC0_MODE_VALUE      AVR32_PM_MODE_CRYSTAL_G0
#  elif CONFIG_BOARD_OSC0_HZ < 3000000
#    define OSC0_MODE_VALUE      AVR32_PM_MODE_CRYSTAL_G1
#  elif CONFIG_BOARD_OSC0_HZ < 8000000
#    define OSC0_MODE_VALUE      AVR32_PM_MODE_CRYSTAL_G2
#  else
#    define OSC0_MODE_VALUE      AVR32_PM_MODE_CRYSTAL_G3
#  endif
#else
#  define OSC0_MODE_VALUE        AVR32_PM_MODE_EXT_CLOCK
#endif

#define PLL0_FREQ   (CONFIG_BOARD_OSC0_HZ * CONFIG_PLL0_MUL / CONFIG_PLL0_DIV)
#if PLL0_FREQ < 80000000
#  if CONFIG_PLL0_MUL <= 8
#    define PLL0_MUL  (CONFIG_PLL0_MUL * 2)
#    define PLL0_DIV  CONFIG_PLL0_DIV
#  elif (CONFIG_PLL0_DIV & 1) == 0
#    define PLL0_MUL  CONFIG_PLL0_MUL
#    define PLL0_DIV  (CONFIG_PLL0_DIV / 2)
#  else
#    error
#  endif
#  define PLL0_OPT  3
#elif PLL0_FREQ < 160000000
#  define PLL0_MUL  CONFIG_PLL0_MUL
#  define PLL0_DIV  CONFIG_PLL0_DIV
#  define PLL0_OPT  1
#else
#  define PLL0_MUL  CONFIG_PLL0_MUL
#  define PLL0_DIV  CONFIG_PLL0_DIV
#  define PLL0_OPT  0
#endif

#define PLL1_FREQ   (CONFIG_BOARD_OSC0_HZ * CONFIG_PLL1_MUL / CONFIG_PLL1_DIV)
#if PLL1_FREQ < 80000000
#  if CONFIG_PLL1_MUL <= 8
#    define PLL1_MUL  (CONFIG_PLL1_MUL * 2)
#    define PLL1_DIV  CONFIG_PLL1_DIV
#  elif (CONFIG_PLL1_DIV & 1) == 0
#    define PLL1_MUL  CONFIG_PLL1_MUL
#    define PLL1_DIV  (CONFIG_PLL1_DIV / 2)
#  else
#    error
#  endif
#  define PLL1_OPT  3
#elif PLL1_FREQ < 160000000
#  define PLL1_MUL  CONFIG_PLL1_MUL
#  define PLL1_DIV  CONFIG_PLL1_DIV
#  define PLL1_OPT  1
#else
#  define PLL1_MUL  CONFIG_PLL1_MUL
#  define PLL1_DIV  CONFIG_PLL1_DIV
#  define PLL1_OPT  0
#endif

#define PLL_MAX_STARTUP_CYCLES    ((1 << AVR32_PM_PLL0_PLLCOUNT_SIZE) - 1)


static void clocksInit(void) {

#if CONFIG_CPU_FREQ > AVR32_FLASHC_FWS_0_MAX_FREQ
    /* set one wait state */
    AVR32_FLASHC.fcr |= AVR32_FLASHC_FCR_FWS_MASK;
#endif

    /* enable osc0 */
    AVR32_PM.oscctrl0 = (OSC0_STARTUP_VALUE <<
                         AVR32_PM_OSCCTRL0_STARTUP_OFFSET) |
                        (OSC0_MODE_VALUE <<
                         AVR32_PM_OSCCTRL0_MODE_OFFSET);
    AVR32_PM.mcctrl |= AVR32_PM_MCCTRL_OSC0EN_MASK;
    /* wait for it to start */
    while (!(AVR32_PM.poscsr & AVR32_PM_POSCSR_OSC0RDY_MASK))
        ;

    /* enable main pll */
    AVR32_PM.pll[0] = ((PLL0_MUL - 1) << AVR32_PM_PLL0_PLLMUL_OFFSET) |
                      (PLL0_DIV << AVR32_PM_PLL0_PLLDIV_OFFSET) |
                      (PLL_MAX_STARTUP_CYCLES <<
                       AVR32_PM_PLL0_PLLCOUNT_OFFSET) |
                      (0 << AVR32_PM_PLL0_PLLOSC_OFFSET) |
                      (PLL0_OPT << AVR32_PM_PLL0_PLLOPT_OFFSET) |
                      AVR32_PM_PLL0_PLLEN_MASK;
    /* wait for lock */
    while (!(AVR32_PM.poscsr & AVR32_PM_POSCSR_LOCK0_MASK))
        ;

    /* switch clock */
    AVR32_PM.mcctrl = (AVR32_PM.mcctrl & ~AVR32_PM_MCCTRL_MCSEL_MASK) |
                      (AVR32_PM_MCCTRL_MCSEL_PLL0 <<
                       AVR32_PM_MCCTRL_MCSEL_OFFSET);


    /* enable second pll */
    AVR32_PM.pll[1] = ((PLL1_MUL - 1) << AVR32_PM_PLL1_PLLMUL_OFFSET) |
                      (PLL1_DIV << AVR32_PM_PLL1_PLLDIV_OFFSET) |
                      (PLL_MAX_STARTUP_CYCLES <<
                       AVR32_PM_PLL1_PLLCOUNT_OFFSET) |
                      (0 << AVR32_PM_PLL1_PLLOSC_OFFSET) |
                      (PLL1_OPT << AVR32_PM_PLL1_PLLOPT_OFFSET) |
                      AVR32_PM_PLL1_PLLEN_MASK;
    /* wait for lock */
    while (!(AVR32_PM.poscsr & AVR32_PM_POSCSR_LOCK1_MASK))
        ;
    /* enable usb clock */
    AVR32_PM.gcctrl[AVR32_PM_GCLK_USBB] = AVR32_PM_GCCTRL_OSCSEL_MASK |
                                          AVR32_PM_GCCTRL_PLLSEL_MASK;
    AVR32_PM.gcctrl[AVR32_PM_GCLK_USBB] |= AVR32_PM_GCCTRL_CEN_MASK;

}

static void clocksDeinit(void) {

    /* switch main clock to slow clock, disable oscillators */
    AVR32_PM.mcctrl = 0;
    /* go back to no wait states */
    AVR32_FLASHC.fcr &= ~AVR32_FLASHC_FCR_FWS_MASK;
    /* disable pll */
    AVR32_PM.pll[0] = 0;

    /* disable usb clock */
    AVR32_PM.gcctrl[AVR32_PM_GCLK_USBB] = 0;
    /* disable second pll */
    AVR32_PM.pll[1] = 0;

    /* clear configuration for osc0 */
    AVR32_PM.oscctrl0 = 0;
}

#define FANS_PWM_PERIOD   (CONFIG_PBA_FREQ / (8 * 25000))

static void fansInit(void) {

    gpioPinFunc(AVR32_PWM_4_1_PIN, AVR32_PWM_4_1_FUNCTION);
    gpioPinFunc(AVR32_PWM_0_0_PIN, AVR32_PWM_0_0_FUNCTION);
    AVR32_PWM.mr = 0;
    AVR32_PWM.channel[CONFIG_PWM_CHANNEL_PWM1].cmr =
        AVR32_PWM_CMR_CPOL_MASK | AVR32_PWM_CMR_CPRE_MCK_DIV_8;
    AVR32_PWM.channel[CONFIG_PWM_CHANNEL_PWM1].cprd = FANS_PWM_PERIOD;
    AVR32_PWM.channel[CONFIG_PWM_CHANNEL_PWM1].cdty = FANS_PWM_PERIOD;
    AVR32_PWM.channel[CONFIG_PWM_CHANNEL_PWM2].cmr =
        AVR32_PWM_CMR_CPOL_MASK | AVR32_PWM_CMR_CPRE_MCK_DIV_8;
    AVR32_PWM.channel[CONFIG_PWM_CHANNEL_PWM2].cprd = FANS_PWM_PERIOD;
    AVR32_PWM.channel[CONFIG_PWM_CHANNEL_PWM2].cdty = FANS_PWM_PERIOD / 5;
    AVR32_PWM.ena = (1 << CONFIG_PWM_CHANNEL_PWM1) |
                    (1 << CONFIG_PWM_CHANNEL_PWM2);

    gpioPinSet(CONFIG_GPIO_FAN1, 1);
    gpioPinOutputEnable(CONFIG_GPIO_FAN1, 1);
    gpioPinSet(CONFIG_GPIO_FAN2, 1);
    gpioPinOutputEnable(CONFIG_GPIO_FAN2, 1);
}

static void startApp(void (*entry)(void)) {

    interruptsDisable();
    clocksDeinit();

    entry(); /* no return */
}

int main(void) {
    void (*entry)(void);

    /* disable watchdog */
    AVR32_WDT.ctrl = (0x55 << AVR32_WDT_CTRL_KEY_OFFSET);
    AVR32_WDT.ctrl = (0xaa << AVR32_WDT_CTRL_KEY_OFFSET);

    interruptsDisable();

    clocksInit();

    memset(usbdevDebugBuffer, sizeof(usbdevDebugBuffer), 0);

    gpioPinOutputEnable(CONFIG_GPIO_RECONFIG_BUTTON, 0);
    gpioPinPullup(CONFIG_GPIO_RECONFIG_BUTTON, 1);
    if ((utilsRebootMode() != UTILS_REBOOT_MODE_LOADER) &&
        gpioPinValue(CONFIG_GPIO_RECONFIG_BUTTON) &&
        ((entry = flashVerifyApp()) != NULL))
        startApp(entry);

    gpioPinSet(CONFIG_GPIO_PWR_ON, 1);
    gpioPinOutputEnable(CONFIG_GPIO_PWR_ON, 1);

    gpioPinSet(CONFIG_GPIO_SPARE_DOWN, 1);
    gpioPinOutputEnable(CONFIG_GPIO_SPARE_DOWN, 1);

    gpioPinOutputEnable(CONFIG_GPIO_SPARE_UP, 0);
    gpioPinPullup(CONFIG_GPIO_SPARE_UP, 1);

    gpioPinOutputEnable(CONFIG_GPIO_GOT_UP, 0);
    gpioPinPullup(CONFIG_GPIO_GOT_UP, 1);

    gpioPinOutputEnable(CONFIG_GPIO_GOT_DOWN, 0);
    gpioPinPullup(CONFIG_GPIO_GOT_DOWN, 1);

    gpioPinOutputEnable(CONFIG_GPIO_USB_DOWN, 0);
    gpioPinPullup(CONFIG_GPIO_USB_DOWN, 1);

    gpioPinSet(CONFIG_GPIO_HAVE_USB, 1);
    gpioPinOutputEnable(CONFIG_GPIO_HAVE_USB, 1);

    gpioPinFunc(AVR32_TWI_SDA_0_0_PIN, AVR32_TWI_SDA_0_0_FUNCTION);
    gpioPinFunc(AVR32_TWI_SCL_0_0_PIN, AVR32_TWI_SCL_0_0_FUNCTION);

    fansInit();

    interruptsInit();

    twiInit();
    twicommsInit();

    chainInit();

    timersInit();

    flashReset();

    usbdevInit();

    interruptsEnable();

    while (1) {
        chainTask();
        usbdevTask();
        twicommsTask();
    }

    return 0; /* not reached */
}

