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
#include "utils.h"
#include "flash.h"
#include "update_data.h"
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
#  define OSC0_MODE_VALU  E      AVR32_PM_MODE_EXT_CLOCK
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

#define PLL_MAX_STARTUP_CYCLES    ((1 << AVR32_PM_PLL0_PLLCOUNT_SIZE) - 1)

/* linker defined symbols */
extern char _FlashAppStart[];
extern char _FlashAppLength[];


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

}


int main(void) {
    uint32_t suffix[4];

    /* disable watchdog */
    AVR32_WDT.ctrl = (0x55 << AVR32_WDT_CTRL_KEY_OFFSET);
    AVR32_WDT.ctrl = (0xaa << AVR32_WDT_CTRL_KEY_OFFSET);

    interruptsInit();

    interruptsDisable();

    clocksInit();

    flashEraseAllFuses();

    flashEraseWrite((void *) update_data_start, update_data,
                    update_data_length);

    memset(suffix, 0xff, sizeof(suffix));
    flashEraseWrite((void *) ((uint32_t) _FlashAppStart +
                              (uint32_t) _FlashAppLength -
                              sizeof(suffix)),
                    suffix, sizeof(suffix));

    flashWriteAllFuses(CONFIG_GP_FUSES);

    utilsReboot(0, UTILS_REBOOT_MODE_LOADER);

    return 0; /* not reached */
}

