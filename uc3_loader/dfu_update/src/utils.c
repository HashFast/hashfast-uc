/* utils.c */

/*
    Copyright (c) 2013 HashFast Technologies LLC
*/

#include <stdint.h>
#include <avr32/io.h>

#include "config.h"
#include "utils.h"


#define MAILBOX                 AVR32_PM.gplp[1]


#define MBMSG_NONE              0x00000000
#define MBMSG_NO_APP_START      0x73746179


void utilsReboot(unsigned int delay, int mode) {
    uint32_t ticks;
    uint32_t prescale;

    ticks = (delay * (AVR32_PM_RCOSC_FREQUENCY / 1000));
    prescale = 0;
    while (ticks /= 2)
        prescale++;
    if (prescale < 12)
        prescale = 12;
    if (prescale > (AVR32_WDT_CTRL_PSEL_MASK >> AVR32_WDT_CTRL_PSEL_OFFSET))
        prescale = (AVR32_WDT_CTRL_PSEL_MASK >> AVR32_WDT_CTRL_PSEL_OFFSET);

    /* disable watchdog */
    AVR32_WDT.ctrl = (0x55 << AVR32_WDT_CTRL_KEY_OFFSET) |
                     (prescale << AVR32_WDT_CTRL_PSEL_OFFSET);
    AVR32_WDT.ctrl = (0xaa << AVR32_WDT_CTRL_KEY_OFFSET) |
                     (prescale << AVR32_WDT_CTRL_PSEL_OFFSET);

    switch (mode) {
    case UTILS_REBOOT_MODE_LOADER:
        MAILBOX = MBMSG_NO_APP_START;
        break;
    case UTILS_REBOOT_MODE_APP:
    case UTILS_REBOOT_MODE_NOTSET:
    default:
        MAILBOX = MBMSG_NONE;
        break;
    }

    /* enable watchdog */
    AVR32_WDT.ctrl = (0x55 << AVR32_WDT_CTRL_KEY_OFFSET) |
                     (prescale << AVR32_WDT_CTRL_PSEL_OFFSET) |
                     AVR32_WDT_CTRL_EN_MASK;
    AVR32_WDT.ctrl = (0xaa << AVR32_WDT_CTRL_KEY_OFFSET) |
                     (prescale << AVR32_WDT_CTRL_PSEL_OFFSET) |
                     AVR32_WDT_CTRL_EN_MASK;

    /* just wait for it to fire */
    while (1)
        ;
}

/* clears any message so this only works once per boot */
int utilsRebootMode(void) {
    int mode;

    mode = UTILS_REBOOT_MODE_NOTSET;
    if (AVR32_PM.rcause & AVR32_PM_RCAUSE_WDT_MASK) {
        switch (MAILBOX) {
        case MBMSG_NONE:
            mode = UTILS_REBOOT_MODE_APP;
            break;
        case MBMSG_NO_APP_START:
            mode = UTILS_REBOOT_MODE_LOADER;
            break;
        }
    }
    MAILBOX = MBMSG_NONE;

    return mode;
}

