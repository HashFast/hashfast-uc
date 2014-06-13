/* flash.c */

/*
    Copyright (c) 2013 HashFast Technologies LLC
*/

#include <string.h>
#include <stdint.h>
#include <avr32/io.h>

#include "config.h"
#include "flash.h"



static struct {
    /* we keep our own internal sector buffer even though the Atmel has
       a built-in buffer because the built-in one does not permit 8 bit
       writes, which would complicate matters.  */
    uint32_t buffer[AVR32_FLASHC_PAGE_SIZE / 4];
} flash;


static void findSectorOffset(uint32_t addr, int *sector, int *offset) {
    int s;

    addr -= AVR32_FLASH_ADDRESS;
    s = addr / AVR32_FLASHC_PAGE_SIZE;
    addr -= s * AVR32_FLASHC_PAGE_SIZE;
    if (sector)
        *sector = (int) s;
    if (offset)
        *offset = (int) addr;
}

static uint32_t __attribute__((section(".ramfunc"))) doFlashCmd(uint32_t cmd) {

    uint32_t status;

    AVR32_FLASHC.fcmd = cmd;
    while (!((status = AVR32_FLASHC.fsr) & AVR32_FLASHC_FSR_FRDY_MASK))
        ;

    return status;
}

static uint32_t __attribute__((section(".ramfunc"))) doFlashCmds(
    uint32_t cmds[], unsigned int count) {
    uint32_t status;
    unsigned int i;

    status = 0;
    for (i = 0; i < count; i++)
        status |= doFlashCmd(cmds[i]);

    return status;
}

static int doEraseWrite(int sector, uint32_t *buffer) {
    uint32_t *flashPtr;
    uint32_t stat;
    int i;

    doFlashCmd((AVR32_FLASHC_FCMD_KEY_KEY <<
                AVR32_FLASHC_FCMD_KEY_OFFSET) |
                (AVR32_FLASHC_FCMD_CMD_CPB <<
                 AVR32_FLASHC_FCMD_CMD_OFFSET));
    flashPtr = (uint32_t *) (AVR32_FLASH_ADDRESS +
                             sector * AVR32_FLASHC_PAGE_SIZE);
    stat = doFlashCmd((AVR32_FLASHC_FCMD_KEY_KEY <<
                       AVR32_FLASHC_FCMD_KEY_OFFSET) |
                      (sector << AVR32_FLASHC_FCMD_PAGEN_OFFSET) |
                      (AVR32_FLASHC_FCMD_CMD_EP <<
                       AVR32_FLASHC_FCMD_CMD_OFFSET));
    if ((stat & (AVR32_FLASHC_FSR_PROGE_MASK |
                 AVR32_FLASHC_FSR_LOCKE_MASK)) == 0) {
        for (i = 0; i < AVR32_FLASHC_PAGE_SIZE / 4; i++)
            *flashPtr++ = *buffer++;
        stat = doFlashCmd((AVR32_FLASHC_FCMD_KEY_KEY <<
                           AVR32_FLASHC_FCMD_KEY_OFFSET) |
                          (sector << AVR32_FLASHC_FCMD_PAGEN_OFFSET) |
                          (AVR32_FLASHC_FCMD_CMD_WP <<
                           AVR32_FLASHC_FCMD_CMD_OFFSET));
    }

    return (stat & (AVR32_FLASHC_FSR_PROGE_MASK |
                    AVR32_FLASHC_FSR_LOCKE_MASK)) ? 1 : 0;
}

int flashEraseWrite(void *dest, const void *source, int bytes) {
    uint32_t stat;
    int sector;
    int offset;
    int count;
    const uint8_t *s;
    uint8_t *d;

    stat = 0;
    findSectorOffset((uint32_t) dest, &sector, &offset);
    s = (uint8_t *) source;
    d = (uint8_t *) AVR32_FLASH_ADDRESS + sector * AVR32_FLASHC_PAGE_SIZE;
    while (bytes &&
           (stat & (AVR32_FLASHC_FSR_PROGE_MASK |
                    AVR32_FLASHC_FSR_LOCKE_MASK)) == 0) {
        count = AVR32_FLASHC_PAGE_SIZE - offset;
        if (count > bytes)
            count = bytes;
        memcpy(flash.buffer, d, AVR32_FLASHC_PAGE_SIZE);
        memcpy((uint8_t *) flash.buffer + offset, s, count);
        stat = doEraseWrite(sector, flash.buffer);
        bytes -= count;
        s += count;
        d += AVR32_FLASHC_PAGE_SIZE;
        offset = 0;
        sector++;
    }

    return (stat & (AVR32_FLASHC_FSR_PROGE_MASK |
                    AVR32_FLASHC_FSR_LOCKE_MASK)) ? 1 : 0;
}

int flashEraseAllFuses(void) {
    uint32_t stat;
    uint32_t commands[32 + 2];
    unsigned int i;

    /* I was unable to get the erase all fuses command to work, so this
       erases them all one at a time. */
    for (i = 0; i < 32; i++) {
        commands[i] = (AVR32_FLASHC_FCMD_KEY_KEY <<
                       AVR32_FLASHC_FCMD_KEY_OFFSET) |
                      ((uint32_t) i <<
                       AVR32_FLASHC_FCMD_PAGEN_OFFSET) |
                      (AVR32_FLASHC_FCMD_CMD_EGPB <<
                       AVR32_FLASHC_FCMD_CMD_OFFSET);
    }
    /* these are for flashc errata */
    commands[i++] = (AVR32_FLASHC_FCMD_KEY_KEY <<
                     AVR32_FLASHC_FCMD_KEY_OFFSET) |
                    (AVR32_FLASHC_FCMD_CMD_CPB <<
                     AVR32_FLASHC_FCMD_CMD_OFFSET);
    commands[i++] = (AVR32_FLASHC_FCMD_KEY_KEY <<
                     AVR32_FLASHC_FCMD_KEY_OFFSET) |
                    /* arbitrary block, page buffer is blank so it won't
                       have any effect */
                    (0x100 <<
                     AVR32_FLASHC_FCMD_PAGEN_OFFSET) |
                    (AVR32_FLASHC_FCMD_CMD_WP <<
                     AVR32_FLASHC_FCMD_CMD_OFFSET);

    stat = doFlashCmds(commands, i);

    return (stat & (AVR32_FLASHC_FSR_PROGE_MASK |
                    AVR32_FLASHC_FSR_LOCKE_MASK)) ? 1 : 0;
}

int flashWriteAllFuses(uint32_t fuses) {
    uint32_t commands[32 + 2];
    uint32_t stat;
    unsigned int i;

    for (i = 0; i < 32; i++) {
        if ((fuses & 1) == 0) {
            commands[i] = (AVR32_FLASHC_FCMD_KEY_KEY <<
                           AVR32_FLASHC_FCMD_KEY_OFFSET) |
                          ((uint32_t) i <<
                           AVR32_FLASHC_FCMD_PAGEN_OFFSET) |
                          (AVR32_FLASHC_FCMD_CMD_WGPB <<
                           AVR32_FLASHC_FCMD_CMD_OFFSET);
        }
        fuses >>= 1;
    }
    /* these are for flashc errata */
    commands[i++] = (AVR32_FLASHC_FCMD_KEY_KEY <<
                     AVR32_FLASHC_FCMD_KEY_OFFSET) |
                    (AVR32_FLASHC_FCMD_CMD_CPB <<
                     AVR32_FLASHC_FCMD_CMD_OFFSET);
    commands[i++] = (AVR32_FLASHC_FCMD_KEY_KEY <<
                     AVR32_FLASHC_FCMD_KEY_OFFSET) |
                    /* arbitrary block, page buffer is blank so it won't
                       have any effect */
                    (0x100 <<
                     AVR32_FLASHC_FCMD_PAGEN_OFFSET) |
                    (AVR32_FLASHC_FCMD_CMD_WP <<
                     AVR32_FLASHC_FCMD_CMD_OFFSET);

    stat = doFlashCmds(commands, i);

    return (stat & (AVR32_FLASHC_FSR_PROGE_MASK |
                    AVR32_FLASHC_FSR_LOCKE_MASK)) ? 1 : 0;
}

