/* flash.c */

/*
    Copyright (c) 2013 HashFast Technologies LLC
*/

#include <stdint.h>
#include <string.h>
#include <avr32/io.h>

#include "config.h"
#include "hf_loader.h"
#include "hf_loader_p.h"
#include "interrupts.h"
#include "crc.h"
#include "flash.h"



#define SECTOR_NONE               (-1)
#define SECTOR_USER_PAGE          (-2)
#define SECTOR_FUSES              (-3)


#ifndef MAX
#define MAX(a,b)   ((a) > (b) ? (a) : (b))
#endif


/* linker defined symbols */
extern char _FlashAppStart[];
extern char _FlashAppLength[];


static struct {
    enum {headerFS, dataFS} state;
    int index;
    int appSpaceWritten;
    hfLoaderBlockHeaderT blockHeader;
    int sector;
    int offset;
    int errorCode;
    /* we keep our own internal sector buffer even though the Atmel has
       a built-in buffer because the built-in one does not permit 8 bit
       writes, which would complicate matters.  */
    uint32_t buffer[MAX(AVR32_FLASHC_PAGE_SIZE,
                        AVR32_FLASHC_USER_PAGE_SIZE) / 4];
} flash;


void (*flashVerifyApp(void))(void) {
    hfLoaderAppSuffixT *suffix;
    uint32_t crc;
    void (*entry)(void);

    entry = NULL;
    suffix = (hfLoaderAppSuffixT *) ((uint32_t) _FlashAppStart +
                                     (uint32_t) _FlashAppLength -
                                     sizeof(hfLoaderAppSuffixT));
    if (suffix->magic == HF_LOADER_SUFFIX_MAGIC &&
        suffix->length <= ((uint32_t) _FlashAppLength -
                           sizeof(hfLoaderAppSuffixT)) &&
        suffix->entry >= (uint32_t) _FlashAppStart &&
        suffix->entry < ((uint32_t) _FlashAppStart + suffix->length)) {
        crc = crcAccumulate(CRC_INITIAL, (uint8_t *) _FlashAppStart,
                            (unsigned int) suffix->length);
        crc = crcAccumulate(crc, (uint8_t *) suffix,
                            sizeof(hfLoaderAppSuffixT) - sizeof(uint32_t));
        if (crc == suffix->crc)
            entry = (void (*)(void)) suffix->entry;
    }

    return entry;
}

void flashReset(void) {

    flash.state = headerFS;
    flash.appSpaceWritten = 0;
    flash.index = 0;
    flash.errorCode = HF_LOADER_STATUS_OK;
}

static void findSectorOffset(uint32_t addr, int *sector, int *offset) {
    int s;

    if (addr >= (uint32_t) &AVR32_FLASHC.fgpfrlo &&
        addr <= (uint32_t) &AVR32_FLASHC.fgpfrlo + 3) {
        addr -= (uint32_t) &AVR32_FLASHC.fgpfrlo;
        s = SECTOR_FUSES;
    } else if (addr >= AVR32_FLASHC_USER_PAGE_ADDRESS &&
               addr < AVR32_FLASHC_USER_PAGE_ADDRESS +
                      AVR32_FLASHC_USER_PAGE_SIZE) {
        addr -= AVR32_FLASHC_USER_PAGE_ADDRESS;
        s = SECTOR_USER_PAGE;
    } else if (addr >= AVR32_FLASH_ADDRESS &&
               addr < AVR32_FLASH_ADDRESS + AVR32_FLASH_SIZE) {
        addr -= AVR32_FLASH_ADDRESS;
        s = addr / AVR32_FLASHC_PAGE_SIZE;
        addr -= s * AVR32_FLASHC_PAGE_SIZE;
    } else {
        addr = 0;
        s = SECTOR_NONE;
    }
    if (sector)
        *sector = (int) s;
    if (offset)
        *offset = (int) addr;
}

static uint32_t __attribute__((section(".ramfunc"))) doFlashCmd(uint32_t cmd) {

    uint32_t status;

    interruptsDisable();
    AVR32_FLASHC.fcmd = cmd;
    while (!((status = AVR32_FLASHC.fsr) & AVR32_FLASHC_FSR_FRDY_MASK))
        ;
    interruptsEnable();

    return status;
}

static uint32_t __attribute__((section(".ramfunc"))) doFlashCmds(
    uint32_t *cmds, unsigned int count) {
    uint32_t s;
    uint32_t status;
    unsigned int i;

    status = 0;
    interruptsDisable();
    for (i = 0; i < count; i++) {
        AVR32_FLASHC.fcmd = cmds[i];
        while (!((s = AVR32_FLASHC.fsr) & AVR32_FLASHC_FSR_FRDY_MASK))
            ;
        status |= s;
    }
    interruptsEnable();

    return status;
}

uint32_t flashSize(void) {
  return doFlashCmd(AVR32_FLASHC_CMD_SIZE);
}

static int doEraseWrite(int sector, uint32_t *buffer) {
    uint32_t *flashPtr;
    uint32_t stat;
    uint32_t fuses;
    uint32_t fusesDelta;
    uint32_t commands[32 + 2];
    int i, j;

    if (sector == SECTOR_FUSES) {
        fuses = *buffer;
        fusesDelta = fuses ^ AVR32_FLASHC.fgpfrlo;
        for (i = 0, j = 0; i < 32; i++) {
            if (fusesDelta & 1)
                commands[j++] = (AVR32_FLASHC_FCMD_KEY_KEY <<
                                 AVR32_FLASHC_FCMD_KEY_OFFSET) |
                                ((uint32_t) i <<
                                 AVR32_FLASHC_FCMD_PAGEN_OFFSET) |
                                (((fuses & 1) ? AVR32_FLASHC_FCMD_CMD_EGPB :
                                                AVR32_FLASHC_FCMD_CMD_WGPB) <<
                                 AVR32_FLASHC_FCMD_CMD_OFFSET);
            fuses >>= 1;
            fusesDelta >>= 1;
        }
        /* these are for flashc errata */
        commands[j++] = (AVR32_FLASHC_FCMD_KEY_KEY <<
                         AVR32_FLASHC_FCMD_KEY_OFFSET) |
                        (AVR32_FLASHC_FCMD_CMD_CPB <<
                         AVR32_FLASHC_FCMD_CMD_OFFSET);
        commands[j++] = (AVR32_FLASHC_FCMD_KEY_KEY <<
                         AVR32_FLASHC_FCMD_KEY_OFFSET) |
                        /* arbitrary block, page buffer is blank so it won't
                           have any effect */
                        (0x100 <<
                         AVR32_FLASHC_FCMD_PAGEN_OFFSET) |
                        (AVR32_FLASHC_FCMD_CMD_WP <<
                         AVR32_FLASHC_FCMD_CMD_OFFSET);
        stat = doFlashCmds(commands, j);
    } else {
        doFlashCmd((AVR32_FLASHC_FCMD_KEY_KEY <<
                    AVR32_FLASHC_FCMD_KEY_OFFSET) |
                   (AVR32_FLASHC_FCMD_CMD_CPB <<
                    AVR32_FLASHC_FCMD_CMD_OFFSET));
        if (sector == SECTOR_USER_PAGE) {
            flashPtr = (uint32_t *) AVR32_FLASHC_USER_PAGE_ADDRESS;
            stat = doFlashCmd((AVR32_FLASHC_FCMD_KEY_KEY <<
                               AVR32_FLASHC_FCMD_KEY_OFFSET) |
                              (AVR32_FLASHC_FCMD_CMD_EUP <<
                               AVR32_FLASHC_FCMD_CMD_OFFSET));
        } else {
            flashPtr = (uint32_t *) (AVR32_FLASH_ADDRESS +
                                     sector * AVR32_FLASHC_PAGE_SIZE);
            stat = doFlashCmd((AVR32_FLASHC_FCMD_KEY_KEY <<
                               AVR32_FLASHC_FCMD_KEY_OFFSET) |
                              (sector << AVR32_FLASHC_FCMD_PAGEN_OFFSET) |
                              (AVR32_FLASHC_FCMD_CMD_EP <<
                               AVR32_FLASHC_FCMD_CMD_OFFSET));
        }
        if ((stat & (AVR32_FLASHC_FSR_PROGE_MASK |
                     AVR32_FLASHC_FSR_LOCKE_MASK)) == 0) {
            for (i = 0; i < ((sector == SECTOR_USER_PAGE) ?
                             AVR32_FLASHC_USER_PAGE_SIZE :
                             AVR32_FLASHC_PAGE_SIZE) / 4; i++)
                *flashPtr++ = *buffer++;
            if (sector == SECTOR_USER_PAGE)
                stat = doFlashCmd((AVR32_FLASHC_FCMD_KEY_KEY <<
                                   AVR32_FLASHC_FCMD_KEY_OFFSET) |
                                  (AVR32_FLASHC_FCMD_CMD_WUP <<
                                   AVR32_FLASHC_FCMD_CMD_OFFSET));
            else
                stat = doFlashCmd((AVR32_FLASHC_FCMD_KEY_KEY <<
                                   AVR32_FLASHC_FCMD_KEY_OFFSET) |
                                  (sector << AVR32_FLASHC_FCMD_PAGEN_OFFSET) |
                                  (AVR32_FLASHC_FCMD_CMD_WP <<
                                   AVR32_FLASHC_FCMD_CMD_OFFSET));
        }
    }

    return (stat & (AVR32_FLASHC_FSR_PROGE_MASK |
                    AVR32_FLASHC_FSR_LOCKE_MASK)) ? 1 : 0;
}

int flashSubmitFWBuffer(unsigned char *buffer, int length) {
    int sector;
    int suffixSector;
    uint8_t *flashPtr;
    int i;
    int size;
    int last;

    last = (length == 0);
    while (length && flash.errorCode == HF_LOADER_STATUS_OK) {
        switch (flash.state) {
        case headerFS:
            i = sizeof(flash.blockHeader) - flash.index;
            if (i > length)
                i = length;
            memcpy((char *) &flash.blockHeader + flash.index, buffer, i);
            buffer += i;
            flash.index += i;
            length -= i;
            if (flash.index == sizeof(flash.blockHeader)) {
                flash.index = 0;
                findSectorOffset(flash.blockHeader.addr,
                                 &flash.sector, &flash.offset);
                if (flash.blockHeader.magic != HF_LOADER_BLOCK_MAGIC)
                    flash.errorCode = HF_LOADER_STATUS_ERR_TARGET;
                else if (flash.blockHeader.length &&
                         (flash.blockHeader.addr <
                          (uint32_t) _FlashAppStart ||
                          flash.blockHeader.addr + flash.blockHeader.length >
                          ((uint32_t) _FlashAppStart +
                           (uint32_t) _FlashAppLength)) &&
                         flash.sector != SECTOR_USER_PAGE &&
                         flash.sector != SECTOR_FUSES)
                    flash.errorCode = HF_LOADER_STATUS_ERR_ADDRESS;
                else if (flash.blockHeader.length) {
                    findSectorOffset(flash.blockHeader.addr +
                                     flash.blockHeader.length - 1,
                                     &sector, NULL);
                    if (flash.sector != sector)
                        flash.errorCode = HF_LOADER_STATUS_ERR_ADDRESS;
                    else {
                        /* preread into buffer from flash in case we're
                           only writing a partial sector */
                        if (flash.sector == SECTOR_USER_PAGE)
                            size = AVR32_FLASHC_USER_PAGE_SIZE;
                        else if (flash.sector == SECTOR_FUSES)
                            size = 4;
                        else
                            size = AVR32_FLASHC_PAGE_SIZE;
                        memcpy(flash.buffer,
                               (char *) flash.blockHeader.addr,
                               size);
                        flash.state = dataFS;
                        flash.index = 0;
                    }
                }
            }
            break;
        case dataFS:
            i = flash.blockHeader.length - flash.index;
            if (i > length)
                i = length;
            memcpy((char *) flash.buffer + flash.offset + flash.index, buffer,
                   i);
            buffer += i;
            flash.index += i;
            length -= i;
            if (flash.index == flash.blockHeader.length) {
                if (flash.sector != SECTOR_USER_PAGE &&
                    flash.sector != SECTOR_FUSES &&
                    !flash.appSpaceWritten) {
                    flash.appSpaceWritten = 1;
                    findSectorOffset((uint32_t) _FlashAppStart +
                                     (uint32_t) _FlashAppLength -
                                     sizeof(hfLoaderAppSuffixT),
                                     &suffixSector, NULL);
                    doFlashCmd((AVR32_FLASHC_FCMD_KEY_KEY <<
                                AVR32_FLASHC_FCMD_KEY_OFFSET) |
                               (suffixSector <<
                                AVR32_FLASHC_FCMD_PAGEN_OFFSET) |
                               (AVR32_FLASHC_FCMD_CMD_EP <<
                                AVR32_FLASHC_FCMD_CMD_OFFSET));
                }
                if (doEraseWrite(flash.sector, flash.buffer))
                    flash.errorCode = HF_LOADER_STATUS_ERR_PROGRAM;
                else {
                    if (flash.sector == SECTOR_FUSES) {
                        flashPtr = (uint8_t *) &AVR32_FLASHC.fgpfrlo;
                        size = 4;
                    } else if (flash.sector == SECTOR_USER_PAGE) {
                        flashPtr = (uint8_t *) AVR32_FLASHC_USER_PAGE_ADDRESS;
                        size = AVR32_FLASHC_USER_PAGE_SIZE;
                    } else {
                        flashPtr = (uint8_t *) (AVR32_FLASH_ADDRESS +
                                                flash.sector *
                                                AVR32_FLASHC_PAGE_SIZE);
                        size = AVR32_FLASHC_PAGE_SIZE;
                    }
                    if (memcmp(flashPtr, flash.buffer,
                               size))
                        flash.errorCode = HF_LOADER_STATUS_ERR_VERIFY;
                    else {
                        flash.state = headerFS;
                        flash.index = 0;
                    }
                }
            }
            break;
        }
    }

    if (last && flash.errorCode == HF_LOADER_STATUS_OK) {
        if (flash.state != headerFS || flash.index != 0)
            flash.errorCode = HF_LOADER_STATUS_ERR_NOT_DONE;
        else if (flash.appSpaceWritten && flashVerifyApp() == NULL) {
            flash.errorCode = HF_LOADER_STATUS_ERR_FIRMWARE;
            findSectorOffset((uint32_t) _FlashAppStart +
                             (uint32_t) _FlashAppLength -
                             sizeof(hfLoaderAppSuffixT),
                             &suffixSector, NULL);
            doFlashCmd((AVR32_FLASHC_FCMD_KEY_KEY <<
                        AVR32_FLASHC_FCMD_KEY_OFFSET) |
                       (suffixSector << AVR32_FLASHC_FCMD_PAGEN_OFFSET) |
                       (AVR32_FLASHC_FCMD_CMD_EP <<
                        AVR32_FLASHC_FCMD_CMD_OFFSET));
        }
    }

    return flash.errorCode;
}

