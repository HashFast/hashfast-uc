/* twicomms.c */

/*
    Copyright (c) 2013 HashFast Technologies LLC
*/

#include <string.h>
#include <stdint.h>
#include <avr32/io.h>

#include "config.h"
#include "hf_loader.h"
#include "hf_loader_p.h"
#include "version.h"
#include "interrupts.h"
#include "chain.h"
#include "gpio.h"
#include "timers.h"
#include "utils.h"
#include "flash.h"
#include "twi.h"
#include "twicomms.h"

#if 0 /*BISON*/
#include "usbdev.h"
#endif

#define SLAVE_FLAG_BUSY                    0x01
#define SLAVE_FLAG_BEGIN                   0x02
#define SLAVE_FLAG_END                     0x04
#define SLAVE_FLAG_VERSION_SET             0x08
#define SLAVE_FLAG_CRC_VALID               0x10
#define SLAVE_FLAG_SERIAL_NUMBER_SET       0x20
#define SLAVE_FLAG_SIZE_SET                0x40

#ifndef MAX
#define MAX(x,y)   ((x) > (y) ? (x) : (y))
#endif


/* linker defined symbols */
extern char _FlashAppStart[];
extern char _FlashAppLength[];



static struct {
    int enable;
    twiConfigT config;
    enum {initTCS,
          powerDownSlavesWaitTCS,
          powerUpSlavesTCS, powerUpSlavesWaitTCS,
          setAddressTCS, setAddressWaitTCS, setAddressCheckEndTCS,
          addressingCompleteWaitTCS,
          idleMasterTCS, idleSlaveTCS,
          rebootSlavesWaitTCS, rebootSendWaitTCS, remoteCommandWaitTCS,
          remoteSerialNumberWaitTCS, remoteVersionWaitTCS, remoteSizeWaitTCS,
          remoteStatusWaitTCS} state;
    volatile int addressSet;
    uint8_t slaveFlags[CONFIG_MAX_SLAVES];
    uint32_t slaveVersions[CONFIG_MAX_SLAVES];
    uint32_t slaveCRCs[CONFIG_MAX_SLAVES];
    uint32_t slaveSizes[CONFIG_MAX_SLAVES];
    uint32_t slaveCmdSizes[CONFIG_MAX_SLAVES];
    uint8_t slaveSerialNumbers[CONFIG_MAX_SLAVES][CONFIG_SERIAL_NUMBER_SIZE];
    uint8_t rxBuffer[TWICOMMS_MAX_DATA_PAYLOAD + 4];
    uint8_t txBuffer[TWICOMMS_MAX_DATA_PAYLOAD + 4];
    uint8_t flashBuffer[TWICOMMS_MAX_DATA_PAYLOAD + 4];
    const uint8_t *remoteBuffers[CONFIG_MAX_SLAVES];
    unsigned int remoteBufferLengths[CONFIG_MAX_SLAVES];
    int remoteStatus[CONFIG_MAX_SLAVES];
    unsigned int flashBufferIndex;
    volatile int flashFirst;
    volatile int flashLast;
    volatile int flashStatus;
    volatile int addressingComplete;
    volatile int restartAddr;
    unsigned int slaves;
    int poll;
    int delay;
    int reboot;
    int rebootSlave;
    int rebootWatchdog;
} twicomms;


/* this callback is invoked at interrupt time. */
static void twiCallback(unsigned int length) {
    static uint8_t txBuffer[MAX(32, CONFIG_SERIAL_NUMBER_SIZE)];
    hfLoaderAppSuffixT *suffix;
    unsigned int responseLength;
    uint32_t size;

    responseLength = 0;
    if (length) {
#if 0 /*BISON*/
        {
            static int index = 0;
            if (index < 30) {
                usbdevDebugBuffer[index++] = twicomms.rxBuffer[0];
                usbdevDebugBuffer[index++] =
                    (gpioPinValue(CONFIG_GPIO_SPARE_UP) << 7) |
                    (gpioPinValue(CONFIG_GPIO_SPARE_DOWN) << 6) |
                    (gpioPinValue(CONFIG_GPIO_HAVE_USB) << 5) |
                    (gpioPinValue(CONFIG_GPIO_USB_DOWN) << 4);
            }
        }
#endif
        switch (twicomms.rxBuffer[0]) {
        case TWICOMMS_CMD_POWERUP:
            gpioPinSet(CONFIG_GPIO_PWR_ON, 0);
            gpioPinSet(CONFIG_GPIO_SPARE_DOWN, 1);
            gpioPinSet(CONFIG_GPIO_HAVE_USB, 1);
            twicomms.addressingComplete = 0;
            break;
        case TWICOMMS_CMD_POWERDOWN:
            gpioPinSet(CONFIG_GPIO_PWR_ON, 1);
            break;
        case TWICOMMS_CMD_ADDRESS:
            twicomms.addressingComplete = 0;
            if (!gpioPinValue(CONFIG_GPIO_SPARE_UP) &&
                gpioPinValue(CONFIG_GPIO_SPARE_DOWN)) {
                twicomms.config.address = twicomms.rxBuffer[1];
                twicomms.addressSet = 1;
                if (chainState == HF_LOADER_CHAIN_OPEN_UP)
                    gpioPinSet(CONFIG_GPIO_HAVE_USB, 0);
                else
                    gpioPinSet(CONFIG_GPIO_SPARE_DOWN, 0);
            }
            break;
        case TWICOMMS_CMD_ADDRESSING_COMPLETE:
            gpioPinSet(CONFIG_GPIO_SPARE_DOWN, 1);
            gpioPinSet(CONFIG_GPIO_HAVE_USB, 1);
            twicomms.addressingComplete = 1;
            break;
        case TWICOMMS_CMD_VERSION:
            /* high bit indicates loader mode; next 15 bits reserved for
               items such as hardware type and rev */
            txBuffer[0] = 0x80;
            txBuffer[1] = 0x00;
            txBuffer[2] = VERSION >> 8;
            txBuffer[3] = VERSION & 0xff;
            suffix = (hfLoaderAppSuffixT *) ((uint32_t) _FlashAppStart +
                                             (uint32_t) _FlashAppLength -
                                             sizeof(hfLoaderAppSuffixT));
            if (suffix->magic == HF_LOADER_SUFFIX_MAGIC) {
                txBuffer[4] = 1;
                txBuffer[5] = (suffix->crc >> 24) & 0xff;
                txBuffer[6] = (suffix->crc >> 16) & 0xff;
                txBuffer[7] = (suffix->crc >> 8) & 0xff;
                txBuffer[8] = (suffix->crc >> 0) & 0xff;
                responseLength = 9;
            } else {
                txBuffer[4] = 0;
                responseLength = 5;
            }
            break;
        case TWICOMMS_CMD_SIZE:
            /* high bit indicates loader mode */
            size = AVR32_FLASH_SIZE;
            txBuffer[0] = size >> 24;
            txBuffer[1] = (size >> 16) & 0xff;
            txBuffer[2] = (size >> 8) & 0xff;
            txBuffer[3] = size & 0xff;
            size = 0;
            size = flashSize();
            txBuffer[4] = size >> 24;
            txBuffer[5] = (size >> 16) & 0xff;
            txBuffer[6] = (size >> 8) & 0xff;
            txBuffer[7] = size & 0xff;
            responseLength = 8;
            break;
        case TWICMD_SERIAL_NUMBER:
            memcpy(txBuffer, (void *) CONFIG_SERIAL_NUMBER_ADDRESS,
                   CONFIG_SERIAL_NUMBER_SIZE);
            responseLength = CONFIG_SERIAL_NUMBER_SIZE;
            break;
        case TWICOMMS_CMD_REBOOT:
            utilsReboot(0, twicomms.rxBuffer[1] ?
                           UTILS_REBOOT_MODE_LOADER :
                           UTILS_REBOOT_MODE_APP);
            break;
        case TWICOMMS_CMD_LOADER_START:
            if (twicomms.rxBuffer[1])
                twicomms.flashFirst = 1;
            else
                twicomms.flashLast = 1;
            break;
        case TWICOMMS_CMD_LOADER_DATA:
            if (twicomms.flashBufferIndex ||
                length -1 > sizeof(twicomms.flashBuffer)) {
                if (twicomms.flashStatus == HF_LOADER_STATUS_OK)
                    twicomms.flashStatus = HF_LOADER_STATUS_ERR_OVERRUN;
            } else {
                memcpy(twicomms.flashBuffer, &twicomms.rxBuffer[1],
                       length - 1);
                twicomms.flashBufferIndex = length - 1;
            }
            break;
        case TWICOMMS_CMD_LOADER_STATUS:
            if (twicomms.flashBufferIndex || twicomms.flashLast)
                txBuffer[0] = HF_LOADER_STATUS_BUSY;
            else
                txBuffer[0] = twicomms.flashStatus;
            responseLength = 1;
            break;
        //default:
        //    txBuffer[0] = 0;
        //    responseLength = 1;
        }
    }
    twiSlaveSetTx(txBuffer, responseLength);
}

void twicommsInit(void) {

    twicomms.state = initTCS;
    twicomms.addressSet = 0;
    twicomms.config.freq = CONFIG_TWI_SPEED;
    twicomms.config.slaveRxBuffer = twicomms.rxBuffer;
    twicomms.config.slaveRxBufferSize = sizeof(twicomms.rxBuffer);
    twicomms.config.callback = twiCallback;
    twicomms.config.master = 0;
    twicomms.config.address = TWICOMMS_SLAVE_STARTADDR;
    twiConfig(&twicomms.config);
}

void twicommsEnable(int enable) {

    twicomms.enable = enable;
    if (enable)
        twicomms.rebootWatchdog = 0;
    else if (chainMaster)
        twicomms.rebootWatchdog = 1000;
}

void twicommsRestartAddr(int rebootSlaves) {

    twicomms.restartAddr = rebootSlaves ? 2 : 1;
}

int twicommsNumSlaves(uint8_t *slaves) {

    *slaves = twicomms.slaves;

    return twicomms.addressingComplete;
}

uint32_t twicommsSlaveVersion(int slave) {

    return twicomms.slaveVersions[slave];
}

int twicommsSlaveCRC(int slave, uint32_t *crc) {

    *crc = twicomms.slaveCRCs[slave];

    return (twicomms.slaveFlags[slave] & SLAVE_FLAG_CRC_VALID) ? 1 : 0;
}

void twicommsSlaveSerialNumber(int slave, uint8_t *sn) {

    memcpy(sn, &twicomms.slaveSerialNumbers[slave][0],
           CONFIG_SERIAL_NUMBER_SIZE);
}

uint32_t twicommsSlaveSize(int slave) {

    return twicomms.slaveSizes[slave];
}

uint32_t twicommsSlaveCmdSize(int slave) {

    return twicomms.slaveCmdSizes[slave];
}

void twicommsRebootRequest(int loader, int slave) {

    twicomms.reboot = loader ? 2 : 1;
    twicomms.rebootSlave = slave;
    if (slave == 0xff)
        twicomms.rebootWatchdog = 500;
}

void twicommsUploadBegin(int slave) {

    twicomms.slaveFlags[slave] |= SLAVE_FLAG_BEGIN;
    twicomms.remoteStatus[slave] = HF_LOADER_STATUS_OK;
}

void twicommsUpload(int slave, const void *data, unsigned int length) {

    if ((length > (sizeof(twicomms.txBuffer) - 1)) ||
        twicomms.remoteBufferLengths[slave] ||
        (twicomms.slaveFlags[slave] & SLAVE_FLAG_BUSY)) {
        if (twicomms.remoteStatus[slave] == HF_LOADER_STATUS_OK)
            twicomms.remoteStatus[slave] = HF_LOADER_STATUS_ERR_OVERRUN;
    } else if (twicomms.remoteStatus[slave] == HF_LOADER_STATUS_OK) {
        twicomms.remoteBuffers[slave] = data;
        twicomms.remoteBufferLengths[slave] = length;
    }
}

void twicommsUploadEnd(int slave) {

    twicomms.slaveFlags[slave] |= SLAVE_FLAG_END;
}

int twicommsUploadStatus(int slave) {
    int status;

    if (twicomms.remoteBufferLengths[slave] ||
        (twicomms.slaveFlags[slave] & (SLAVE_FLAG_BUSY | SLAVE_FLAG_END)))
        status = HF_LOADER_STATUS_BUSY;
    else
        status = twicomms.remoteStatus[slave];

    return status;
}

void twicommsTask(void) {
    static unsigned int lastTick;
    unsigned int elapsedTime;
    int status;

    elapsedTime = timersTick - lastTick;
    lastTick += elapsedTime;
    if (twicomms.delay <= elapsedTime)
        twicomms.delay = 0;
    else
        twicomms.delay -= elapsedTime;
    if (twicomms.rebootWatchdog) {
        if (twicomms.rebootWatchdog > elapsedTime)
            twicomms.rebootWatchdog -= elapsedTime;
        else
            utilsReboot(0, (twicomms.reboot == 2) ?
                            UTILS_REBOOT_MODE_LOADER :
                            UTILS_REBOOT_MODE_APP);
    }

    if (chainState == HF_LOADER_CHAIN_MIDDLE && !twicomms.addressingComplete)
        gpioPinSet(CONFIG_GPIO_HAVE_USB, gpioPinValue(CONFIG_GPIO_USB_DOWN));

    if (twicomms.addressSet) {
#if 0 /*BISON*/
        usbdevDebugBuffer[62] = twicomms.config.address;
#endif
        twicomms.addressSet = 0;
        twiConfig(&twicomms.config);
    }
#if 0 /*BISON*/
    usbdevDebugBuffer[63] = twicomms.state;
#endif
    if (twicomms.delay == 0) {
        switch (twicomms.state) {
        case initTCS:
            if (chainState != HF_LOADER_CHAIN_UNCONFIGURED) {
                if (chainMaster && twicomms.enable) {
                    twicomms.config.master = 1;
                    twicomms.config.address = TWICOMMS_MASTER_ADDR; /* unused */
                    twiConfig(&twicomms.config);
                    twicomms.slaves = 0;
                    twicomms.addressingComplete = 0;
                    memset(twicomms.slaveFlags, 0,
                           sizeof(twicomms.slaveFlags));
                    memset(twicomms.slaveVersions, 0,
                           sizeof(twicomms.slaveVersions));
                    memset(twicomms.slaveCRCs, 0,
                           sizeof(twicomms.slaveCRCs));
                    memset(&twicomms.slaveSerialNumbers[0][0], 0,
                           sizeof(twicomms.slaveSerialNumbers));
                    if (chainState == HF_LOADER_CHAIN_NONE ||
                        chainState == HF_LOADER_CHAIN_LOOPBACK) {
                        twicomms.addressingComplete = 1;
                        twicomms.state = idleMasterTCS;
                    } else {
                        gpioPinSet(CONFIG_GPIO_PWR_ON, 0);
                        twicomms.delay = 1200;
                        twicomms.state = powerUpSlavesTCS;
                    }
                } else if (!chainMaster)
                    twicomms.state = idleSlaveTCS;
            }
            break;
        case powerUpSlavesTCS:
            twiReset();
            twiConfig(&twicomms.config);
            twicomms.txBuffer[0] = TWICOMMS_CMD_POWERUP;
            twicomms.txBuffer[1] = 0;
            if (twiMasterWriteRead(0, twicomms.txBuffer, 2, 0, 0) ==
                TWI_SUCCESS)
                twicomms.state = powerUpSlavesWaitTCS;
            break;
        case powerUpSlavesWaitTCS:
            status = twiStatus();
            if (status != TWI_BUSY) {
                twicomms.delay = 1200;
                twicomms.state = setAddressTCS;
            }
            break;
        case setAddressTCS:
            gpioPinSet(CONFIG_GPIO_SPARE_DOWN, 0);
            twicomms.txBuffer[0] = TWICOMMS_CMD_ADDRESS;
            twicomms.txBuffer[1] = TWICOMMS_SLAVE_STARTADDR + twicomms.slaves;
            if (twiMasterWriteRead(0, twicomms.txBuffer, 2, 0, 0) ==
                TWI_SUCCESS)
                twicomms.state = setAddressWaitTCS;
            break;
        case setAddressWaitTCS:
            status = twiStatus();
            if (status != TWI_BUSY) {
                twicomms.slaves++;
                twicomms.delay = 3;
                twicomms.state = setAddressCheckEndTCS;
            }
            break;
        case setAddressCheckEndTCS:
            if (twicomms.slaves == CONFIG_MAX_SLAVES ||
                !gpioPinValue(CONFIG_GPIO_USB_DOWN)) {
                /* address cycle done */
                twicomms.txBuffer[0] = TWICOMMS_CMD_ADDRESSING_COMPLETE;
                twicomms.txBuffer[1] = twicomms.slaves;
                if (twiMasterWriteRead(0, twicomms.txBuffer, 2, 0, 0) ==
                    TWI_SUCCESS)
                    twicomms.state = addressingCompleteWaitTCS;
            } else
                twicomms.state = setAddressTCS;
            break;
        case addressingCompleteWaitTCS:
            status = twiStatus();
            if (status != TWI_BUSY) {
                twicomms.addressingComplete = 1;
                twicomms.state = idleMasterTCS;
            }
            break;
        case idleMasterTCS:
            if (twicomms.reboot) {
                twicomms.txBuffer[0] = TWICOMMS_CMD_REBOOT;
                twicomms.txBuffer[1] = (twicomms.reboot == 2) ? 1 : 0;
                if (twiMasterWriteRead((twicomms.rebootSlave == 0xff) ?
                                       0 : TWICOMMS_SLAVE_STARTADDR +
                                           twicomms.rebootSlave,
                                       twicomms.txBuffer, 2, 0, 0) ==
                    TWI_SUCCESS) {
                    twicomms.state = rebootSendWaitTCS;
                    twicomms.reboot = 0;
                }
            } else if (!twicomms.enable) {
                twicomms.txBuffer[0] = TWICOMMS_CMD_POWERDOWN;
                twicomms.txBuffer[1] = 0;
                if (twiMasterWriteRead(0, twicomms.txBuffer, 2, 0, 0) ==
                    TWI_SUCCESS)
                    twicomms.state = powerDownSlavesWaitTCS;
                break;
            } else if (twicomms.restartAddr) {
                if (twicomms.restartAddr > 1) {
                    twicomms.txBuffer[0] = TWICOMMS_CMD_REBOOT;
                    twicomms.txBuffer[1] = 1;
                    if (twiMasterWriteRead(0, twicomms.txBuffer, 2, 0, 0) ==
                        TWI_SUCCESS)
                        twicomms.state = rebootSlavesWaitTCS;
                } else
                    twicomms.state = initTCS;
                twicomms.restartAddr = 0;
            } else if (twicomms.slaves) {
                if (++twicomms.poll >= twicomms.slaves)
                    twicomms.poll = 0;
                if (twicomms.slaveFlags[twicomms.poll] & SLAVE_FLAG_BUSY) {
                    twicomms.txBuffer[0] = TWICOMMS_CMD_LOADER_STATUS;
                    if (twiMasterWriteRead(TWICOMMS_SLAVE_STARTADDR +
                                           twicomms.poll,
                                           twicomms.txBuffer, 1,
                                           twicomms.rxBuffer, 1) ==
                        TWI_SUCCESS)
                        twicomms.state = remoteStatusWaitTCS;
                } else if (!(twicomms.slaveFlags[twicomms.poll] &
                             SLAVE_FLAG_SERIAL_NUMBER_SET)) {
                    twicomms.txBuffer[0] = TWICMD_SERIAL_NUMBER;
                    if (twiMasterWriteRead(TWICOMMS_SLAVE_STARTADDR +
                                           twicomms.poll,
                                           twicomms.txBuffer, 1,
                                           twicomms.rxBuffer,
                                           CONFIG_SERIAL_NUMBER_SIZE) ==
                        TWI_SUCCESS)
                        twicomms.state = remoteSerialNumberWaitTCS;

                } else if (!(twicomms.slaveFlags[twicomms.poll] &
                           SLAVE_FLAG_VERSION_SET)) {
                    twicomms.txBuffer[0] = TWICOMMS_CMD_VERSION;
                    if (twiMasterWriteRead(TWICOMMS_SLAVE_STARTADDR +
                                           twicomms.poll,
                                           twicomms.txBuffer, 1,
                                           twicomms.rxBuffer, 9) ==
                        TWI_SUCCESS)
                        twicomms.state = remoteVersionWaitTCS;
                } else if (!(twicomms.slaveFlags[twicomms.poll] & SLAVE_FLAG_SIZE_SET)) {
                    if((twicomms.slaveVersions[twicomms.poll] & 0x0000FFFF) >= 0x00000004) {
                        twicomms.txBuffer[0] = TWICOMMS_CMD_SIZE;
                        if (twiMasterWriteRead(TWICOMMS_SLAVE_STARTADDR +
                                               twicomms.poll,
                                               twicomms.txBuffer, 1,
                                               twicomms.rxBuffer, 8) ==
                            TWI_SUCCESS)
                            twicomms.state = remoteSizeWaitTCS;
                    } else {
                        twicomms.slaveFlags[twicomms.poll] |= SLAVE_FLAG_SIZE_SET;
                        twicomms.state = idleMasterTCS;
                    }
                } else if (twicomms.slaveFlags[twicomms.poll] &
                           SLAVE_FLAG_BEGIN) {
                    twicomms.txBuffer[0] = TWICOMMS_CMD_LOADER_START;
                    twicomms.txBuffer[1] = 1;
                    if (twiMasterWriteRead(TWICOMMS_SLAVE_STARTADDR +
                                           twicomms.poll,
                                           twicomms.txBuffer, 2,
                                           0, 0) ==
                        TWI_SUCCESS) {
                        twicomms.slaveFlags[twicomms.poll] &=
                            ~(SLAVE_FLAG_BEGIN | SLAVE_FLAG_CRC_VALID);
                        twicomms.state = remoteCommandWaitTCS;
                    }
                } else if (twicomms.remoteBufferLengths[twicomms.poll]) {
                    twicomms.txBuffer[0] = TWICOMMS_CMD_LOADER_DATA;
                    memcpy(&twicomms.txBuffer[1],
                           twicomms.remoteBuffers[twicomms.poll],
                           twicomms.remoteBufferLengths[twicomms.poll]);
                    if (twiMasterWriteRead(
                            TWICOMMS_SLAVE_STARTADDR +
                            twicomms.poll,
                            twicomms.txBuffer,
                            twicomms.remoteBufferLengths[twicomms.poll] + 1,
                            0, 0) ==
                        TWI_SUCCESS) {
                        twicomms.slaveFlags[twicomms.poll] |= SLAVE_FLAG_BUSY;
                        twicomms.state = remoteCommandWaitTCS;
                        twicomms.remoteBufferLengths[twicomms.poll] = 0;
                    }
                } else if (twicomms.slaveFlags[twicomms.poll] &
                           SLAVE_FLAG_END) {
                    twicomms.txBuffer[0] = TWICOMMS_CMD_LOADER_START;
                    twicomms.txBuffer[1] = 0;
                    if (twiMasterWriteRead(TWICOMMS_SLAVE_STARTADDR +
                                           twicomms.poll,
                                           twicomms.txBuffer, 2,
                                           0, 0) ==
                        TWI_SUCCESS) {
                        twicomms.slaveFlags[twicomms.poll] |=
                            SLAVE_FLAG_BUSY;
                        twicomms.slaveFlags[twicomms.poll] &=
                            ~(SLAVE_FLAG_END | SLAVE_FLAG_VERSION_SET);
                        twicomms.state = remoteCommandWaitTCS;
                    }
                }
            }
            break;
        case remoteCommandWaitTCS:
            status = twiStatus();
            if (status != TWI_BUSY) {
                twicomms.state = idleMasterTCS;
            }
            break;
        case rebootSlavesWaitTCS:
            status = twiStatus();
            if (status != TWI_BUSY) {
                twicomms.state = initTCS;
                twicomms.delay = 500;
            }
            break;
        case powerDownSlavesWaitTCS:
            status = twiStatus();
            if (status != TWI_BUSY) {
                gpioPinSet(CONFIG_GPIO_PWR_ON, 1);
                twicomms.state = initTCS;
                twicomms.delay = 500;
                twicomms.rebootWatchdog = 0;
            }
            break;
        case rebootSendWaitTCS:
            status = twiStatus();
            if (status != TWI_BUSY) {
                if (twicomms.rebootSlave == 0xff) {
                    utilsReboot(0, (twicomms.reboot == 2) ?
                                    UTILS_REBOOT_MODE_LOADER :
                                    UTILS_REBOOT_MODE_APP);
                } else {
                    twicomms.delay = 500;
                    twicomms.state = initTCS;
                }
            }
            break;
        case remoteSerialNumberWaitTCS:
            status = twiStatus();
            if (status != TWI_BUSY) {
                if (status == TWI_SUCCESS) {
                    memcpy(&twicomms.slaveSerialNumbers[twicomms.poll][0],
                           &twicomms.rxBuffer[0],
                           CONFIG_SERIAL_NUMBER_SIZE);
                    twicomms.slaveFlags[twicomms.poll] |=
                        SLAVE_FLAG_SERIAL_NUMBER_SET;
                }
                twicomms.state = idleMasterTCS;
            }
            break;
        case remoteVersionWaitTCS:
            status = twiStatus();
            if (status != TWI_BUSY) {
                if (status == TWI_SUCCESS) {
                    twicomms.slaveVersions[twicomms.poll] =
                        ((uint32_t) twicomms.rxBuffer[0] << 24) |
                        ((uint32_t) twicomms.rxBuffer[1] << 16) |
                        ((uint32_t) twicomms.rxBuffer[2] << 8) |
                        twicomms.rxBuffer[3];
                    if (twicomms.rxBuffer[4] == 0x01) {
                        twicomms.slaveCRCs[twicomms.poll] =
                            ((uint32_t) twicomms.rxBuffer[5] << 24) |
                            ((uint32_t) twicomms.rxBuffer[6] << 16) |
                            ((uint32_t) twicomms.rxBuffer[7] << 8) |
                            twicomms.rxBuffer[8];
                        twicomms.slaveFlags[twicomms.poll] |=
                            SLAVE_FLAG_CRC_VALID;
                    }
                    twicomms.slaveFlags[twicomms.poll] |=
                        SLAVE_FLAG_VERSION_SET;
                }
                twicomms.state = idleMasterTCS;
            }
            break;
        case remoteSizeWaitTCS:
            status = twiStatus();
            if (status != TWI_BUSY) {
                if (status == TWI_SUCCESS) {
                    twicomms.slaveSizes[twicomms.poll] =
                        ((uint32_t) twicomms.rxBuffer[0] << 24) |
                        ((uint32_t) twicomms.rxBuffer[1] << 16) |
                        ((uint32_t) twicomms.rxBuffer[2] << 8) |
                        twicomms.rxBuffer[3];
                    twicomms.slaveCmdSizes[twicomms.poll] =
                        ((uint32_t) twicomms.rxBuffer[4] << 24) |
                        ((uint32_t) twicomms.rxBuffer[5] << 16) |
                        ((uint32_t) twicomms.rxBuffer[6] << 8) |
                        twicomms.rxBuffer[7];
                    twicomms.slaveFlags[twicomms.poll] |=
                        SLAVE_FLAG_SIZE_SET;
                }
                twicomms.state = idleMasterTCS;
            }
            break;
        case remoteStatusWaitTCS:
            status = twiStatus();
            if (status != TWI_BUSY) {
                if (status == TWI_SUCCESS) {
                    if (twicomms.rxBuffer[0] != HF_LOADER_STATUS_BUSY) {
                        if (!(twicomms.slaveFlags[twicomms.poll] &
                              SLAVE_FLAG_BEGIN))
                            twicomms.remoteStatus[twicomms.poll] =
                                twicomms.rxBuffer[0];
                        twicomms.slaveFlags[twicomms.poll] &= ~SLAVE_FLAG_BUSY;
                    }
                }
                twicomms.state = idleMasterTCS;
            }
            break;
        case idleSlaveTCS:
            interruptsDisable();
            if (twicomms.flashBufferIndex) {
                interruptsEnable();
                if (twicomms.flashFirst) {
                    twicomms.flashFirst = 0;
                    flashReset();
                    twicomms.flashStatus = HF_LOADER_STATUS_OK;
                }
                if (twicomms.flashStatus == HF_LOADER_STATUS_OK)
                    twicomms.flashStatus = flashSubmitFWBuffer(
                                               twicomms.flashBuffer,
                                               twicomms.flashBufferIndex);
                twicomms.flashBufferIndex = 0;
            } else if (twicomms.flashLast && twicomms.flashBufferIndex == 0) {
                interruptsEnable();
                twicomms.flashLast = 0;
                if (twicomms.flashStatus == HF_LOADER_STATUS_OK)
                    twicomms.flashStatus = flashSubmitFWBuffer(0, 0);
            } else
                interruptsEnable();
            break;
        }
    }
}

