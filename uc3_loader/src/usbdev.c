/* usbdev.c */

/*
    Copyright (c) 2013 HashFast Technologies LLC
*/

#include <string.h>
#include <stdint.h>
#include <avr32/io.h>

#include "config.h"
#include "version.h"
#include "hf_loader.h"
#include "hf_loader_p.h"
#include "interrupts.h"
#include "chain.h"
#include "twicomms.h"
#include "timers.h"
#include "flash.h"
#include "utils.h"
#include "usbb_device.h"
#include "udc.h"
#include "usbb_otg.h"
#include "usbdev.h"



#ifndef MAX
#define MAX(a,b)   ((a) > (b) ? (a) : (b))
#endif


/* linker defined symbols */
extern char _FlashAppStart[];
extern char _FlashAppLength[];


uint8_t usbdevDebugBuffer[USBDEV_CONTROL_EP_SIZE];

#if USBDEV_CONTROL_EP_SIZE < CONFIG_SERIAL_NUMBER_SIZE
#error
#endif

static struct {
    enum {detachedUDS, attachedUDS, rebootUDS} state;
    unsigned int delay;
    uint8_t setupPayload[USBDEV_CONTROL_EP_SIZE];
    uint8_t bulkData[MAX(UDI_VENDOR_EPS_SIZE_BULK_FS,
                         UDI_VENDOR_EPS_SIZE_BULK_HS)];
    int flashStatus;
    volatile int reboot;
    volatile int rebootModule;
    volatile int dataEnable;
    volatile int bulkDataLength;
    volatile int bulkBufferFilled;
    volatile int bulkBufferBusy;
    volatile int uploadBegin;
    volatile int uploadEnd;
    volatile int uploadTarget;
} usbdev;


/* called at interrupt time */
static void bulkDataCallback(udd_ep_status_t status, iram_size_t transferred,
                             udd_ep_id_t ep) {

    if (status == UDD_EP_TRANSFER_OK && usbdev.dataEnable) {
        usbdev.bulkDataLength = transferred;
        usbdev.bulkBufferFilled = 1;
    }
}

void usbdevInit(void) {

    usbdev.state = detachedUDS;
    usbdev.delay = 500;
    udc_start();
}

void usbdevTask(void) {
    static unsigned int lastTick;
    unsigned int elapsedTicks;

    elapsedTicks = timersTick - lastTick;
    lastTick += elapsedTicks;

    if (usbdev.delay > elapsedTicks)
        usbdev.delay -= elapsedTicks;
    else
        usbdev.delay = 0;

    if (usbdev.reboot && usbdev.state != rebootUDS) {
        /* give plenty of time for setup transaction containing reboot
           command to finish to kee host happy */
        usbdev.delay = 10;
        usbdev.state = rebootUDS;
    }
    if (usbdev.delay == 0) {
        switch (usbdev.state) {
        case detachedUDS:
            if (Is_otg_vbus_high()) {
                udd_attach();
                usbdev.state = attachedUDS;
                twicommsEnable(1);
            }
            break;
        case attachedUDS:
            if (!Is_otg_vbus_high()) {
                udd_detach();
                usbdev.delay = 500;
                usbdev.dataEnable = 0;
                usbdev.bulkBufferFilled = 0;
                usbdev.uploadBegin = 0;
                usbdev.uploadEnd = 0;
                usbdev.state = detachedUDS;
                twicommsEnable(0);
            } else {
                if (usbdev.bulkBufferBusy &&
                    twicommsUploadStatus(usbdev.uploadTarget - 1) !=
                    HF_LOADER_STATUS_BUSY) {
                    usbdev.bulkBufferBusy = 0;
                    usbdev.bulkBufferFilled = 0;
                    if (usbdev.dataEnable)
                        udi_vendor_bulk_out_run(usbdev.bulkData,
                                                sizeof(usbdev.bulkData),
                                                bulkDataCallback);
                }
                interruptsDisable();
                if (usbdev.uploadBegin) {
                    interruptsEnable();
                    if (usbdev.uploadTarget == 0) {
                        flashReset();
                        usbdev.flashStatus = HF_LOADER_STATUS_OK;
                    } else
                        twicommsUploadBegin(usbdev.uploadTarget - 1);
                    usbdev.uploadBegin = 0;
                } else if (usbdev.bulkBufferFilled) {
                    interruptsEnable();
                    if (usbdev.bulkDataLength) {
                        if (usbdev.uploadTarget == 0) {
                            if (usbdev.flashStatus == HF_LOADER_STATUS_OK)
                                usbdev.flashStatus =
                                    flashSubmitFWBuffer(usbdev.bulkData,
                                                        usbdev.bulkDataLength);
                            usbdev.bulkBufferFilled = 0;
                        } else if (twicommsUploadStatus(usbdev.uploadTarget -
                                                        1) !=
                                   HF_LOADER_STATUS_BUSY) {
                            twicommsUpload(usbdev.uploadTarget - 1,
                                           usbdev.bulkData,
                                           usbdev.bulkDataLength);
                            usbdev.bulkBufferBusy = 1;
                        }
                    } else
                        usbdev.bulkBufferFilled = 0;
                    if (!usbdev.bulkBufferFilled && usbdev.dataEnable)
                        udi_vendor_bulk_out_run(usbdev.bulkData,
                                                sizeof(usbdev.bulkData),
                                                bulkDataCallback);
                } else if (usbdev.uploadEnd && !usbdev.bulkBufferBusy &&
                           !udd_nb_busy_bank(UDI_VENDOR_EP_BULK_OUT)) {
                    interruptsEnable();
                    if (usbdev.uploadTarget == 0) {
                        if (usbdev.flashStatus == HF_LOADER_STATUS_OK)
                            usbdev.flashStatus = flashSubmitFWBuffer(0, 0);
                    } else
                        twicommsUploadEnd(usbdev.uploadTarget - 1);
                    usbdev.uploadEnd = 0;
                } else
                    interruptsEnable();
            }
            break;
        case rebootUDS:
            if (usbdev.rebootModule == 0)
                utilsReboot(0, (usbdev.reboot == 2) ? 
                               UTILS_REBOOT_MODE_LOADER :
                               UTILS_REBOOT_MODE_APP);
            else
                twicommsRebootRequest((usbdev.reboot == 2) ? 1 : 0,
                                      (usbdev.rebootModule == 0xff) ?
                                      0xff : usbdev.rebootModule - 1);
            usbdev.reboot = 0;
            usbdev.state = attachedUDS;
            break;
        }
    }
}


/* called at interrupt time */
int usbdevVendorEnable(void) {

    usbdev.dataEnable = 1;
    udi_vendor_bulk_out_run(usbdev.bulkData, sizeof(usbdev.bulkData),
                            bulkDataCallback);

    return 1;
}

/* called at interrupt time */
void usbdevVendorDisable(void) {

    usbdev.dataEnable = 0;
}

/* called at interrupt time */
int usbdevSetup(void) {
    int result;
    hfLoaderAppSuffixT *suffix;
    uint32_t version;
    uint32_t size;
    int valid;
    int status;

    result = 0;
    if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_TYPE_MASK) ==
        USB_REQ_TYPE_VENDOR) {
        switch (udd_g_ctrlreq.req.bRequest) {
        case HF_LOADER_USB_REBOOT:
            usbdev.rebootModule = udd_g_ctrlreq.req.wIndex;
            usbdev.reboot = udd_g_ctrlreq.req.wValue ? 2 : 1;
            result = 1;
            break;
        case HF_LOADER_USB_RESTART_ADDR:
            twicommsRestartAddr(udd_g_ctrlreq.req.wValue ? 1 : 0);
            result = 1;
            break;
        case HF_LOADER_USB_VERSION:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_IN) {
                if (udd_g_ctrlreq.req.wIndex == 0)
                    version = 0x80000000 | VERSION;
                else
                    version = twicommsSlaveVersion(
                                  (int) udd_g_ctrlreq.req.wIndex - 1);
                usbdev.setupPayload[0] = version & 0xff;
                usbdev.setupPayload[1] = (version >> 8) & 0xff;
                usbdev.setupPayload[2] = (version >> 16) & 0xff;
                usbdev.setupPayload[3] = version >> 24;
                udd_g_ctrlreq.payload_size = 4;
                valid = 0;
                version = 0;
                if (udd_g_ctrlreq.req.wIndex == 0) {
                    suffix = (hfLoaderAppSuffixT *)
                             ((uint32_t) _FlashAppStart +
                              (uint32_t) _FlashAppLength -
                              sizeof(hfLoaderAppSuffixT));
                    if (suffix->magic == HF_LOADER_SUFFIX_MAGIC) {
                        valid = 1;
                    }
                    version = suffix->crc;
                } else
                    valid = twicommsSlaveCRC(
                                (int) udd_g_ctrlreq.req.wIndex - 1,
                                &version);
                usbdev.setupPayload[4] = valid ? 0x01 : 0x00;
                usbdev.setupPayload[5] = version & 0xff;
                usbdev.setupPayload[6] = (version >> 8) & 0xff;
                usbdev.setupPayload[7] = (version >> 16) & 0xff;
                usbdev.setupPayload[8] = version >> 24;
                udd_g_ctrlreq.payload_size = 9;
                udd_g_ctrlreq.payload = usbdev.setupPayload;
                if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                    udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                result = 1;
            }
            break;
        case HF_LOADER_USB_SERIAL:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_IN) {
                if (udd_g_ctrlreq.req.wIndex == 0)
                    memcpy(usbdev.setupPayload,
                           (void *) CONFIG_SERIAL_NUMBER_ADDRESS,
                           CONFIG_SERIAL_NUMBER_SIZE);
                else
                    twicommsSlaveSerialNumber(
                        (int) udd_g_ctrlreq.req.wIndex - 1,
                        usbdev.setupPayload);
                udd_g_ctrlreq.payload = usbdev.setupPayload;
                udd_g_ctrlreq.payload_size = CONFIG_SERIAL_NUMBER_SIZE;
                if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                    udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                result = 1;
            }
            break;
        case HF_LOADER_USB_FLASH_SIZE:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_IN) {
                if (udd_g_ctrlreq.req.wIndex == 0)
                    size = AVR32_FLASH_SIZE;
                else
                    size = twicommsSlaveSize((int) udd_g_ctrlreq.req.wIndex - 1);
                usbdev.setupPayload[0] = size & 0xff;
                usbdev.setupPayload[1] = (size >> 8) & 0xff;
                usbdev.setupPayload[2] = (size >> 16) & 0xff;
                usbdev.setupPayload[3] = size >> 24;
                udd_g_ctrlreq.payload_size = 4;
                size = 0;
                if (udd_g_ctrlreq.req.wIndex == 0)
                    size = flashSize();
                else
                    size = twicommsSlaveCmdSize((int) udd_g_ctrlreq.req.wIndex - 1);
                usbdev.setupPayload[4] = size & 0xff;
                usbdev.setupPayload[5] = (size >> 8) & 0xff;
                usbdev.setupPayload[6] = (size >> 16) & 0xff;
                usbdev.setupPayload[7] = size >> 24;
                udd_g_ctrlreq.payload_size = 8;
                udd_g_ctrlreq.payload = usbdev.setupPayload;
                if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                    udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                result = 1;
            }
            break;
        case HF_LOADER_USB_CONFIG:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_IN) {
                usbdev.setupPayload[0] = (chainState ==
                                          HF_LOADER_CHAIN_UNCONFIGURED) ?
                                         0xff : (chainMaster ? 1 : 0);
                if (twicommsNumSlaves(&usbdev.setupPayload[1]))
                    usbdev.setupPayload[3] = 1;
                else
                    usbdev.setupPayload[3] = 0;
                usbdev.setupPayload[2] = (uint8_t) chainState;
                udd_g_ctrlreq.payload = usbdev.setupPayload;
                udd_g_ctrlreq.payload_size = 4;
                if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                    udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                result = 1;
            }
            break;
        case HF_LOADER_USB_STATUS:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_IN) {
                if (udd_g_ctrlreq.req.wIndex == 0) {
                    if (usbdev.bulkBufferFilled || usbdev.uploadEnd)
                        status = HF_LOADER_STATUS_BUSY;
                    else
                        status = usbdev.flashStatus;
                } else {
                    if (usbdev.bulkBufferFilled || usbdev.bulkBufferBusy ||
                        usbdev.uploadEnd)
                        status = HF_LOADER_STATUS_BUSY;
                    else
                        status = twicommsUploadStatus(
                                     (int) udd_g_ctrlreq.req.wIndex - 1);
                }
                usbdev.setupPayload[0] = status & 0xff;
                usbdev.setupPayload[1] = (status >> 8) & 0xff;
                usbdev.setupPayload[2] = (status >> 16) & 0xff;
                usbdev.setupPayload[3] = (status >> 24) & 0xff;
                udd_g_ctrlreq.payload = usbdev.setupPayload;
                udd_g_ctrlreq.payload_size = 4;
                if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                    udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                result = 1;
            }
            break;
        case HF_LOADER_USB_START:
            usbdev.uploadTarget = (int) udd_g_ctrlreq.req.wIndex;
            usbdev.uploadBegin = 1;
            result = 1;
            break;
        case HF_LOADER_USB_FINISH:
            usbdev.uploadEnd = 1;
            result = 1;
            break;
        case HF_LOADER_USB_DEBUG:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_IN) {
                memcpy(usbdev.setupPayload, usbdevDebugBuffer,
                       USBDEV_CONTROL_EP_SIZE);
                udd_g_ctrlreq.payload = usbdev.setupPayload;
                udd_g_ctrlreq.payload_size = USBDEV_CONTROL_EP_SIZE;
                if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                    udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                result = 1;
            }
            break;
        }
    }

    return result;
}

