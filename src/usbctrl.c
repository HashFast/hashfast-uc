/** @file usbctrl.c
 * @brief USB Control Channel
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

#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <intc.h>
#include <udc.h>

#include "module_handler.h"
#include "da2s.h"
#include "hf_usbctrl.h"
#include "usbctrl.h"
#include "hf_nvram.h"

#define FLAGS_REBOOTING              0x00000001
#define FLAGS_SETTING_NAME           0x00000002
#define FLAGS_POWERING_UP            0x00000004
#define FLAGS_POWERING_DOWN          0x00000008
#define FLAGS_SETTING_FAN_SETTINGS   0x00000010
#define FLAGS_SETTING_OP_SETTINGS    0x00000020
#define FLAGS_ASIC_CTRL              0x00000040
#define FLAGS_MODE_SET               0x00000080

#define DBYTES_PER_DIE   6
#define DBYTES_PER_ASIC 24

uint8_t usbctrlDebugBuffer[64];

/**
 * USB Control Channel Debug Monitor
 */
static struct {
    struct {
        char buffer[128];
        volatile unsigned int head;
        volatile unsigned int tail;
    } tx;
    struct {
        char buffer[128];
        volatile unsigned int head;
        volatile unsigned int tail;
    } rx;
} debugMonitor;

/**
 * USB Control Channel Debug Stream
 */
static struct {
    unsigned int head;
    unsigned int tail;
    unsigned int lost;
    char buffer[1024];
} debugStream;

/**
 * USB Control Channel structure
 */
static struct {
    uint32_t flags;
    uint16_t timer;
    uint8_t module;
    uint8_t status;
    enum {idleS = 0, rebootS} state;
    enum {loaderRB, appRB} rebootMode;
    union {
        char name[HF_NAME_SIZE];
        fan_settings_t fanSettings;
        op_settings_t opSettings;
        uint16_t asicFlags;
        struct {
            char mode;
            uint32_t baud;
        } mode;
    } v;
    char reboot;
    uint16_t rebootModule;
    union {
        uint8_t b[64];
        char name[HF_NAME_SIZE];
        /*
         * Other host tools are already treating serial as if it is a byte array
         * even though the first element is declared as a gawble. We'll continue
         * that treatment through this interface.
         */
        serial_number_t serial;
        /* eight bit fields only - no byte order issues */
        fan_settings_t fanSettings;
    } payload;
} usbctrl;

/**
 * Read a character from the debug monitor.
 * @return
 */
int usbctrlDebugMonitorRead(void) {
    int c;
    unsigned int tail;

    tail = debugMonitor.rx.tail;
    if (tail != debugMonitor.rx.head) {
        c = debugMonitor.rx.buffer[tail] & 0xff;
        if (++tail >= sizeof(debugMonitor.rx.buffer))
            tail = 0;
        debugMonitor.rx.tail = tail;
    } else
        c = -1;

    return c;
}

/**
 * Write a character to the debug monitor.
 * @param c
 * @return
 */
int usbctrlDebugMonitorWrite(char c) {
    unsigned int head;
    int result;

    result = -1;
    head = debugMonitor.tx.head;
    if (++head >= sizeof(debugMonitor.tx.buffer))
        head = 0;
    if (head != debugMonitor.tx.tail) {
        debugMonitor.tx.buffer[debugMonitor.tx.head] = c;
        debugMonitor.tx.head = head;
        result = c & 0xff;
    }

    return result;
}

/**
 * Write a string to the debug stream.
 * Queues the complete message or nothing.
 * @param s
 * @return
 */
int usbctrlDebugStreamWriteStr(const char *s) {
    int count;
    int written;
    int head, tail;
    irqflags_t irq;
    int i;

    irq = cpu_irq_save();
    head = debugStream.head;
    tail = debugStream.tail;
    if (head < tail)
        count = tail - head - 1;
    else
        count = sizeof(debugStream.buffer) - head + tail - 1;
    written = strlen(s);
    if (written > count) {
        written = 0;
        debugStream.lost++;
    }
    count = written;
    while (count) {
        i = sizeof(debugStream.buffer) - head;
        if (i > count)
            i = count;
        memcpy(&debugStream.buffer[head], s, i);
        count -= i;
        s += i;
        head += i;
        if (head >= sizeof(debugStream.buffer))
            head = 0;
    }
    debugStream.head = head;
    cpu_irq_restore(irq);

    return written;
}

#ifndef usbctrlDebugStreamPrintf

/**
 * Write a formatted string to the debug stream.
 * Queues the complete message or nothing.
 * @param format
 * @return
 */
int usbctrlDebugStreamPrintf(const char *format, ...) {
    va_list a;
    char str[80];
    int length;

    if (format) {
        va_start(a, format);
        length = vsnprintf(str, sizeof(str), format, a);
        va_end(a);
        if (length >= sizeof(str))
            length = 0;
        if (length)
            length = usbctrlDebugStreamWriteStr(str);
    } else
        length = 0;

    return length;
}

#endif /* usbctrlDebugStreamPrintf */

/**
 * Called by ISR
 */
static void handleSetupEnd(void) {
    unsigned int count;
    int i;

    if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_TYPE_MASK) ==
        USB_REQ_TYPE_VENDOR) {
        switch (udd_g_ctrlreq.req.bRequest) {
        case HF_USBCTRL_NAME:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_OUT) {
                memset(usbctrl.v.name, 0, sizeof(usbctrl.v.name));
                count = udd_g_ctrlreq.req.wLength;
                if (count > sizeof(usbctrl.v.name))
                    count = sizeof(usbctrl.v.name);
                memcpy(usbctrl.v.name, usbctrl.payload.name, count);
                usbctrl.flags |= FLAGS_SETTING_NAME;
            }
            break;
        case HF_USBCTRL_FAN_PARMS:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_OUT) {
                memcpy(&usbctrl.v.fanSettings, &usbctrl.payload.fanSettings,
                       sizeof(usbctrl.v.fanSettings));
                usbctrl.flags |= FLAGS_SETTING_FAN_SETTINGS;
            }
            break;
        case HF_USBCTRL_ASIC_PARMS:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_OUT) {
                if (usbctrl.payload.b[0] == 0 &&
                    usbctrl.payload.b[1] == 0) {
                    memset(&usbctrl.v.opSettings, 0,
                           sizeof(usbctrl.v.opSettings));
                    usbctrl.v.opSettings.revision = 1;
                    usbctrl.v.opSettings.ref_frequency = usbctrl.payload.b[2];
                    usbctrl.v.opSettings.magic = U_MAGIC;
                    for (i = 0; i < 4; i++) {
                        usbctrl.v.opSettings.die[i].frequency =
                            usbctrl.payload.b[i * 4 + 4] |
                            (uint16_t) usbctrl.payload.b[i * 4 + 5] << 8;
                        usbctrl.v.opSettings.die[i].voltage =
                            usbctrl.payload.b[i * 4 + 6] |
                            (uint16_t) usbctrl.payload.b[i * 4 + 7] << 8;
                    }
                    usbctrl.module = udd_g_ctrlreq.req.wIndex & 0xff;
                    usbctrl.flags |= FLAGS_SETTING_OP_SETTINGS;
                }
            }
            break;
        case HF_USBCTRL_MODE:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_OUT) {
                usbctrl.v.mode.mode = udd_g_ctrlreq.req.wValue;
                usbctrl.v.mode.baud =
                    usbctrl.payload.b[0] |
                    ((uint16_t) usbctrl.payload.b[1] << 8) |
                    ((uint32_t) usbctrl.payload.b[2] << 16) |
                    ((uint32_t) usbctrl.payload.b[3] << 24);
                usbctrl.flags |= FLAGS_MODE_SET;
            }
            break;
        }
    }
}

/**
 * Called by ISR
 */
int usbctrlSetupPacket(void) {
    int handled;
    unsigned int wLength;
    unsigned int count;
    unsigned int head;
    unsigned int tail;
    uint8_t module;
    uint16_t v;
    uint32_t g;
    int i;

    handled = 0;

    if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_TYPE_MASK) ==
        USB_REQ_TYPE_VENDOR) {
        switch (udd_g_ctrlreq.req.bRequest) {
        case HF_USBCTRL_REBOOT:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_OUT) {
                usbctrl.rebootModule = udd_g_ctrlreq.req.wIndex;
                usbctrl.rebootMode = udd_g_ctrlreq.req.wValue ? loaderRB :
                                                                appRB;
                usbctrl.reboot = 1;
                usbctrl.flags |= FLAGS_REBOOTING;
                handled = 1;
            }
            break;
        case HF_USBCTRL_VERSION:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_IN) {
                g = fw_version(udd_g_ctrlreq.req.wIndex);
                usbctrl.payload.b[0] = (g >> 0) & 0xff;
                usbctrl.payload.b[1] = (g >> 8) & 0xff;
                usbctrl.payload.b[2] = (g >> 16) & 0xff;
                usbctrl.payload.b[3] = (g >> 24) & 0xff;
                usbctrl.payload.b[4] = fw_crc(udd_g_ctrlreq.req.wIndex, &g) ?
                                       0x01 : 0x00;
                usbctrl.payload.b[5] = (g >> 0) & 0xff;
                usbctrl.payload.b[6] = (g >> 8) & 0xff;
                usbctrl.payload.b[7] = (g >> 16) & 0xff;
                usbctrl.payload.b[8] = (g >> 24) & 0xff;
                usbctrl.payload.b[9] = module_type(udd_g_ctrlreq.req.wIndex);
                udd_g_ctrlreq.payload = usbctrl.payload.b;
                udd_g_ctrlreq.payload_size = 10;
                if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                    udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                usbctrl.status = 0;
                handled = 1;
            }
            break;
        case HF_USBCTRL_CONFIG:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_IN) {
                usbctrl.payload.b[0] = (ucinfo.chain_configuration ==
                                        CC_UNCONFIGURED) ?
                                        0xff : (ucinfo.master ? 1 : 0);
                usbctrl.payload.b[1] = ucinfo.num_slaves;
                usbctrl.payload.b[2] = ucinfo.chain_configuration;
                usbctrl.payload.b[3] = ucinfo.board_initialized ? 1 : 0;
                udd_g_ctrlreq.payload = usbctrl.payload.b;
                udd_g_ctrlreq.payload_size = 4;
                if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                    udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                usbctrl.status = 0;
                handled = 1;
            }
            break;
        case HF_USBCTRL_STATUS:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_IN) {
                usbctrl.payload.b[0] = (usbctrl.flags >> 0) & 0xff;
                usbctrl.payload.b[1] = (usbctrl.flags >> 8) & 0xff;
                usbctrl.payload.b[2] = (usbctrl.flags >> 16) & 0xff;
                usbctrl.payload.b[3] = (usbctrl.flags >> 24) & 0xff;
                usbctrl.payload.b[4] = ucinfo.fault_code;
                usbctrl.payload.b[5] = ucinfo.fault_extra;
                usbctrl.payload.b[6] = usbctrl.status;
                usbctrl.payload.b[7] = 0;
                udd_g_ctrlreq.payload = usbctrl.payload.b;
                udd_g_ctrlreq.payload_size = 8;
                if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                    udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                handled = 1;
            }
            break;
        case HF_USBCTRL_SERIAL:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_IN) {
                memset(&usbctrl.payload.serial, 0, sizeof(serial_number_t));
                module_serial(udd_g_ctrlreq.req.wIndex,
                              &usbctrl.payload.serial);
                udd_g_ctrlreq.payload = usbctrl.payload.b;
                udd_g_ctrlreq.payload_size = sizeof(serial_number_t);
                if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                    udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                usbctrl.status = 0;
                handled = 1;
            }
            break;
        case HF_USBCTRL_NAME:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_OUT) {
                if (usbctrl.flags == 0) {
                    udd_g_ctrlreq.payload = usbctrl.payload.b;
                    udd_g_ctrlreq.payload_size = sizeof(usbctrl.payload.b);
                    if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                        udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                    udd_g_ctrlreq.callback = handleSetupEnd;
                    handled = 1;
                }
            } else {
                memcpy(usbctrl.payload.name, hf_nvram_name(), HF_NAME_SIZE);
                udd_g_ctrlreq.payload = usbctrl.payload.b;
                udd_g_ctrlreq.payload_size = HF_NAME_SIZE;
                if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                    udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                usbctrl.status = 0;
                handled = 1;
            }
            break;
        case HF_USBCTRL_FAN:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_OUT) {
                fan_set(udd_g_ctrlreq.req.wIndex & 0xff,
                        udd_g_ctrlreq.req.wIndex >> 8,
                        udd_g_ctrlreq.req.wValue);
                handled = 1;
            } else {
                usbctrl.payload.b[0] = 0;
                usbctrl.payload.b[1] = 0;
                for (i = 0; i < 4; i++) {
                    v = gwq_get_tach((udd_g_ctrlreq.req.wIndex & 0xff) * 4 +
                                     i);
                    usbctrl.payload.b[i * 2 + 2] = v & 0xff;
                    usbctrl.payload.b[i * 2 + 3] = v >> 8;
                }
                v = fan_get(udd_g_ctrlreq.req.wIndex & 0xff,
                            udd_g_ctrlreq.req.wIndex >> 8);
                usbctrl.payload.b[10] = v & 0xff;
                usbctrl.payload.b[12] = v >> 8;
                udd_g_ctrlreq.payload = usbctrl.payload.b;
                udd_g_ctrlreq.payload_size = 12;
                if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                    udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                usbctrl.status = 0;
                handled = 1;
            }
            break;
        case HF_USBCTRL_POWER:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_OUT) {
                /* reserve most possible codes for possible future setting
                   of power related variables and finer grained controls than
                   all on/off */
                if (udd_g_ctrlreq.req.wLength == 0 &&
                    udd_g_ctrlreq.req.wIndex == 0) {
                    switch (udd_g_ctrlreq.req.wValue) {
                    case 0x0000:
                        usb_powerdown_request();
                        usbctrl.flags |= FLAGS_POWERING_DOWN;
                        handled = 1;
                        break;
                    case 0x0001:
                        usb_powerup_request();
                        usbctrl.flags |= FLAGS_POWERING_UP;
                        handled = 1;
                        break;
                    default:
                        break;
                    }
                }
            } else {
                module = udd_g_ctrlreq.req.wIndex & 0xff;
                if (module < MAX_SLAVES + 1) {
                    usbctrl.payload.b[0] = 0;
                    usbctrl.payload.b[1] = 0;
                    for (i = 0; i < 4; i++) {
                        v = moduleStatus[module].inputMillivolts[i];
                        usbctrl.payload.b[i * 6 + 2] = v & 0xff;
                        usbctrl.payload.b[i * 6 + 3] = v >> 8;
                        v = moduleStatus[module].outputMillivolts[i];
                        usbctrl.payload.b[i * 6 + 4] = v & 0xff;
                        usbctrl.payload.b[i * 6 + 5] = v >> 8;
                        v = gwq_get_board_temperature(module * 4 + i);
                        usbctrl.payload.b[i * 6 + 6] = v & 0xff;
                        usbctrl.payload.b[i * 6 + 7] = v >> 8;
                    }
                    udd_g_ctrlreq.payload = usbctrl.payload.b;
                    udd_g_ctrlreq.payload_size = 26;
                    if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                        udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                    usbctrl.status = 0;
                    handled = 1;
                }
            }
            break;
        case HF_USBCTRL_FAN_PARMS:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_OUT) {
                if (usbctrl.flags == 0 &&
                    udd_g_ctrlreq.req.wLength >= sizeof(fan_settings_t) &&
                    udd_g_ctrlreq.req.wIndex == 0 &&
                    (udd_g_ctrlreq.req.wValue & 0xff00) == 0xa500) {
                    udd_g_ctrlreq.payload = usbctrl.payload.b;
                    udd_g_ctrlreq.payload_size = sizeof(usbctrl.payload.b);
                    if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                        udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                    udd_g_ctrlreq.callback = handleSetupEnd;
                    handled = 1;
                }
            } else {
                if (udd_g_ctrlreq.req.wIndex == 0) {
                    memcpy(&usbctrl.payload.fanSettings,
                           hf_nvram_get_fan_settings(),
                           sizeof(usbctrl.payload.fanSettings));
                    udd_g_ctrlreq.payload = usbctrl.payload.b;
                    udd_g_ctrlreq.payload_size =
                        sizeof(usbctrl.payload.fanSettings);
                    if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                        udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                    usbctrl.status = 0;
                    handled = 1;
                }
            }
            break;
        case HF_USBCTRL_ASIC_PARMS:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_OUT) {
                if (usbctrl.flags == 0 &&
                    udd_g_ctrlreq.req.wLength >= 20 &&
                    udd_g_ctrlreq.req.wIndex < ucinfo.num_slaves + 1 &&
                    (udd_g_ctrlreq.req.wValue & 0xff00) == 0xa500 &&
                    (udd_g_ctrlreq.req.wValue & 0x00ff) == 0x0000) {
                    udd_g_ctrlreq.payload = usbctrl.payload.b;
                    udd_g_ctrlreq.payload_size = sizeof(usbctrl.payload.b);
                    if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                        udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                    udd_g_ctrlreq.callback = handleSetupEnd;
                    handled = 1;
                }
            } else {
                module = udd_g_ctrlreq.req.wIndex & 0xff;
                if (module < MAX_SLAVES + 1) {
                    usbctrl.payload.b[0] = 0;
                    usbctrl.payload.b[1] = 0;
                    usbctrl.payload.b[2] = all_ref_clocks[module];
                    usbctrl.payload.b[3] = 0;
                    for (i = 0; i < 4; i++) {
                        v = all_die_settings[module * 4 + i].frequency;
                        usbctrl.payload.b[i * 4 + 4] = v & 0xff;
                        usbctrl.payload.b[i * 4 + 5] = v >> 8;
                        v = all_die_settings[module * 4 + i].voltage;
                        usbctrl.payload.b[i * 4 + 6] = v & 0xff;
                        usbctrl.payload.b[i * 4 + 7] = v >> 8;
                    }
                    udd_g_ctrlreq.payload = usbctrl.payload.b;
                    udd_g_ctrlreq.payload_size = 20;
                    if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                        udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                    usbctrl.status = 0;
                    handled = 1;
                }
            }
            break;
        case HF_USBCTRL_VOLTAGE:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_OUT) {
                module_voltage_set(udd_g_ctrlreq.req.wIndex & 0xff,
                                   udd_g_ctrlreq.req.wIndex >> 8,
                                   udd_g_ctrlreq.req.wValue);
                handled = 1;
            }
            break;
        case HF_USBCTRL_ASIC_CTRL:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_OUT) {
                module = udd_g_ctrlreq.req.wIndex & 0xff;
                if (usbctrl.flags == 0 &&
                    (module < MAX_SLAVES + 1 ||
                     module == 0xff)) {
                    usbctrl.module = module;
                    usbctrl.v.asicFlags = udd_g_ctrlreq.req.wValue;
                    usbctrl.flags |= FLAGS_ASIC_CTRL;
                    handled = 1;
                }
            }
            break;
        case HF_USBCTRL_MODE:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_OUT) {
                if (usbctrl.flags == 0 &&
                    udd_g_ctrlreq.req.wLength >= 4 &&
                    udd_g_ctrlreq.req.wIndex == 0 &&
                    udd_g_ctrlreq.req.wValue >= 0 &&
                    udd_g_ctrlreq.req.wValue <= 1) {
                    udd_g_ctrlreq.payload = usbctrl.payload.b;
                    udd_g_ctrlreq.payload_size = sizeof(usbctrl.payload.b);
                    if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                        udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                    udd_g_ctrlreq.callback = handleSetupEnd;
                    handled = 1;
                }
            }
            break;
        /*
         * Core Overview
         */
        case HF_USBCTRL_CORE_OVERVIEW:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_IN) {
                usbctrl.payload.b[0] = ucinfo.die_count;
                usbctrl.payload.b[1] = ucinfo.core_count;
                usbctrl.payload.b[2] = (ucinfo.total_cores >> 0) & 0xff;
                usbctrl.payload.b[3] = (ucinfo.total_cores >> 8) & 0xff;
                usbctrl.payload.b[4] = (ucinfo.total_good_cores >> 0) & 0xff;
                usbctrl.payload.b[5] = (ucinfo.total_good_cores >> 8) & 0xff;
                usbctrl.payload.b[6] = (ucinfo.shed_supported) ? 0x01 : 0x00;
                usbctrl.payload.b[7] = (ucinfo.groups >> 0) & 0xff;
                usbctrl.payload.b[8] = (ucinfo.groups >> 8) & 0xff;
                usbctrl.payload.b[9] = ucinfo.cores_per_group;
                usbctrl.payload.b[10] = (ucinfo.cores_per_group_cycle >> 0) & 0xff;
                usbctrl.payload.b[11] = (ucinfo.cores_per_group_cycle >> 8) & 0xff;
                usbctrl.payload.b[12] = ucinfo.groups_per_group_cycle;
                usbctrl.payload.b[13] = ucinfo.group_core_offset;
                usbctrl.payload.b[14] = (ucinfo.inflight >> 0) & 0xff;
                usbctrl.payload.b[15] = (ucinfo.inflight >> 8) & 0xff;
                usbctrl.payload.b[16] = (ucinfo.active_jobs >> 0) & 0xff;
                usbctrl.payload.b[17] = (ucinfo.active_jobs >> 8) & 0xff;
                usbctrl.payload.b[14] = (ucinfo.inflight >> 0) & 0xff;
                usbctrl.payload.b[15] = (ucinfo.inflight >> 8) & 0xff;
                usbctrl.payload.b[16] = (ucinfo.active_jobs >> 0) & 0xff;
                usbctrl.payload.b[17] = (ucinfo.active_jobs >> 8) & 0xff;
                udd_g_ctrlreq.payload = usbctrl.payload.b;
                udd_g_ctrlreq.payload_size = 18;
                if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                    udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                handled = 1;
            }
            break;
        /*
         * Core Enable
         */
        case HF_USBCTRL_CORE_ENABLE:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_OUT) {
                core_map_set_enabled((udd_g_ctrlreq.req.wIndex),
                                  (udd_g_ctrlreq.req.wValue));
                handled = 1;
            } else {
                usbctrl.payload.b[0] = 0x00;
                udd_g_ctrlreq.payload = usbctrl.payload.b;
                udd_g_ctrlreq.payload_size = 1;
                if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                    udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                handled = 1;
            }
            break;
        /*
         * Core Disable
         */
        case HF_USBCTRL_CORE_DISABLE:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_OUT) {
                core_map_set_disabled((udd_g_ctrlreq.req.wIndex),
                                 (udd_g_ctrlreq.req.wValue));
                handled = 1;
            } else {
                usbctrl.payload.b[0] = 0x00;
                udd_g_ctrlreq.payload = usbctrl.payload.b;
                udd_g_ctrlreq.payload_size = 1;
                if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                    udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                handled = 1;
            }
            break;
        /*
         * Core Clear
         */
        case HF_USBCTRL_CORE_CLEAR:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_OUT) {
                core_map_reset((udd_g_ctrlreq.req.wValue));
                handled = 1;
            } else {
                usbctrl.payload.b[0] = 0x00;
                udd_g_ctrlreq.payload = usbctrl.payload.b;
                udd_g_ctrlreq.payload_size = 1;
                if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                    udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                handled = 1;
            }
            break;
        /*
         * Core Status
         */
        case HF_USBCTRL_CORE_STATUS:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_IN) {
                int core_index = udd_g_ctrlreq.req.wValue;
                usbctrl.payload.b[0] = CORE_GOOD(core_index) ? 0x01 : 0x00;
                usbctrl.payload.b[1] = CORE_GOOD(core_index) ? 0x01 : 0x00;
                udd_g_ctrlreq.payload = usbctrl.payload.b;
                udd_g_ctrlreq.payload_size = 2;
                if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                    udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                handled = 1;
            }
            break;
        /*
         * Die Status
         */
        case HF_USBCTRL_CORE_DIE_STATUS:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_IN) {
                int die_index = udd_g_ctrlreq.req.wValue;
                int doffset = die_index * DBYTES_PER_DIE;
                for(i = 0; i < DBYTES_PER_DIE; i++) {
                    usbctrl.payload.b[i*2] = (core_good[doffset + i] >> 0) & 0xff;
                    usbctrl.payload.b[i*2 + 1] = (core_good[doffset + i] >> 8) & 0xff;
                }
                for(i = 0; i < DBYTES_PER_DIE; i++) {
                    usbctrl.payload.b[12 + i*2] = (core_good_persist[doffset + i] >> 0) & 0xff;
                    usbctrl.payload.b[12 + i*2 + 1] = (core_good_persist[doffset + i] >> 8) & 0xff;
                }
                for(i = 0; i < DBYTES_PER_DIE; i++) {
                    usbctrl.payload.b[24 + i*2] = (core_good[doffset + i] >> 0) & 0xff;
                    usbctrl.payload.b[24 + i*2 + 1] = (core_good[doffset + i] >> 8) & 0xff;
                }
                for(i = 0; i < DBYTES_PER_DIE; i++) {
                    usbctrl.payload.b[36 + i*2] = (core_good[doffset + i] >> 0) & 0xff;
                    usbctrl.payload.b[36 + i*2 + 1] = (core_good[doffset + i] >> 8) & 0xff;
                }
                udd_g_ctrlreq.payload = usbctrl.payload.b;
                udd_g_ctrlreq.payload_size = 48;
                if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                    udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                handled = 1;
            }
            break;
        /*
         * ASIC Status
         */
        case HF_USBCTRL_CORE_ASIC_STATUS:
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_IN) {
                int asic_index = udd_g_ctrlreq.req.wValue;
                int aoffset = asic_index * DBYTES_PER_ASIC;
                for(i = 0; i < DBYTES_PER_ASIC; i++) {
                    usbctrl.payload.b[i*2] = (core_good[aoffset + i] >> 0) & 0xff;
                    usbctrl.payload.b[i*2 + 1] = (core_good[aoffset + i] >> 8) & 0xff;
                }
                udd_g_ctrlreq.payload = usbctrl.payload.b;
                udd_g_ctrlreq.payload_size = 48;
                if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                    udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                handled = 1;
            }
            break;
        /*
         * DEBUG CONTROL
         */
        case HF_USBCTRL_DEBUG_BUFFER:  /* fixed buffer output */
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_IN) {
                memcpy(usbctrl.payload.b, usbctrlDebugBuffer,
                       sizeof(usbctrl.payload.b));
                udd_g_ctrlreq.payload = usbctrl.payload.b;
                udd_g_ctrlreq.payload_size = sizeof(usbctrl.payload.b);
                if (udd_g_ctrlreq.payload_size > udd_g_ctrlreq.req.wLength)
                    udd_g_ctrlreq.payload_size = udd_g_ctrlreq.req.wLength;
                handled = 1;
            }
            break;
        case HF_USBCTRL_DEBUG_STREAM:  /* stream output */
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_IN) {
                udd_g_ctrlreq.payload = usbctrl.payload.b;
                udd_g_ctrlreq.payload_size = 0;
                tail = debugStream.tail;
                wLength = udd_g_ctrlreq.req.wLength;
                if (wLength > sizeof(usbctrl.payload.b))
                    wLength = sizeof(usbctrl.payload.b);
                while (tail != debugStream.head &&
                       udd_g_ctrlreq.payload_size < wLength) {
                    if (debugStream.head >= tail)
                        count = debugStream.head - tail;
                    else
                        count = sizeof(debugStream.buffer) - tail;
                    if (count > wLength - udd_g_ctrlreq.payload_size)
                        count = wLength - udd_g_ctrlreq.payload_size;
                    if (count) {
                        memcpy(&usbctrl.payload.b[udd_g_ctrlreq.payload_size],
                               &debugStream.buffer[tail], count);
                        tail += count;
                        if (tail >= sizeof(debugStream.buffer))
                            tail = 0;
                        udd_g_ctrlreq.payload_size += count;
                    }
                }
                debugStream.tail = tail;
                if (udd_g_ctrlreq.payload_size == 0) {
                    /* TODO
                     * I should be able to send a zero byte response to the
                     * control request (although I'm not certain, the USB spec
                     * might forbid it - I haven't checked).  Anyway, it isn't
                     * working and it isn't worth taking the time to determine
                     * why so I'm putting in this workaround.
                     */
                    usbctrl.payload.b[0] = '\0';
                    udd_g_ctrlreq.payload_size = 1;
                }
                handled = 1;
            }
            break;
        case HF_USBCTRL_DEBUG_CLI:  /* interactive comm */
            if ((udd_g_ctrlreq.req.bmRequestType & USB_REQ_DIR_MASK) ==
                USB_REQ_DIR_IN) {
                udd_g_ctrlreq.payload = usbctrl.payload.b;
                udd_g_ctrlreq.payload_size = 0;
                tail = debugMonitor.tx.tail;
                wLength = udd_g_ctrlreq.req.wLength;
                if (wLength > sizeof(usbctrl.payload.b))
                    wLength = sizeof(usbctrl.payload.b);
                while (tail != debugMonitor.tx.head &&
                       udd_g_ctrlreq.payload_size < wLength) {
                    if (debugMonitor.tx.head >= tail)
                        count = debugMonitor.tx.head - tail;
                    else
                        count = sizeof(debugMonitor.tx.buffer) - tail;
                    if (count > wLength - udd_g_ctrlreq.payload_size)
                        count = wLength - udd_g_ctrlreq.payload_size;
                    if (count) {
                        memcpy(&usbctrl.payload.b[udd_g_ctrlreq.payload_size],
                               &debugMonitor.tx.buffer[tail], count);
                        tail += count;
                        if (tail >= sizeof(debugMonitor.tx.buffer))
                            tail = 0;
                        udd_g_ctrlreq.payload_size += count;
                    }
                }
                debugMonitor.tx.tail = tail;
                if (udd_g_ctrlreq.payload_size == 0) {
                    /* TODO
                     * I should be able to send a zero byte response to the
                     * control request (although I'm not certain, the USB spec
                     * might forbid it - I haven't checked).  Anyway, it isn't
                     * working and it isn't worth taking the time to determine
                     * why so I'm putting in this workaround.
                     */
                    usbctrl.payload.b[0] = '\0';
                    udd_g_ctrlreq.payload_size = 1;
                }
                handled = 1;
            } else {
                if (udd_g_ctrlreq.req.wLength) {
                    /* TODO
                     * Should setup buffer to receive data into, to allow
                     * multiple bytes in one transaction. But that will require
                     * figuring out how ASF wants to notify us when the buffer
                     * has been filled. For now one byte per setup is enough,
                     * it's just for keyboard input.
                     */
                } else {
                    head = debugMonitor.rx.head;
                    if (++head >= sizeof(debugMonitor.rx.buffer))
                        head = 0;
                    if (head != debugMonitor.rx.tail) {
                        debugMonitor.rx.buffer[debugMonitor.rx.head] =
                            udd_g_ctrlreq.req.wValue;
                        debugMonitor.rx.head = head;
                    }
                    handled = 1;
                }
            }
            break;
        }
    }
    return handled;
}

/**
 * Periodic USB Control Channel task.
 */
void usbctrlTask(void) {
    static uint16_t lastTick;
    uint16_t tick;
    uint16_t elapsed;
    uint8_t txBuffer[2];
    irqflags_t irq;
    uint8_t b;

    tick = msec_ticker;
    elapsed = tick - lastTick;
    lastTick = tick;

    if (usbctrl.timer <= elapsed)
        usbctrl.timer = 0;
    else
        usbctrl.timer -= elapsed;

    switch (usbctrl.state) {
    case idleS:
        if (usbctrl.reboot) {
            /* time for usbctrl ack to host */
            usbctrl.timer = 10;
            usbctrl.reboot = 0;
            usbctrl.state = rebootS;
        } else if (usbctrl.flags & FLAGS_SETTING_NAME) {
            hf_nvram_name_set(usbctrl.v.name);
            usbctrl.status = 0;
            usbctrl.flags &= ~FLAGS_SETTING_NAME;
        } else if (usbctrl.flags & FLAGS_SETTING_FAN_SETTINGS) {
            usbctrl.status = 0;
            hf_nvram_fan_settings_set(&usbctrl.v.fanSettings);
            usbctrl.flags &= ~FLAGS_SETTING_FAN_SETTINGS;
        } else if (usbctrl.flags & FLAGS_SETTING_OP_SETTINGS) {
            usbctrl.status = hf_nvram_write_die_settings(
                                 usbctrl.module, &usbctrl.v.opSettings) ?
                             0 : 1;
            usbctrl.flags &= ~FLAGS_SETTING_OP_SETTINGS;
        } else if (usbctrl.flags & FLAGS_ASIC_CTRL) {
            usbctrl.status = 0;
            b = usbctrl.v.asicFlags & 0x07;
            if (!(usbctrl.v.asicFlags & HF_USBCTRL_ASIC_CTRL_VALUE_RESET))
                b |= F_ASIC_UNRESET;
            if (usbctrl.v.asicFlags & HF_USBCTRL_ASIC_CTRL_VALUE_PLL_BYPASS)
                b |= F_ASIC_PLL_BYPASS;
            if (usbctrl.module > 0) {
                txBuffer[0] = TWICMD_FPGA_ASIC_CTL;
                txBuffer[1] = b | F_FORCE_BAUD;
                usbctrl.status = twi_sync_rw(TWI_BUS_UC,
                                             (usbctrl.module == 0xff) ?
                                             TWI_BROADCAST :
                                             TWI_SLAVE_STARTADDR +
                                             usbctrl.module - 1,
                                             txBuffer, 2, NULL, 0) ? 0 : 1;
            }
            if (usbctrl.module == 0 || usbctrl.module == 0xff)
                fpga_reg_write(FA_ASIC_CONTROL, b);
            usbctrl.flags &= ~FLAGS_ASIC_CTRL;
        } else if (usbctrl.flags & FLAGS_MODE_SET) {
            usbctrl.status = 0;
            da2sEnable(usbctrl.v.mode.mode, usbctrl.v.mode.baud);
            usbctrl.flags &= ~FLAGS_MODE_SET;
        }
        if ((usbctrl.flags & FLAGS_POWERING_UP) && system_on()) {
            usbctrl.status = 0;
            usbctrl.flags &= ~FLAGS_POWERING_UP;
        }
        if ((usbctrl.flags & FLAGS_POWERING_DOWN) && system_off()) {
            usbctrl.status = 0;
            usbctrl.flags &= ~FLAGS_POWERING_DOWN;
        }
        break;
    case rebootS:
        if (usbctrl.timer == 0) {
            if (ucinfo.master && !ucinfo.no_slaves) {
                txBuffer[0] = TWICMD_REBOOT;
                txBuffer[1] = (usbctrl.rebootMode == loaderRB) ? 1 : 0;
                twi_sync_rw(TWI_BUS_UC,
                            (usbctrl.rebootModule == 0 ||
                             usbctrl.rebootModule == 0xff) ?
                            TWI_BROADCAST :
                            (TWI_SLAVE_STARTADDR + usbctrl.rebootModule - 1),
                            txBuffer, 2, NULL, 0);
            }
            if (usbctrl.rebootModule == 0 || usbctrl.rebootModule == 0xff) {
                if (usbctrl.rebootMode == loaderRB) {
                    /*
                     * Tell Atmel DFU loader not to start app ever again
                     * (harmless with custom loader).
                     */
                    flashc_erase_gp_fuse_bit(31, true);
                    flashc_write_gp_fuse_bit(31, true);
                    /*
                     * Tell custom bootloader not to start app on this boot
                     * (harmless with Atmel DFU loader).
                     */
                    AVR32_PM.gplp[1] = 0x73746179;
                }
                /* no return */
                self_reset();
            }
            usbctrl.state = idleS;
            irq = cpu_irq_save();
            if (!usbctrl.reboot) {
                usbctrl.status = 0;
                usbctrl.flags &= ~FLAGS_REBOOTING;
            }
            cpu_irq_restore(irq);
        }
        break;
    }
}
