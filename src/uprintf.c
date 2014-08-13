/** @file uprintf.c
 * @brief Accumulate debug printf's and dump them to the host periodically
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
#include <stdio.h>
#include "cli.h"
#include "da2s.h"

#ifndef uprintf

static uint16_t debug_bitmap = 0
#ifdef WORK_RESTART_DEBUG
    | UD_WORK_RESTART
#endif
    | UD_STARTUP | UD_CHAR;

void uprintf(uint16_t dmask, char *format, ...) {
    va_list aptr;
    char cbuf[128];
    uint16_t n;

    if (!(dmask & debug_bitmap)) {
        return;
    }

    if (format) {
        va_start(aptr, format);
        n = vsnprintf(cbuf, sizeof(cbuf), format, aptr);
        va_end(aptr);
        cbuf[n] = '\0';
        cliWriteString(cbuf);
    }
}

#endif /* uprintf */

#ifndef notify_host

/**
 * Send a notification to the host via OP_USB_NOTIFY
 * NEVER call notify_host() except under diagnostic or exception circumstances,
 * because the function below uses a polled output USB function that could block.
 * @param format
 */
void notify_host(char *format, ...) {
    va_list aptr;
    int n;

    struct {
        struct hf_header h;
        uint32_t data;
        char buf[MAX_NOTIFY_MESSAGE_SIZE + 4];
    }__attribute__((packed)) notify;

    if (main_b_cdc_enable == false)
        return;

    if (format && !ucinfo.usb_init_header && !da2sEnabled) {
        memset(&notify.h, 0, sizeof(notify.h));
        notify.h.preamble = HF_PREAMBLE;
        notify.h.operation_code = OP_USB_NOTICE;

        notify.data = 0;                //TODO Forget what this was ever for

        va_start(aptr, format);
        n = vsnprintf(notify.buf, sizeof(notify.buf) - 4, format, aptr);
        va_end(aptr);
        notify.buf[n++] = '\0';
        while (n & 0x3) {
            notify.buf[++n] = '\0';
        }
        notify.h.data_length = (4 + n) / 4;
        notify.h.crc8 = hf_crc8((uint8_t *) &notify.h);

        udi_cdc_write_buf(&notify, sizeof(struct hf_header) + 4 + n);
    }
}

#endif /* notify_host */
