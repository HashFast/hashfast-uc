//
// Accumulate debug printf's and dump them to the host periodically
//

#include "main.h"
#include <stdio.h>
#include "cli.h"
#include "da2s.h"

#ifndef uprintf

static uint16_t debug_bitmap = 0
#ifdef WORK_RESTART_DEBUG
    | UD_WORK_RESTART
#endif
    | UD_STARTUP|UD_CHAR
    ;

void uprintf(uint16_t dmask, char *format, ...)
    {
    va_list aptr;
    char cbuf[128];
    uint16_t n;

    if (!(dmask & debug_bitmap))
        return;

    if (format)
        {
        va_start(aptr, format);
        n = vsnprintf(cbuf, sizeof(cbuf), format, aptr);
        va_end(aptr);
        cbuf[n] = '\0';
        cliWriteString(cbuf);
        }
    }
#endif


//
// NOTE:
//    NEVER call notify_host() except under diagnostic or exception circumstances, because
//    the function below uses a polled output USB function that could block.
//

void notify_host(char *format, ...)
    {
    va_list aptr;
    int n;

    struct {
        struct hf_header h;
        uint32_t data;
        char buf[MAX_NOTIFY_MESSAGE_SIZE+4];
        } __attribute__((packed)) notify;


    if (main_b_cdc_enable == false)
        return;

    if (format && !ucinfo.usb_init_header && !da2sEnabled)
        {
        memset(&notify.h, 0, sizeof(notify.h));
        notify.h.preamble = HF_PREAMBLE;
        notify.h.operation_code = OP_USB_NOTICE;

        notify.data = 0;                // Forget what this was ever for

        va_start(aptr, format);
        n = vsnprintf(notify.buf, sizeof(notify.buf)-4, format, aptr);
        va_end(aptr);
        notify.buf[n++] = '\0';
        while (n & 0x3)
            notify.buf[++n] = '\0';

        notify.h.data_length = (4+n)/4;
        notify.h.crc8 = hf_crc8((uint8_t *)&notify.h);

        udi_cdc_write_buf(&notify, sizeof(struct hf_header)+4+n);
        }
    }
