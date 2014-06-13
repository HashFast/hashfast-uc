/* da2s.c */

/*
    Copyright (c) 2014 HashFast Technologies LLC
*/

#include <stdint.h>
#include <intc.h>
#include <udc.h>

#include "da2s.h"


#define USART_DA2S                         (*GN_UART)

#define DMA_USART_DA2S_RX_CHANNEL          DMA_UART_RX_CHANNEL
#define DMA_USART_DA2S_TX_CHANNEL          DMA_UART_TX_CHANNEL

#define DMA_USART_DA2S_RX      (AVR32_PDCA.channel[DMA_USART_DA2S_RX_CHANNEL])
#define DMA_USART_DA2S_TX      (AVR32_PDCA.channel[DMA_USART_DA2S_TX_CHANNEL])

/* 230400 is the max baud the ASIC with a 25MHz ref clock can do out of
   reset */
#define BAUD_DEFAULT                 230400

#define RX_BUFFER_BYTES                 256

#define TX_BUFFER_BYTES                 256


char da2sEnabled = 0;


static struct {
    struct {
        uint8_t buffer[RX_BUFFER_BYTES];
        unsigned int tail;
    } rx;
    struct {
        uint8_t buffer[TX_BUFFER_BYTES];
        unsigned int head;
        unsigned int queuedTail;
    } tx;
} da2s;



static __attribute__((__interrupt__)) void rxDMA(void) {

    DMA_USART_DA2S_RX.marr = (uint32_t) &da2s.rx.buffer[0];
    DMA_USART_DA2S_RX.tcrr = sizeof(da2s.rx.buffer);
}

static __attribute__((__interrupt__)) void txDMA(void) {
    uint32_t count;

    if (da2s.tx.queuedTail <= da2s.tx.head)
        count = da2s.tx.head - da2s.tx.queuedTail;
    else
        count = sizeof(da2s.tx.buffer) - da2s.tx.queuedTail;
    if (count) {
        DMA_USART_DA2S_TX.marr = (uint32_t)
                                 &da2s.tx.buffer[da2s.tx.queuedTail];
        DMA_USART_DA2S_TX.tcrr = count;
        da2s.tx.queuedTail += count;
        if (da2s.tx.queuedTail >= sizeof(da2s.tx.buffer))
            da2s.tx.queuedTail = 0;
    } else
        DMA_USART_DA2S_TX.idr = AVR32_PDCA_IDR_RCZ_MASK;
}

static void disableUsartAndDma(void) {
    irqflags_t irq;

    irq = cpu_irq_save();
    USART_DA2S.idr = ~(uint32_t) 0;
    DMA_USART_DA2S_TX.idr = ~(uint32_t) 0;
    DMA_USART_DA2S_RX.idr = ~(uint32_t) 0;
    cpu_irq_restore(irq);
    DMA_USART_DA2S_TX.cr = AVR32_PDCA_CR_ECLR_MASK | AVR32_PDCA_CR_TDIS_MASK;
    DMA_USART_DA2S_RX.cr = AVR32_PDCA_CR_ECLR_MASK | AVR32_PDCA_CR_TDIS_MASK;
    DMA_USART_DA2S_TX.tcrr = 0;
    DMA_USART_DA2S_RX.tcrr = 0;
    DMA_USART_DA2S_TX.tcr = 0;
    DMA_USART_DA2S_RX.tcr = 0;
    USART_DA2S.cr = AVR32_USART_CR_RSTTX | AVR32_USART_CR_RSTRX;
}

void da2sEnable(char enable, uint32_t baud) {
    uint32_t div;

    if (enable) {
        if (!da2sEnabled) {
            disableUsartAndDma();
            if (baud == 0)
                baud = BAUD_DEFAULT;
        }
        if (baud) {
            div = (sysclk_get_pba_hz() + (baud * (16 / 8) / 2)) /
                  (baud * (16 / 8));
            USART_DA2S.brgr = ((div >> 3) << AVR32_USART_BRGR_CD) |
                              ((div & 7) << AVR32_USART_BRGR_FP);
        }
        if (!da2sEnabled) {
            da2s.rx.tail = 0;
            da2s.tx.head = 0;
            da2s.tx.queuedTail = 0;
            da2sEnabled = 1;
            USART_DA2S.mr = (AVR32_USART_MR_OVER_X16 <<
                             AVR32_USART_MR_OVER) |
                            (AVR32_USART_MR_MSBF_LSBF <<
                             AVR32_USART_MR_MSBF) |
                            (AVR32_USART_MR_CHMODE_NORMAL <<
                             AVR32_USART_MR_CHMODE) |
                            (AVR32_USART_MR_NBSTOP_1 <<
                             AVR32_USART_MR_NBSTOP) |
                            (AVR32_USART_MR_PAR_NONE <<
                             AVR32_USART_MR_PAR) |
                            (AVR32_USART_MR_CHRL_8 <<
                             AVR32_USART_MR_CHRL) |
                            (AVR32_USART_MR_USCLKS_MCK <<
                             AVR32_USART_MR_USCLKS) |
                            (AVR32_USART_MR_MODE_NORMAL <<
                             AVR32_USART_MR_MODE);
            INTC_register_interrupt(&rxDMA, AVR32_PDCA_IRQ_2, AVR32_INTC_INT3);
            INTC_register_interrupt(&txDMA, AVR32_PDCA_IRQ_1, AVR32_INTC_INT2);
            DMA_USART_DA2S_TX.mar = (uint32_t) &da2s.tx.buffer[0];
            DMA_USART_DA2S_RX.mar = (uint32_t) &da2s.rx.buffer[0];
            DMA_USART_DA2S_TX.psr = AVR32_PDCA_PID_USART1_TX;
            DMA_USART_DA2S_RX.psr = AVR32_PDCA_PID_USART1_RX;
            DMA_USART_DA2S_TX.mr = AVR32_PDCA_MR_SIZE_BYTE <<
                                   AVR32_PDCA_MR_SIZE_OFFSET;
            DMA_USART_DA2S_RX.mr = AVR32_PDCA_MR_SIZE_BYTE <<
                                   AVR32_PDCA_MR_SIZE_OFFSET;
            DMA_USART_DA2S_TX.cr = AVR32_PDCA_CR_TEN_MASK;
            DMA_USART_DA2S_RX.cr = AVR32_PDCA_CR_TEN_MASK;
            DMA_USART_DA2S_RX.ier = AVR32_PDCA_IER_RCZ_MASK;
            USART_DA2S.cr = AVR32_USART_CR_RXEN_MASK |
                            AVR32_USART_CR_TXEN_MASK;
        }
    } else if (da2sEnabled) {
        disableUsartAndDma();
        da2sEnabled = 0;
        /* if we wanted the capability to switch back to normal mode, we
           would ask the usb_uart module to reinit here.  but at least for
           now we'll require a reboot; there's probably no reason to ever
           switch between modes without a reboot. */
    }
}

void da2sTask(void) {
    int uartCount;
    int count;
    uint32_t r;
    int i;

    if (da2sEnabled && main_b_cdc_enable) {
        r = DMA_USART_DA2S_RX.mar;
        uartCount = (int) ((r - (uint32_t) &da2s.rx.buffer[0]) -
                           (uint32_t) da2s.rx.tail);
        if (uartCount < 0)
            uartCount += sizeof(da2s.rx.buffer);
        if (uartCount) {
            count = udi_cdc_get_free_tx_buffer();
            if (count > uartCount)
                count = uartCount;
            while (count > 0) {
                i = sizeof(da2s.rx.buffer) - da2s.rx.tail;
                if (i > count)
                    i = count;
                i -= udi_cdc_write_buf(&da2s.rx.buffer[da2s.rx.tail], i);
                if (i == 0)
                    break;
                count -= i;
                da2s.rx.tail += i;
                if (da2s.rx.tail >= sizeof(da2s.rx.buffer))
                    da2s.rx.tail = 0;
            }
        }

        count = udi_cdc_get_nb_received_data();
        if (count) {
            r = DMA_USART_DA2S_TX.mar;
            uartCount = (int) ((uint32_t) da2s.tx.head -
                               (r - (uint32_t) &da2s.tx.buffer[0]));
            if (uartCount < 0)
                uartCount += sizeof(da2s.rx.buffer);
            uartCount = sizeof(da2s.rx.buffer) - 1 - uartCount;
            if (count > uartCount)
                count = uartCount;
            if (count) {
                while (count > 0) {
                    i = sizeof(da2s.tx.buffer) - da2s.tx.head;
                    if (i > count)
                        i = count;
                    i -= udi_cdc_read_buf(&da2s.tx.buffer[da2s.tx.head], i);
                    if (i == 0)
                        break;
                    count -= i;
                    da2s.tx.head += i;
                    if (da2s.tx.head >= sizeof(da2s.tx.buffer))
                        da2s.tx.head = 0;
                }
                DMA_USART_DA2S_TX.ier = AVR32_PDCA_IER_RCZ_MASK;
            }
        }
    }
}

