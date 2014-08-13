/** @file usb_uart.c
 * @brief USB / UART interfaces
 *
 * Everything to do with handling USB and UART packet level traffic, including
 *      - The USB device endpoints
 *      - The UART link that talks to the ASIC(s)
 *      - DMA handling and gateway functions between the two
 *      - Internal packet generation and reception
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
#include "cli.h"
#include "hf_util.h"
#include "hf_protocol.h"

static void send_stats1(void);
static void uart_tx_dma_init(uint8_t *, int);

/**
 * UART rx mode
 */
enum rxuart_mode_t {
    RX_UART_DISABLED = 0,       //!< RX_UART_DISABLED
    RX_UART_ENABLED             //!< RX_UART_ENABLED
};

/* Statistic counters */
static struct hf_usb_stats1 stats __attribute__((aligned(2)));

static uint8_t uart_rx_buffer[RX_BUFFERS * RX_BUFFER_SIZE] __attribute__((aligned(4)));
static uint8_t uart_tx_buffer[TX_BUFFERS * TX_BUFFER_SIZE] __attribute__((aligned(4)));

static uint8_t volatile *rx_head = &uart_rx_buffer[0];
static uint8_t volatile *rx_tail = &uart_rx_buffer[0];
static uint8_t rx_buffers_used;
static uint8_t tx_buffers_used;

static uint8_t * const rx_buffer_limit = (uart_rx_buffer + (RX_BUFFERS * RX_BUFFER_SIZE));
static uint8_t * const tx_buffer_limit = (uart_tx_buffer + (TX_BUFFERS * TX_BUFFER_SIZE));
static uint8_t * const tx_buffer_first = (uart_tx_buffer + sizeof(struct uart_sendinfo));

static uint8_t *tx_head = (uart_tx_buffer + sizeof(struct uart_sendinfo));
static uint8_t *tx_tail = (uart_tx_buffer + sizeof(struct uart_sendinfo));

static bool looking_for_preamble = true;
static bool usb_iface_cfg = false;
static uint8_t disable_host = 0;
static uint16_t usb_sync_loss = 0;
static uint16_t uart_crc32_errors = 0;
static uint16_t uart_rx_header_addr_bad = 0;
static uint16_t uart_rx_data_addr_bad = 0;
static uint16_t uart_rx_too_large = 0;
static uint16_t last_stats1_sent;

extern uint16_t group_sequence_head;

static uint8_t dma_tx_in_progress = 0;
static struct hf_header *mh;                    //!< Multiple frame pointer

/**
 * Check incoming USB packets
 */
void CheckUsbIncoming(void) {
    struct ucinfo_t *info = &ucinfo;

    static uint8_t index;
    static uint8_t body_bytes;
    static uint8_t *t;
    static uint8_t dlen;
    static uint8_t crc8;
    static uint8_t header_byte;
    static bool sync_loss;
    struct hf_header *h;
    int16_t ch;
    int16_t rxb;
    uint8_t i;
    bool send_packet = false;
    irqflags_t irq;

    if (disable_host) {
        return;
    }

    if (main_b_cdc_enable == false) {
        /* no USB connection */
        looking_for_preamble = true;
        return;
    } else {
        usb_iface_cfg = true;
    }

    /* flow control back to USB host */
    if (tx_buffers_used >= TX_BUFFERS - 2) {
        return;
    }

    if (looking_for_preamble) {

#ifndef FEATURE_DISABLE_ABORTS
        if (info->resend_op_config == true) {
            if (tx_buffers_used >= TX_BUFFERS - 2) {
                return;
            }
            info->resend_op_config = false;
            info->work_restart_in_progress = false;
            info->restart_phase = 0;
            uprintf(UD_WORK_RESTART, "resent_op_config: %d ms: head %4d tail %4d\n", elapsed_since(ucinfo.work_restart_start_time), group_sequence_head, group_sequence_tail);

            if (info->gwq_enabled) {
                /*
                 * Send twice to guard against transmission errors losing one.
                 * Right now status messages are turned off, so GWQ is at a
                 * standstill. Watchdog will kick in if we lose both.
                 */
                make_config_frame((struct hf_header *) tx_head, HF_BROADCAST_ADDRESS, NULL, 0, 0, 0);
                asic_queue_transmit();
                make_config_frame((struct hf_header *) tx_head, HF_BROADCAST_ADDRESS, NULL, 0, 0, 0);
                asic_queue_transmit();

                if (ucinfo.mixed_reference_clocks) {
                    /*Fix up the die with a different reference clock.
                     */
                    for (i = 0; i < ucinfo.die_count; i++) {
                        if (module_ref_clocks[i] != ucinfo.ref_frequency) {
                            if ((h = (struct hf_header *) asic_get_tx_buffer())) {
                                make_config_frame(h, i, NULL, 0, 0, module_ref_clocks[i]);
                                asic_queue_transmit();
                            }
                            /* else
                             * We're out of luck. Just have to deal with messed
                             * up timing until next work restart which shouldn't
                             * be a problem. There should be plenty of tx
                             * buffers at the end of a work restart.
                             */
                        }
                    }
                }
                gwq_work_restart_complete();
            }
            /* else
             * It was a ums_clock_change() and OP_CONFIG's are the user's
             * responsibility.
             */
            return;
        }

#ifdef FEATURE_COWARDLY_WORK_RESTART
        else if (info->restart_phase) {
            if (tx_buffers_used >= TX_BUFFERS - 2) {
                return;
            }
            if ((gwq_work_restart_process((struct hf_header *) tx_head)) == true) {
                asic_queue_transmit();
                return;
            }
        }
#endif /* FEATURE_COWARDLY_WORK_RESTART */

        /* Don't accept more work until restart completed. */
        if (info->work_restart_requested || info->restart_phase) {
            return;
        }

#endif /* FEATURE_DISABLE_ABORTS */
    }

    /*
     * If we are doing an initialization cycle, IGNORE everything the host
     * sends us.
     */
    if (ucinfo.usb_init_header)
        return;

    /* There is a free tx buffer, so we can try to receive something. */
    while ((rxb = udi_cdc_get_nb_received_data()) > 0) {
        if ((ch = udi_cdc_getc()) < 0) {
            stats.usb_rx_receive_byte_errors++;
            break;
        }
        if (looking_for_preamble) {
            if ((uint8_t) ch == HF_PREAMBLE) {
                //hf_trace(TR_USB_INCOMING_PREAMBLE, tx_buffers_used | (looking_for_preamble << 4));
                stats.usb_rx_preambles++;
                *tx_head = (uint8_t) ch;
                t = tx_head + 1;
                looking_for_preamble = false;
                send_packet = true;
                crc8 = 0xff;
                header_byte = 1;
                if (sync_loss) {
                    sync_loss = false;
                    usb_sync_loss++;
                }
            } else {
                sync_loss = true;
            }
            continue;
        }
        if (header_byte) {
            if (header_byte < 7) {
                crc8 = crc8_table[crc8 ^ (uint8_t) ch];
            }
            *t++ = (uint8_t) ch;
            if (header_byte == 6)
                dlen = (uint8_t) ch;
            if (header_byte == 7) {
                header_byte = 0;
                if (crc8 != ch) {
                    //snprintf(display_buf, sizeof(display_buf), "%02x: %02x%02x%02x%02x %02x%02x%02x%02x", crc8,
                    //    *(t-8), *(t-7), *(t-6), *(t-5), *(t-4), *(t-3), *(t-2), *(t-1));
                    /* Bad header CRC */
                    stats.usb_rx_bad_hcrc++;
                    hf_trace(TR_USB_INCOMING_HEADER_CRC, crc8);
                    looking_for_preamble = true;
                    continue;
                } else if (dlen == 0) {
                    /*
                     * Dispatch SHORT operation
                     * cgminer will periodically send OP_PING's at us while work
                     * is starved due to trouble communicating with the pool
                     * server. That can be used to keep the watchdog at bay.
                     */
                    host_watchdog_clock = HOST_WATCHDOG_RECHARGE;

                    h = (struct hf_header *) tx_head;

#ifndef FEATURE_DISABLE_ABORTS
                    if (h->operation_code == OP_WORK_RESTART)
                        ucinfo.work_restart_requested = true;
#endif /* FEATURE_DISABLE_ABORTS */

                    tx_head += TX_BUFFER_SIZE;
                    if (tx_head >= tx_buffer_limit)
                        tx_head = tx_buffer_first;
                    irq = cpu_irq_save();
                    if (++tx_buffers_used > stats.max_tx_buffers)
                        stats.max_tx_buffers = tx_buffers_used;
                    cpu_irq_restore(irq);
                    looking_for_preamble = true;
                    if (tx_buffers_used >= TX_BUFFERS || !dma_tx_in_progress)
                        break;
                } else {
                    /* prepare for packet body data */
                    index = 8;
                    body_bytes = dlen * 4 + 8;
                }
            } else
                header_byte++;
        } else {
            /* packet body */
            *t++ = (uint8_t) ch;
            if (++index >= body_bytes) {
                /* Got the whole packet. */
                h = (struct hf_header *) tx_head;
#ifndef FEATURE_DISABLE_ABORTS
                if (h->operation_code == OP_WORK_RESTART)
                    ucinfo.work_restart_requested = true;
#endif
                if (h->operation_code == OP_HASH) {

                    host_watchdog_clock = HOST_WATCHDOG_RECHARGE;
                    if (!last_stats1_sent)
                        last_stats1_sent = sec_ticker;

                    if (h->chip_address == 254) {
                        if (ucinfo.ntime_roll_total <= 1) {
                            send_packet = gwq_process_hash_nogroup(h);
                            for (i = 0; i < 4; i++) {
                                /*
                                 * Add the silly spare3 field which is kept out
                                 * of the USB i/f to keep the OP_HASH header
                                 * plus data within 64 bytes.
                                 */
                                *t++ = (uint8_t) 0;
                            }
                        } else {
                            /* GWQ protocol converts it to a group operation */
                            send_packet = gwq_process_hash_group(h);
                            t += 4;
                        }
                    } else {
                        /* UMS protocol */
                        for (i = 0; i < 4; i++) {
                            /*
                             * Add the silly spare3 field which is kept out
                             * of the USB i/f to keep the OP_HASH header
                             * plus data within 64 bytes.
                             */
                            *t++ = (uint8_t) 0;
                        }
                    }
                    /* Adjust length and re-compute header CRC-8. */
                    h->data_length = U32SIZE(struct hf_hash_serial);
                    h->crc8 = hf_crc8((uint8_t *) h);
                }

                hf_trace(TR_USB_INCOMING_DISPATCH_LONG, dlen);

                if (send_packet) {
                    tx_head += TX_BUFFER_SIZE;
                    if (tx_head >= tx_buffer_limit)
                        tx_head = tx_buffer_first;
                    irq = cpu_irq_save();
                    if (++tx_buffers_used > stats.max_tx_buffers)
                        stats.max_tx_buffers = tx_buffers_used;
                    cpu_irq_restore(irq);
                }
                looking_for_preamble = true;
                if (tx_buffers_used >= TX_BUFFERS || (tx_buffers_used > 0 && !dma_tx_in_progress))
                    break;
            }
        }
    }
}

static int got_shutdown;
extern bool power_up_only;

/**
 * Check outgoing UART
 */
void CheckUartOutgoing() {
    struct hf_header *h;
    uint8_t len;
    irqflags_t irq;
    int sts;
    struct ucinfo_t *info = &ucinfo;
    struct uart_sendinfo *s;
    uint8_t loopback = 0;

    irq = cpu_irq_save();
    if (tx_buffers_used > 0 && !dma_tx_in_progress && (pdca_get_channel_status(DMA_UART_TX_CHANNEL) == false)) {
        cpu_irq_restore(irq);
        hf_trace(TR_UART_OUTGOING_BEGIN, tx_buffers_used);
        h = (struct hf_header *) tx_tail;
        s = (struct uart_sendinfo *) tx_tail;
        s--;
        len = 8;
        if (h->data_length) {
            /* Don't include CRC-32 yet - might be USB operation. */
            len += (h->data_length * 4);
        }
        switch (h->operation_code) {
        /*
         * The first thing a host sends to us over a USB link, to initialize
         * communications.
         */
        case OP_POWER:
            /* For diagnostic use. */
            if (le16_to_cpu(h->hdata) & DIAGNOSTIC_POWER_OFF) {
                usb_powerdown_request();
                len = 0;
                break;
            } else if (le16_to_cpu(h->hdata) & DIAGNOSTIC_POWER_ON) {
                if (system_on()) {
                    len = sizeof(*h);
                    loopback = 1;
                    break;
                } else
                    power_up_only = true;
            }
            /* no break */

        case OP_USB_INIT:
            host_watchdog_clock = HOST_WATCHDOG_RECHARGE;
            /* no break */
            /* no break */

#ifdef FEATURE_CHARACTERIZATION
        case OP_CHARACTERIZE:
#endif /* FEATURE_CHARACTERIZATION */
            /*
             * Same initial actions occur for characterization
             */
            chain_usb_init((struct hf_usb_init_header *) h); // This sets info->usb_init_header which disables
            got_shutdown = 0;
            flush_all: irq = cpu_irq_save();

            tx_head = tx_buffer_first;
            tx_tail = tx_buffer_first;
            tx_buffers_used = 0;

            /* TODO
             * Disable receiving, could even be in the middle of packet DMA.
             */
            rx_head = &uart_rx_buffer[0];
            rx_tail = &uart_rx_buffer[0];
            /* Flush EVERYTHING in the receive queue. */
            rx_buffers_used = 0;

            info->connected = 1;
            cpu_irq_restore(irq);
            /* Stand on any old uart_sendinfo data. */
            memset(uart_tx_buffer, 0, sizeof(uart_tx_buffer));

            /* Power up if we are not. */
            usb_powerup_request();
            ACTIVITY_LED_ON;

            led_mode = LED_STATIC;
            return;
            break;

        case OP_WORK_RESTART:
            if (info->work_restart_in_progress == false && info->gwq_enabled == true && (len = gwq_work_restart(h)) != 0) {
                /* GWQ mode
                 * gwq_work_restart() actually returns 2 packets, an OP_CONFIG
                 * and an OP_ABORT. They both get sent (back-to-back). The
                 * OP_CONFIG stops periodic status reports, and instead switches
                 * to status triggered on cores going idle. This is put back
                 * later when all jobs have completed.
                 */
                ucinfo.work_restart_requested = false;
            } else if (info->work_restart_in_progress == false && info->gwq_enabled == false && (len = ums_clock_change(h)) != 0) {
                /* UMS mode
                 * Normally won't return a packet so we'll toss the packet.
                 * Normal GWQ engine will do the requested clock change.
                 */
                ucinfo.work_restart_requested = false;
            } else {
                /*
                 * Toss the request. Already have a work restart going on, or
                 * there is no work active, or it's UMS.
                 */
                ucinfo.work_restart_requested = false;
                tx_tail += TX_BUFFER_SIZE;
                if (tx_tail >= tx_buffer_limit)
                    tx_tail = tx_buffer_first;
                irq = cpu_irq_save();
                tx_buffers_used--;
                cpu_irq_restore(irq);
                return;
            }
            break;

#ifdef FEATURE_COWARDLY_WORK_RESTART
        case OP_ABORT:
            if (h->data_length) {
                /*
                 * Multiple back-to-back OP_ABORT's, CRC-8 correct in first one
                 * once data_length cleared.
                 * h->data_length is number of back-to-back aborts.
                 */
                len = h->data_length * sizeof(struct hf_header);
                h->data_length = 0;
            }
            break;
#endif /* FEATURE_COWARDLY_WORK_RESTART */

        case OP_USB_SHUTDOWN:
            got_shutdown = 1;
            /* This sets info->usb_init_header which disables. */
            chain_usb_init((struct hf_usb_init_header *) h);
            goto flush_all;
            break;

#ifdef INCLUDE_TRACING
            case OP_GET_TRACE:
            h = &header;
            h->data_length = (TRACE_RECORDS*sizeof(struct tracerecord_t)/4);
            irq = cpu_irq_save();
            h->hdata = (uint16_t)(TRACE_HEAD_OFFSET);
            h->crc8 = hf_crc8(h);
            cpu_irq_restore(irq);
            CDC_Device_SendData(&VirtualSerial1_CDC_Interface, h, sizeof(*h));

            /* toss the request */
            tx_tail += TX_BUFFER_SIZE;
            if (tx_tail >= tx_buffer_limit)
            tx_tail = tx_buffer_first;
            irq = cpu_irq_save();
            tx_buffers_used--;
            cpu_irq_restore(irq);

            CDC_Device_SendData(&VirtualSerial1_CDC_Interface, &trace_records, TRACE_RECORDS * sizeof(struct tracerecord_t));
            CDC_Device_Flush(&VirtualSerial1_CDC_Interface);
            return;
            break;
#endif /* INCLUDE_TRACING */

        case OP_LOOPBACK_USB:
            if (main_b_cdc_enable == false) {
                sts = udi_cdc_write_buf(tx_tail, len);
                // XXX
                stats.usb_tx_attempts++;
                stats.usb_tx_packets++;
            }
            tx_tail += TX_BUFFER_SIZE;
            if (tx_tail >= tx_buffer_limit)
                tx_tail = tx_buffer_first;
            irq = cpu_irq_save();
            tx_buffers_used--;
            cpu_irq_restore(irq);
            return;
            break;

        /*
         * Enter the boot loader.
         * TODO Might need some more protections here.
         */
        case OP_DFU:
            /* ask slaves to reboot into loader mode */
            //twi_broadcast(TWICMD_REBOOT, 1);
            /*
             * Tell Atmel DFU loader not to start app ever again (harmless with
             * custom loader).
             */
            flashc_erase_gp_fuse_bit(31, true);
            flashc_write_gp_fuse_bit(31, true);
            /*
             * tell custom bootloader not to start app on this boot (harmless
             * with Atmel DFU loader).
             */
            AVR32_PM.gplp[1] = 0x73746179;
            /* never returns */
            self_reset();
            break;

        /*
         * Echo the packet received.
         */
        case OP_PING:
            len = op_ping(h);
            loopback = 1;
            break;

        /*
         * Return the number of cores and the core map.
         */
        case OP_CORE_MAP:
            len = send_core_map(h);
            loopback = 1;
            break;

        /*
         * Return version information.
         */
        case OP_VERSION:
            len = op_version(h);
            loopback = 1;
            break;

        /*
         * Write or read die settings.
         */
        case OP_SETTINGS:
            len = hf_nvram_op_die_settings(h);
            loopback = 1;
            break;

        case OP_FAN:
            fan_set(h->chip_address, h->core_address, le16_to_cpu(h->hdata));
            len = 0;
            loopback = 1;
            break;

        case OP_NAME:
            len = hf_nvram_op_name(h);
            loopback = 1;
            break;

#ifdef FEATURE_FACTORY_TESTS
        /*
         * Factory operations
         */

        case OP_SERIAL:
            len = hf_nvram_op_serial(h);
            loopback = 1;
            break;

        case OP_LIMITS:
            len = hf_nvram_op_limits(h);
            loopback = 1;
            break;

        case OP_HISTORY:
            len = hf_nvram_op_history(h);
            loopback = 1;
            break;

        case OP_FAN_SETTINGS:
            len = hf_nvram_fan_settings(h);
            loopback = 1;
            break;

        case OP_BAD_CORE:
            len = hf_nvram_op_bad_core(h);
            loopback = 1;
            break;
#endif /* FEATURE_FACTORY_TESTS */

        default:
            break;
        }

        if (loopback) {
            /* operations which loop straight back to the host */
            loopback = 0;
            if (len && main_b_cdc_enable == true)
                udi_cdc_write_buf(h, len);
            tx_tail += TX_BUFFER_SIZE;
            if (tx_tail >= tx_buffer_limit)
                tx_tail = tx_buffer_first;
            irq = cpu_irq_save();
            tx_buffers_used--;
            cpu_irq_restore(irq);
        } else {
            if (s->code == US_REPEAT) {
                gwq_process_hash_repeat(s);
            }
            if (h->data_length) {
                /* compute and plug in the CRC-32 */
                hf_crc32((uint8_t *) (h + 1), ((uint16_t) h->data_length) * 4, 1);
                len += 4;
            }

            irq = cpu_irq_save();
            dma_tx_in_progress++;
            cpu_irq_restore(irq);
            /* initialize multiple header pointer */
            mh = h;
            uart_tx_dma_init((uint8_t *) h, len);
        }
    } else {
        cpu_irq_restore(irq);
    }

}

/**
 * Called with VBUS changes
 * @param vbus_level
 */
void usb_vbus_event(bool vbus_level) {
    if (vbus_level == false) {
        /* someone pulled the USB plug */
        usb_powerdown_request();
    }
}

/**
 * Bus reset
 */
void usb_bus_reset(void) {
    /*
     * Note that bus reset is a normal part of USB enumeration, so do not take
     * action on it without qualifying it in some way, such as the cdc interface
     * having been configured (which is what usb_iface_cfg indicates).
     */
    if (usb_iface_cfg)
        usb_powerdown_request();
}

static pdca_channel_options_t pdca_tx;

/**
 * DMA handler for sending packets out the UART
 * Called when a DMA transmit transfer has completed
 */
__attribute__((__interrupt__)) static void pdca_tx_int_handler(void) {
    //uint32_t status;
    struct uart_sendinfo *s = (struct uart_sendinfo *) tx_tail;

    profileEnter(PROFILE_CHANNEL_UARTTXDMA_ISR);
    stats.uart_tx_interrupts++;
    //status = pdca_get_transfer_status(DMA_UART_TX_CHANNEL);
    pdca_disable_interrupt_transfer_complete(DMA_UART_TX_CHANNEL);
    pdca_disable(DMA_UART_TX_CHANNEL);

    s--;
    if (s->code == US_MULTIPLE) {
        /* send multiple, concatenated packets */
        mh += sizeof(*mh) + (mh->data_length << 2);
        if (mh->preamble == HF_PREAMBLE) {
            uart_tx_dma_init((uint8_t *) mh, sizeof(*mh) + (mh->data_length << 2));
            profileExit(PROFILE_CHANNEL_UARTTXDMA_ISR);
            return;
        } else
            s->code = 0;
    }

    if (!s->code) {
        tx_tail += TX_BUFFER_SIZE;
        if (tx_tail >= tx_buffer_limit)
            tx_tail = tx_buffer_first;
        tx_buffers_used--;
    }

    //tmp = 0;
    //while (!(GN_UART->csr & (1<<AVR32_USART_CSR_TXEMPTY_OFFSET)))
    //    tmp++;

    dma_tx_in_progress--;
    profileExit(PROFILE_CHANNEL_UARTTXDMA_ISR);
}

/**
 * UART tx DMA initialize
 * @param buf
 * @param len
 */
static void uart_tx_dma_init(uint8_t *buf, int len) {
    static bool once = true;

    if (once) {
        // XXX move this out to a one-time initialization
        once = false;
        pdca_tx.addr = (void *) buf;
        pdca_tx.size = len;
        pdca_tx.pid = AVR32_PDCA_PID_USART1_TX;
        pdca_tx.r_addr = NULL;
        pdca_tx.r_size = 0;
        pdca_tx.transfer_size = PDCA_TRANSFER_SIZE_BYTE;

        stats.uart_tx_queue_dma++;

        pdca_init_channel(DMA_UART_TX_CHANNEL, &pdca_tx);
        pdca_disable_interrupt_reload_counter_zero(DMA_UART_TX_CHANNEL);
        pdca_enable_interrupt_transfer_complete(DMA_UART_TX_CHANNEL);
        pdca_enable(DMA_UART_TX_CHANNEL);
    } else {
        pdca_reload_channel(DMA_UART_TX_CHANNEL, (void *) buf, len);
        pdca_enable_interrupt_transfer_complete(DMA_UART_TX_CHANNEL);
        pdca_enable(DMA_UART_TX_CHANNEL);
    }
}

/*
 * UART receive
 */

static pdca_channel_options_t pdca_rx;

static inline void dma_RX_header_done(uint32_t);
static inline void dma_RX_data_done(uint32_t);
static void dma_uart_rx_initialize(void);

static uint8_t rx_uart_mode = RX_UART_DISABLED;
static uint8_t buffer_already_processed = 0;
static uint8_t pass_up_to_host = 1;
static uint8_t consecutive_usb_write_timeouts;

/* for debug */
static iram_size_t usb_write_space_available;
static uint16_t total_sent;

/**
 * Check UART incoming
 */
void CheckUartIncoming(void) {
    irqflags_t irq;
    int nb;
    static uint8_t flip;
    struct hf_header *h;
    struct ucinfo_t *info = &ucinfo;
    uint32_t crc32, *p;
    int sts;

    switch (rx_uart_mode) {
    case RX_UART_DISABLED:
        dma_uart_rx_initialize();
        rx_uart_mode = RX_UART_ENABLED;
        break;

    case RX_UART_ENABLED:
    default:
        break;
    }

    if (disable_host) {
        buffer_already_processed = 0;
        /* for next restart */
        pass_up_to_host = 1;
        return;
    }

    /*
     * If we're forwarding to the USB port, do so if we can.
    */
    total_sent = 0;
    irq = cpu_irq_save();
    while (rx_buffers_used > 0) {
        cpu_irq_restore(irq);
        h = (struct hf_header *) rx_tail;

        if (!buffer_already_processed) {

            /* If it's a long packet, validate the CRC-32. */
            if (h->data_length) {
                p = (uint32_t *) (h + 1);
                crc32 = hf_crc32((uint8_t *) p, h->data_length * 4, 0);
                p += h->data_length;
                if (le32_to_cpu(*p) != crc32) {
                    uart_crc32_errors++;
                    stats.uart_rx_bad_crc32++;
                    /* toss the buffer */
                    rx_tail += RX_BUFFER_SIZE;
                    if (rx_tail >= rx_buffer_limit)
                        rx_tail = uart_rx_buffer;
                    irq = cpu_irq_save();
                    rx_buffers_used--;
                    cpu_irq_restore(irq);
                    continue;
                }
            }

            if (info->gwq_enabled)
                pass_up_to_host = gwq_process_input(h);
        }

        if (((!info->connected || !pass_up_to_host) && h->operation_code != OP_LOOPBACK_UART) || info->usb_init_header) {
            /*
             * Toss the buffer, nowhere to send it or it doesn't belong to the
             * host yet.
             */
            rx_tail += RX_BUFFER_SIZE;
            if (rx_tail >= rx_buffer_limit)
                rx_tail = uart_rx_buffer;
            irq = cpu_irq_save();
            rx_buffers_used--;
            cpu_irq_restore(irq);
            continue;
        }

        /* Ensure a send will actually work, so we don't stall below. */
        usb_write_space_available = udi_cdc_get_free_tx_buffer() - total_sent;
        if (usb_write_space_available < 64) {
            buffer_already_processed = 1;
            break;
        }

        buffer_already_processed = 0;
        /*
         * We've got one or more packets to send to the host.
         * Header CRC-8 has already been checked.
         */
        if (h->operation_code == OP_HASH && h->data_length == 15)      // XXX
        {
            /* drop the spare3 field */
            h->data_length--;
            /* fix header CRC-8 */
            h->crc8 = hf_crc8((uint8_t *) h);
        }
        nb = h->data_length << 2;
        /* drop CRC-8 */
        nb = (nb) ? nb + 8 : 8;
        total_sent += nb;
        //if (total_sent > 64)                      // XXX Avert LUFA partial tx bug
        //    return; // XXX goto flush_and_return; // Enough for now, don't exceed frame size
        stats.usb_tx_attempts++;
        if (main_b_cdc_enable == true) {
            sts = udi_cdc_write_buf((const void *) rx_tail, nb);

            if (sts == 0) {
                stats.usb_tx_packets++;
                // XXX LEDs_SetAllLEDs(LEDS_LED1 & flip);
                flip = ~flip;
                rx_tail += RX_BUFFER_SIZE;
                if (rx_tail >= rx_buffer_limit)
                    rx_tail = uart_rx_buffer;
                irq = cpu_irq_save();
                rx_buffers_used--;
                consecutive_usb_write_timeouts = 0;
                continue;
            } else if (sts == nb) {
                stats.usb_tx_timeouts++;
                if (++consecutive_usb_write_timeouts > 8 /* && !try_usb_forever */) {
                    shutdown_request();
                }
            } else if (sts < nb) {
                stats.usb_tx_incompletes++;
            }

#if 0
            stats.usb_tx_endpointstalled++;
            stats.usb_tx_disconnected++;
            stats.usb_tx_suspended++;
#endif /* 0 */
        }
        /* Here for errors. Always toss the buffer, and bail. */
        rx_tail += RX_BUFFER_SIZE;
        if (rx_tail >= rx_buffer_limit)
            rx_tail = uart_rx_buffer;
        irq = cpu_irq_save();
        rx_buffers_used--;
        break;
    }
    cpu_irq_restore(irq);

    /* Send an OP_USB_STATS1 around every 10 seconds or so, if we have room. */
    if (last_stats1_sent && ucinfo.connected && seconds_elapsed_since(last_stats1_sent) >= 10 && !disable_host) {
        usb_write_space_available = udi_cdc_get_free_tx_buffer();
        if (usb_write_space_available >= (sizeof(struct hf_header) + sizeof(struct hf_usb_stats1))) {
            send_stats1();
            last_stats1_sent = (sec_ticker) ? sec_ticker : 1;
        }
    }
}

static void set_up_pdca_rx(void *, int);

static uint8_t dma_RX_doing_header = 1;

static int dd_rxint_in;
static int dd_rxint_out;

/**
 * PDCA rx interrupt request
 */
__attribute__((__interrupt__)) static void pdca_rx_int_handler(void) {
    uint32_t status = pdca_get_transfer_status(DMA_UART_RX_CHANNEL);

    /* TODO
     * Doing this profile appears to result in data loss; probably normal as the
     * profileEnter() delays the setup of the next DMA and there's only the
     * one-byte-holding register.
     */
    //profileEnter(PROFILE_CHANNEL_UARTRXDMA_ISR);
    dd_rxint_in++;

    if (dma_RX_doing_header)
        dma_RX_header_done(status);
    else
        dma_RX_data_done(status);

    dd_rxint_out++;
    //profileExit(PROFILE_CHANNEL_UARTRXDMA_ISR);
}

/**
 * Setup PDCA rx
 * @param buf
 * @param len
 */
void set_up_pdca_rx(void *buf, int len) {
    static bool once = true;

    if (once) {
        pdca_rx.addr = buf;
        pdca_rx.size = len;
        pdca_rx.pid = AVR32_PDCA_PID_USART1_RX;
        pdca_rx.r_addr = NULL;
        pdca_rx.r_size = 0;
        pdca_rx.transfer_size = PDCA_TRANSFER_SIZE_BYTE;

        pdca_init_channel(DMA_UART_RX_CHANNEL, &pdca_rx);
        pdca_disable_interrupt_reload_counter_zero(DMA_UART_RX_CHANNEL);
        pdca_enable_interrupt_transfer_complete(DMA_UART_RX_CHANNEL);
        pdca_enable(DMA_UART_RX_CHANNEL);
        once = false;
    } else {
        pdca_reload_channel(DMA_UART_RX_CHANNEL, (void *) buf, len);
        pdca_enable_interrupt_transfer_complete(DMA_UART_RX_CHANNEL);
        pdca_enable(DMA_UART_RX_CHANNEL);
    }
}

static int value;
static int false_values;
static int rx_ints;

/**
 * USART interrupt request
 */
__attribute__((__interrupt__)) static void usart_interrupt(void) {
    volatile uint8_t *addr;

    /* TODO
     * Profiling this function seems to result in a crash. It could certainly be
     * that this overhead is too large so we drop data, which could prevent the
     * system from operating, but it shouldn't crash.
     */
    //profileEnter(PROFILE_CHANNEL_UART_ISR);
    rx_ints++;

    if (GN_UART->csr & AVR32_USART_CSR_RXRDY_MASK) {
        bool b_error = (USART_SUCCESS != usart_read_char(GN_UART, &value));

        if (b_error) {
            usart_reset_status(GN_UART);
            udi_cdc_signal_framing_error();
            //ui_com_error();
        } else {
            if ((char) value == HF_PREAMBLE) {
                if (rx_buffers_used >= RX_BUFFERS) {
                    /* no buffer available */
                    stats.uart_rx_missed_preamble_ints++;
                } else {
                    hf_trace(TR_UART_RX_INTERRUPT_PREAMBLE, 0);
                    stats.uart_rx_preamble_ints++;
                    /*
                     * A candidate frame.
                     * Switch to DMA mode - 7 bytes remaining!
                     */
                    GN_UART->idr = AVR32_USART_IER_RXRDY_MASK;
                    *rx_head = HF_PREAMBLE;
                    addr = rx_head + 1;
                    /* DMA the remaining 7 bytes */
                    set_up_pdca_rx((void *) addr, 7);
                    hf_trace(TR_UART_RX_DMA_START, 7);
                }
            } else
                false_values++;
        }
    }
    //profileExit(PROFILE_CHANNEL_UART_ISR);
}

/**
 * Called back when a DMA receive transfer for a packet body has completed.
 * @param status
 */
static void dma_RX_data_done(uint32_t status) {
    uint32_t final_addr;

    hf_trace(TR_UART_RX_DMA_DATA_DONE, (uint8_t)status);
    stats.uart_rx_data_done++;
    if (status & PDCA_TRANSFER_COMPLETE) {
#if 0
        /* Some integrity checking. I've seen some partial DMA transfers. */
        {
            struct hf_header *h = (struct hf_header *)rx_head;

            if (xfer != h->data_length*4+4)
            {
                stats.uart_rx_short_dma++;
                uprintf(1, "rxdma: h 0x%x hdata %x data_length %d CTRLB %x TRFCNT %d", h, h->hdata, h->data_length, ctrlb, xfer);
                goto re_arm;
            }
        }
#endif /* 0 */
        final_addr = AVR32_PDCA.channel[DMA_UART_RX_CHANNEL].mar;
        if (final_addr != (uint32_t) rx_head + 8 + rx_head[6] * 4 + 4) {
            uart_rx_data_addr_bad++;
#if 0
            usb_debug_stream_printf("unexpected uart rx data dma"
                    " final addr"
                    " (0x%08x, expected 0x%08x)\n",
                    (unsigned int) final_addr,
                    (unsigned int) rx_head + 8 +
                    rx_head[6] * 4 + 4);
            usb_debug_stream_printf("rx buffers start 0x%08x "
                    "rx_head 0x%08x\n",
                    (unsigned int) uart_rx_buffer,
                    (unsigned int) rx_head);
#endif /* 0 */
        }
        /* Long packet is done. Move it out for processing. */
        rx_head += RX_BUFFER_SIZE;
        if (rx_head >= rx_buffer_limit)
            rx_head = &uart_rx_buffer[0];
        if (++rx_buffers_used > stats.max_rx_buffers)
            stats.max_rx_buffers = rx_buffers_used;
        if (rx_buffers_used >= RX_BUFFERS) {
            /*
             * We're full and must stop.
             * Should never happen, UART (slow) -> USB (fast).
             */
            stats.uart_rx_buffers_full++;
            dma_RX_doing_header = 1;
            pdca_disable_interrupt_transfer_complete(DMA_UART_RX_CHANNEL);
            pdca_disable(DMA_UART_RX_CHANNEL);
            GN_UART->ier = AVR32_USART_IER_RXRDY_MASK;
        } else {
/* re-arm receive */
//re_arm:
            dma_RX_doing_header = 1;
            pdca_reload_channel(DMA_UART_RX_CHANNEL, (void *) rx_head, 8);
            //pdca_enable_interrupt_transfer_complete(DMA_UART_RX_CHANNEL);
            //pdca_enable(DMA_UART_RX_CHANNEL);
        }
    } else {
        /* Bad DMA. Re-arm interrupt, look for a new preamble byte. */
        GN_UART->ier = AVR32_USART_IER_RXRDY_MASK;
        pdca_disable_interrupt_transfer_complete(DMA_UART_RX_CHANNEL);
        pdca_disable(DMA_UART_RX_CHANNEL);
        dma_RX_doing_header = 1;
    }
}

/**
 * Called back when a DMA receive transfer for a header has completed.
 * XXX Might be a bit long for an ISR.
 * @param status
 */
static inline void dma_RX_header_done(uint32_t status) {
    uint8_t crc8;
    uint8_t dlen;
    uint8_t volatile *addr;
    uint32_t final_addr;

    hf_trace(TR_UART_RX_DMA_HEADER_DONE, (uint8_t)status);
    stats.uart_rx_header_done++;

    if (status & PDCA_TRANSFER_COMPLETE) {
        final_addr = AVR32_PDCA.channel[DMA_UART_RX_CHANNEL].mar;
        if (final_addr != (uint32_t) rx_head + 8) {
            uart_rx_header_addr_bad++;
#if 0
            usb_debug_stream_printf("unexpected uart rx header dma"
                    " final addr"
                    " (0x%08x, expected 0x%08x)\n",
                    (unsigned int) final_addr,
                    (unsigned int) rx_head + 8);
#endif /* 0 */
        }
        /* validate header */
        dlen = *(rx_head + 6);
        crc8 = crc8_table[0xff ^ *(rx_head + 1)];
        crc8 = crc8_table[crc8 ^ *(rx_head + 2)];
        crc8 = crc8_table[crc8 ^ *(rx_head + 3)];
        crc8 = crc8_table[crc8 ^ *(rx_head + 4)];
        crc8 = crc8_table[crc8 ^ *(rx_head + 5)];
        crc8 = crc8_table[crc8 ^ dlen];

        if (*rx_head == HF_PREAMBLE && crc8 == *(rx_head + 7)) {
            if (dlen && dlen <= (RX_BUFFER_SIZE - 8 - 4) / 4) {
                /* setup DMA for the packet body */
                dma_RX_doing_header = 0;
                addr = rx_head + 8;
                pdca_reload_channel(DMA_UART_RX_CHANNEL, (void *) addr, dlen * 4 + 4);
                //pdca_enable_interrupt_transfer_complete(DMA_UART_RX_CHANNEL);
                //pdca_enable(DMA_UART_RX_CHANNEL);
                hf_trace(TR_UART_RX_DMA_START, dlen*4+4);
            } else if (dlen) {
                uart_rx_too_large++;
                pdca_disable_interrupt_transfer_complete(DMA_UART_RX_CHANNEL);
                pdca_disable(DMA_UART_RX_CHANNEL);
                GN_UART->ier = AVR32_USART_IER_RXRDY_MASK;
            } else {
                /* A short packet. Move it out for processing. */
                rx_head += RX_BUFFER_SIZE;
                if (rx_head >= rx_buffer_limit)
                    rx_head = &uart_rx_buffer[0];
                if (++rx_buffers_used > stats.max_rx_buffers)
                    stats.max_rx_buffers = rx_buffers_used;
                if (rx_buffers_used >= RX_BUFFERS) {
                    /*
                     * We're full and must stop.
                     * Should never happen, UART (slow) -> USB (fast).
                     */
                    stats.uart_rx_buffers_full++;
                    pdca_disable_interrupt_transfer_complete(DMA_UART_RX_CHANNEL);
                    pdca_disable(DMA_UART_RX_CHANNEL);
                    GN_UART->ier = AVR32_USART_IER_RXRDY_MASK;
                } else {
#if 1
                    volatile avr32_pdca_channel_t *pdca = &AVR32_PDCA.channel[DMA_UART_RX_CHANNEL];

                    pdca->marr = (uint32_t) rx_head;
                    pdca->tcrr = 8;
                    pdca->cr = AVR32_PDCA_ECLR_MASK;
                    pdca->isr;
#else
                    /* re-arm receive */
                    pdca_reload_channel(DMA_UART_RX_CHANNEL, (void *)rx_head, 8);
                    //pdca_enable_interrupt_transfer_complete(DMA_UART_RX_CHANNEL);
                    //pdca_enable(DMA_UART_RX_CHANNEL);
#endif
                    hf_trace(TR_UART_RX_DMA_START, 8);
                }
            }
        } else {
            /* Bad header CRC. Re-arm interrupt, look for new preamble byte. */
            stats.uart_rx_bad_hcrc++;
            pdca_disable_interrupt_transfer_complete(DMA_UART_RX_CHANNEL);
            pdca_disable(DMA_UART_RX_CHANNEL);
            GN_UART->ier = AVR32_USART_IER_RXRDY_MASK;
        }
    } else {
        /* Bad DMA. Re-arm interrupt, look for new preamble byte. */
        stats.uart_rx_bad_dma++;
        pdca_disable_interrupt_transfer_complete(DMA_UART_RX_CHANNEL);
        pdca_disable(DMA_UART_RX_CHANNEL);
        GN_UART->ier = AVR32_USART_IER_RXRDY_MASK;
    }

}

/**
 * Sets up most of the parameters in the receive DMA channel.
 * Called once at initialization time.
 */
static void dma_uart_rx_initialize() {
    GN_UART->ier = AVR32_USART_IER_RXRDY_MASK;
    dma_RX_doing_header = 1;
}

/*
 * Open / Close the UART
 */

static usart_options_t usart_options;

/**
 * Called by udi_cdc.c
 * @param port
 */
void uart_rx_notify(uint8_t port) {
    /* if UART is open */
#if 0
    if (GN_UART->imr & AVR32_USART_IER_RXRDY_MASK) {
        // Enable UART TX interrupt to send a new value
        USART->ier = AVR32_USART_IER_TXRDY_MASK;
    }
#endif /* 0 */
}

/**
 * Called by udi_cdc.c
 * @param port
 * @param cfg
 */
void uart_config(uint8_t port, usb_cdc_line_coding_t * cfg) {
#ifdef IGNORE_ALL_THIS
    uint32_t stopbits, parity;
    uint32_t imr;

    switch (cfg->bCharFormat) {
        case CDC_STOP_BITS_2:
        stopbits = USART_2_STOPBITS;
        break;
        case CDC_STOP_BITS_1_5:
        stopbits = USART_1_5_STOPBITS;
        break;
        case CDC_STOP_BITS_1:
        default:
        /* default stop bit = 1 stop bit */
        stopbits = USART_1_STOPBIT;
        break;
    }

    switch (cfg->bParityType) {
        case CDC_PAR_EVEN:
        parity = USART_EVEN_PARITY;
        break;
        case CDC_PAR_ODD:
        parity = USART_ODD_PARITY;
        break;
        case CDC_PAR_MARK:
        parity = USART_MARK_PARITY;
        break;
        case CDC_PAR_SPACE:
        parity = USART_SPACE_PARITY;
        break;
        default:
        case CDC_PAR_NONE:
        parity = USART_NO_PARITY;
        break;
    }

    /* ptions for USART */
    usart_options.baudrate = LE32_TO_CPU(cfg->dwDTERate);
    usart_options.charlength = cfg->bDataBits;
    usart_options.paritytype = parity;
    usart_options.stopbits = stopbits;
    usart_options.channelmode = USART_NORMAL_CHMODE;
    imr = USART->imr;
    USART->idr = 0xFFFFFFFF;
    usart_init_rs232(USART, &usart_options, sysclk_get_pba_hz());
    /* restore both RX and TX */
    USART->ier = imr;

#endif /* IGNORE_ALL_THIS */
}

static unsigned long saved_default_baudrate;

/**
 * Open UART
 */
void uart_open() {
    struct ucinfo_t *info = &ucinfo;

    gpio_enable_module_pin(UART_TX, AVR32_USART1_TXD_0_0_FUNCTION);
    gpio_enable_module_pin(UART_RX, AVR32_USART1_RXD_0_0_FUNCTION);

    if (hf_nvram_die_settings_valid()) {
        /*
         * All boards start out at 230,400 baud. This is the only default common
         * baud rate available between 25 Mhz and 125 Mhz reference clock
         * systems. At a later stage, dynamic baud rate of 1.25 Mbaud is set.
         */
        info->ref_frequency = hf_nvram_die_settings()->ref_frequency;
        switch (info->ref_frequency) {
        case 125:
            /* BAUD_RATE_PWRUP_4 = 230,400 */
            usart_options.baudrate = BAUD_RATE_PWRUP_4;
            info->asic_baud_rate_code = 4;
            /* baud delay = 100 clock periods @ 125 Mhz */
            info->dynamic_baud_rate = 1250000;
            break;

        case 25:
            /* BAUD_RATE_PWRUP_7 = 1,152,000. 1,152,000/5 = 230,400 */
            usart_options.baudrate = BAUD_RATE_PWRUP_7 / 5;
            info->asic_baud_rate_code = 7;
            /* baud delay = 20 clock periods @ 25 Mhz */
            info->dynamic_baud_rate = 1250000;
            break;

        default:
            /* TODO
             * Something new? Mixed chaining won't work but otherwise should be
             * okay.
             */
            usart_options.baudrate = BAUD_RATE_PWRUP_7 * info->ref_frequency / DEFAULT_REF_CLOCK;
            info->asic_baud_rate_code = 7;
            info->dynamic_baud_rate = 0;
            break;
        }
    } else {
        /* TODO Obsolete
         * Die settings are always initialized now.
         */
        usart_options.baudrate = BAUD_RATE_PWRUP_7;
    }

    saved_default_baudrate = usart_options.baudrate;

    usart_options.charlength = 8;
    usart_options.paritytype = USART_NO_PARITY;
    usart_options.stopbits = USART_1_STOPBIT;
    usart_options.channelmode = USART_NORMAL_CHMODE;

    /* initialize in RS232 mode */
    sysclk_enable_pba_module(USART_SYSCLK);
    if (USART_SUCCESS != usart_init_rs232(GN_UART, &usart_options, sysclk_get_pba_hz()))
        return;

    //* enable RX interrupt */
    //Disable_global_interrupt();
    INTC_register_interrupt((__int_handler ) &usart_interrupt, AVR32_USART1_IRQ, AVR32_INTC_INT3);
    INTC_register_interrupt((__int_handler) &pdca_rx_int_handler, AVR32_PDCA_IRQ_2, AVR32_INTC_INT3);
    INTC_register_interrupt((__int_handler) &pdca_tx_int_handler, AVR32_PDCA_IRQ_1, AVR32_INTC_INT1);
    GN_UART->ier = AVR32_USART_IER_RXRDY_MASK;
    //Enable_global_interrupt();
}

/**
 * Set UART baudrate
 * @param baudrate
 */
void uart_set_baudrate(unsigned long baudrate) {
    uint32_t imr;

    usart_options.baudrate = baudrate;
    usart_options.charlength = 8;
    usart_options.paritytype = USART_NO_PARITY;
    usart_options.stopbits = USART_1_STOPBIT;
    usart_options.channelmode = USART_NORMAL_CHMODE;

    imr = GN_UART->imr;
    GN_UART->idr = 0xFFFFFFFF;
    usart_init_rs232(GN_UART, &usart_options, sysclk_get_pba_hz());
    GN_UART->ier = imr;
}

/**
 * Set UART default baudrate
 */
void uart_set_default_baudrate() {
    if (saved_default_baudrate)
        uart_set_baudrate(saved_default_baudrate);
}

/**
 * Close UART
 */
void uart_close() {
    /* disable interrupts */
    GN_UART->idr = 0xffffffff;
}

/*
 * Functions for local packet send/receive.
 */

/**
 * Enable ASIC local mode
 */
void asic_enable_local() {
    disable_host = 1;
}

/**
 * Enable ASIC host mode
 */
void asic_enable_host() {
    disable_host = 0;
}

#if 0

/**
 * ASIC mode
 * @return
 */
bool inline asic_host_enabled(void) {
    return (disable_host) ? false : true;
}

#endif

/**
 * Get ASIC TX buffer
 * @return
 */
struct hf_header *asic_get_tx_buffer() {
    struct hf_header *t;
    irqflags_t irq;

    irq = cpu_irq_save();
    if (tx_buffers_used < TX_BUFFERS)
        t = (struct hf_header *) tx_head;
    else
        t = (struct hf_header *) NULL;
    cpu_irq_restore(irq);

    return (t);
}

/**
 * ASIC TX queue
 */
void asic_queue_transmit() {
    irqflags_t irq;

    //hf_trace(TR_USB_INCOMING_DISPATCH_LONG, dlen);
    tx_head += TX_BUFFER_SIZE;
    if (tx_head >= tx_buffer_limit)
        tx_head = tx_buffer_first;
    irq = cpu_irq_save();
    if (++tx_buffers_used > stats.max_tx_buffers)
        stats.max_tx_buffers = tx_buffers_used;
    cpu_irq_restore(irq);
}

/**
 * Get ASIC TX queue count
 * @return
 */
uint8_t asic_get_transmit_count() {
    return (tx_buffers_used);
}

/**
 * Get ASIC RX queue count
 * @return
 */
uint8_t asic_get_receive_count() {
    return (rx_buffers_used);
}

/**
 * ASIC get RX buffers
 * @return
 */
struct hf_header *asic_get_receive() {
    irqflags_t irq;

    irq = cpu_irq_save();
    if (rx_buffers_used > 0) {
        cpu_irq_restore(irq);
        // We've got one or more packets to send to the host.
        return ((struct hf_header *) rx_tail);

    }
    cpu_irq_restore(irq);

    return ((struct hf_header *) NULL);
}

/**
 * Pop a packet from the ASIC RX queue
 */
void asic_pop_receive() {
    irqflags_t irq;

    irq = cpu_irq_save();
    if (rx_buffers_used > 0) {
        rx_tail += RX_BUFFER_SIZE;
        if (rx_tail >= rx_buffer_limit)
            rx_tail = uart_rx_buffer;
        rx_buffers_used--;
    }
    cpu_irq_restore(irq);
}

/**
 * Send statistics
 */
static void send_stats1() {
    struct {
        struct hf_header h;
        struct hf_usb_stats1 stats_le;
    }__attribute__((packed)) frame, *s = &frame;

    if (main_b_cdc_enable == false || !ucinfo.connected) {
        memset(&stats, 0, sizeof(stats));
        return;
    }

    memset(s, 0, sizeof(*s));
    s->h.preamble = HF_PREAMBLE;
    s->h.operation_code = OP_USB_STATS1;
    s->h.data_length = (sizeof(struct hf_usb_stats1) + 3) / 4;
    s->h.crc8 = hf_crc8((uint8_t *) s);

    s->stats_le.usb_rx_preambles = cpu_to_le16(stats.usb_rx_preambles);
    s->stats_le.usb_rx_receive_byte_errors = cpu_to_le16(stats.usb_rx_receive_byte_errors);
    s->stats_le.usb_rx_bad_hcrc = cpu_to_le16(stats.usb_rx_bad_hcrc);
    s->stats_le.usb_tx_attempts = cpu_to_le16(stats.usb_tx_attempts);
    s->stats_le.usb_tx_packets = cpu_to_le16(stats.usb_tx_packets);
    s->stats_le.usb_tx_timeouts = cpu_to_le16(stats.usb_tx_timeouts);
    s->stats_le.usb_tx_incompletes = cpu_to_le16(stats.usb_tx_incompletes);
    s->stats_le.usb_tx_endpointstalled = cpu_to_le16(stats.usb_tx_endpointstalled);
    s->stats_le.usb_tx_disconnected = cpu_to_le16(stats.usb_tx_disconnected);
    s->stats_le.usb_tx_suspended = cpu_to_le16(stats.usb_tx_suspended);
    s->stats_le.uart_tx_queue_dma = cpu_to_le16(stats.uart_tx_queue_dma);
    s->stats_le.uart_tx_interrupts = cpu_to_le16(stats.uart_tx_interrupts);
    s->stats_le.uart_rx_preamble_ints = cpu_to_le16(stats.uart_rx_preamble_ints);
    s->stats_le.uart_rx_missed_preamble_ints = cpu_to_le16(stats.uart_rx_missed_preamble_ints);
    s->stats_le.uart_rx_header_done = cpu_to_le16(stats.uart_rx_header_done);
    s->stats_le.uart_rx_data_done = cpu_to_le16(stats.uart_rx_data_done);
    s->stats_le.uart_rx_bad_hcrc = cpu_to_le16(stats.uart_rx_bad_hcrc);
    s->stats_le.uart_rx_bad_crc32 = cpu_to_le16(stats.uart_rx_bad_crc32);
    //s->stats_le.bad_sequence = cpu_to_le16(stats.bad_sequence);
    s->stats_le.uart_rx_bad_dma = cpu_to_le16(stats.uart_rx_bad_dma);
    s->stats_le.uart_rx_short_dma = cpu_to_le16(stats.uart_rx_short_dma);
    s->stats_le.uart_rx_buffers_full = cpu_to_le16(stats.uart_rx_buffers_full);
    s->stats_le.max_tx_buffers = cpu_to_le16(stats.max_tx_buffers);
    s->stats_le.max_rx_buffers = cpu_to_le16(stats.max_rx_buffers);

    memset(&stats, 0, sizeof(stats));

    udi_cdc_write_buf(s, sizeof(*s));
}

#ifdef FEATURE_DEBUG_CLI

/**
 * UART statistics
 * @param first
 * @param paramCount
 * @param params
 * @return success
 */
int uart_cli_stats(int first, int parm_count, uint32_t *parms) {
    static int chunk;
    int done;

    if (first)
        chunk = 0;

    done = 0;
    switch (chunk++) {
    case 0:
        cliWriteString("rx:\n");
        cliWriteString(" preamble ints ");
        cliWriteChawmpHex(stats.uart_rx_preamble_ints);
        cliWriteString(" missed ");
        cliWriteChawmpHex(stats.uart_rx_missed_preamble_ints);
        cliWriteChar('\n');
        break;
    case 1:
        cliWriteString(" header done ");
        cliWriteChawmpHex(stats.uart_rx_header_done);
        cliWriteString(" data done ");
        cliWriteChawmpHex(stats.uart_rx_data_done);
        cliWriteString(" bad hcrc ");
        cliWriteChawmpHex(stats.uart_rx_bad_hcrc);
        cliWriteString(" too large ");
        cliWriteChawmpHex(uart_rx_too_large);
        cliWriteChar('\n');
        break;
    case 2:
        cliWriteString(" bad crc32 ");
        cliWriteChawmpHex(uart_crc32_errors);
        cliWriteChar('\n');
        break;
    case 3:
        cliWriteString(" bad header final addr ");
        cliWriteChawmpHex(uart_rx_header_addr_bad);
        cliWriteString(" bad data final addr ");
        cliWriteChawmpHex(uart_rx_data_addr_bad);
        cliWriteChar('\n');
        break;
    case 4:
        cliWriteString(" bad dma ");
        cliWriteChawmpHex(stats.uart_rx_bad_dma);
        cliWriteString(" short dma ");
        cliWriteChawmpHex(stats.uart_rx_short_dma);
        cliWriteString("\n buffers full ");
        cliWriteChawmpHex(stats.uart_rx_buffers_full);
        cliWriteChar('\n');
        /* no break */
    default:
        done = 1;
        break;
    }

    return done;
}

/**
 * USB statistics
 * @param first
 * @param paramCount
 * @param params
 * @return success
 */
int usb_cli_stats(int first, int parm_count, uint32_t *parms) {
    static int chunk;
    int done;

    if (first)
        chunk = 0;

    done = 0;
    switch (chunk++) {
    case 0:
        cliWriteString("rx:\n");
        cliWriteString(" preambles ");
        cliWriteChawmpHex(stats.usb_rx_preambles);
        cliWriteString(" sync loss ");
        cliWriteChawmpHex(usb_sync_loss);
        cliWriteChar('\n');
        break;
    case 1:
        cliWriteString(" bad header crc ");
        cliWriteChawmpHex(stats.usb_rx_bad_hcrc);
        cliWriteChar('\n');
        /* no break */
    default:
        done = 1;
        break;
    }

    return done;
}
#endif /* FEATURE_DEBUG_CLI */
