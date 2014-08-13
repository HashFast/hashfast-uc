/** @file usb_uart.h
 * @brief USB / UART interfaces
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

/* USB send buffers */
#define RX_BUFFERS                  32
#define RX_BUFFER_SIZE              72

extern volatile bool main_b_cdc_enable;

/**
 * UART Send Information
 * Used to emulate the broken ntime rolling feature by sending the same hash job
 * with adjusted ntime offsets to multiple targets.
 */
struct uart_sendinfo {
    uint8_t code;                               // 0 = Do nothing, single send
    uint8_t ntime;
    uint8_t ntime_limit;
    uint8_t spare1;
    uint16_t core_index;
    uint16_t spare2;
}__attribute__((packed,aligned(4)));

/* Values for "code", 0 means no special treatment */
#define US_REPEAT                   1
#define US_MULTIPLE                 2

/* UART transmit buffers, which are also USB receive buffers */
#define TX_BUFFERS                  64

/* AP:
 * That's 80 bytes per frame. I've seen a potential for an overrun so I'm sizing
 * this up to 128 bytes per tx buffer for now.
 */
//#define TX_BUFFER_SIZE              (sizeof(struct hf_header) + sizeof(struct hf_hash_serial) + 4 + sizeof(struct uart_sendinfo))
#define TX_BUFFER_SIZE              128

void asic_enable_local(void);
void asic_enable_host(void);
//bool inline asic_host_enabled(void);
struct hf_header *asic_get_tx_buffer(void);
void asic_queue_transmit(void);
uint8_t asic_get_receive_count(void);
uint8_t asic_get_transmit_count(void);
struct hf_header *asic_get_receive(void);
void asic_pop_receive(void);
void display_usb_uart_status(void);
int uart_cli_stats(int, int, uint32_t *);
int usb_cli_stats(int, int, uint32_t *);
void usb_vbus_event(bool);
void usb_bus_reset(void);
void uart_set_baudrate(unsigned long);
void uart_set_default_baudrate(void);
