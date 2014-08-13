/** @file hf_trace.h
 * @brief HF traces
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

#ifdef INCLUDE_TRACING

/**
 * Trace record
 */
typedef struct tracerecord_t {
    uint16_t time;              //!< In usec relative
    uint8_t event;
    uint8_t data;
} TRACERECORD_T;

#define TRACE_RECORDS 256

/**
 * Definitions of trace events
 */
enum trace_events_t {
    TR_NULL = 0,                   //!< TR_NULL
    TR_USB_INCOMING_PREAMBLE,      //!< TR_USB_INCOMING_PREAMBLE
    TR_USB_INCOMING_HEADER_CRC,    //!< TR_USB_INCOMING_HEADER_CRC
    TR_USB_INCOMING_DISPATCH_SHORT,//!< TR_USB_INCOMING_DISPATCH_SHORT
    TR_USB_INCOMING_DISPATCH_LONG, //!< TR_USB_INCOMING_DISPATCH_LONG
    TR_UART_OUTGOING_BEGIN,        //!< TR_UART_OUTGOING_BEGIN
    TR_UART_TX_DMA_START,          //!< TR_UART_TX_DMA_START
    TR_UART_TX_DMA_INTERRUPT,      //!< TR_UART_TX_DMA_INTERRUPT
    TR_UART_RX_INTERRUPT_PREAMBLE, //!< TR_UART_RX_INTERRUPT_PREAMBLE
    TR_UART_RX_DMA_START,          //!< TR_UART_RX_DMA_START
    TR_UART_RX_DMA_HEADER_DONE,    //!< TR_UART_RX_DMA_HEADER_DONE
    TR_UART_RX_DMA_DATA_DONE,      //!< TR_UART_RX_DMA_DATA_DONE
    TR_UART_INCOMING_START,        //!< TR_UART_INCOMING_START
    TR_UART_INCOMING_USB_SEND_DONE,//!< TR_UART_INCOMING_USB_SEND_DONE
    TR_USB_INC_TX_BUFFERS,         //!< TR_USB_INC_TX_BUFFERS
    TR_UART_INCOMING_SEQUENCE,     //!< TR_UART_INCOMING_SEQUENCE
    TR_UART_INCOMING_SIZE,         //!< TR_UART_INCOMING_SIZE
    TR_USB_INCOMING_CRC            //!< TR_USB_INCOMING_CRC
};

void hf_trace(uint8_t, uint8_t);
void hf_trace_init(void);

extern struct tracerecord_t trace_records[TRACE_RECORDS];
extern struct tracerecord_t *t_head;

#define TRACE_HEAD_OFFSET ((uint16_t)(t_head - &trace_records[0])/sizeof(struct tracerecord_t))

#else /* INCLUDE_TRACING */

#define hf_trace(a,b)
#define hf_trace_init()

#endif /* INCLUDE_TRACING */

#define SPIN_0          AVR32_PIN_PB10
#define SPIN_1          AVR32_PIN_PB11
#define SPIN_2          AVR32_PIN_PA29
#define SPIN_3          AVR32_PIN_PA30

void a3bu_trace_init(void);

#if 0
#define SCOPE_0_ON   gpio_set_pin_high(SPIN_0);
#define SCOPE_0_OFF  gpio_set_pin_low(SPIN_0);
#define SCOPE_1_ON   gpio_set_pin_high(SPIN_1);
#define SCOPE_1_OFF  gpio_set_pin_low(SPIN_1);
#define SCOPE_2_ON   gpio_set_pin_high(SPIN_2);
#define SCOPE_2_OFF  gpio_set_pin_low(SPIN_2);
#define SCOPE_3_ON   gpio_set_pin_high(SPIN_3);
#define SCOPE_3_OFF  gpio_set_pin_low(SPIN_3);

#define CONFIGURE_SCOPE_PINS \
    gpio_enable_gpio_pin(SPIN_0); \
    gpio_enable_gpio_pin(SPIN_1); \
    gpio_enable_gpio_pin(SPIN_2); \
    gpio_enable_gpio_pin(SPIN_3); \
    gpio_configure_pin(SPIN_0, GPIO_DIR_OUTPUT); \
    gpio_configure_pin(SPIN_1, GPIO_DIR_OUTPUT); \
    gpio_configure_pin(SPIN_2, GPIO_DIR_OUTPUT); \
    gpio_configure_pin(SPIN_3, GPIO_DIR_OUTPUT)

#else /* 0 */

#define SCOPE_0_ON   gpio_set_pin_high(LED_POWER)
#define SCOPE_0_OFF  gpio_set_pin_low(LED_POWER)
#define SCOPE_1_ON   gpio_set_pin_high(LED_ACTIVITY)
#define SCOPE_1_OFF  gpio_set_pin_low(LED_ACTIVITY)
#define SCOPE_2_ON
#define SCOPE_2_OFF
#define SCOPE_3_ON
#define SCOPE_3_OFF

#define CONFIGURE_SCOPE_PINS

#endif /* 0 */
