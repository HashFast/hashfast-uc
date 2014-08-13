/** @file hf_trace.c
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

#include "main.h"

#ifdef INCLUDE_TRACING

struct tracerecord_t trace_records[TRACE_RECORDS];
struct tracerecord_t *t_head = trace_records;

static uint8_t t_overflow;

/**
 * Wrap trace
 */
static void tc_wrap(void) {
    t_overflow++;
}

/**
 * Initialize trace
 */
void hf_trace_init() {
    tc_enable(&TCC0);
    tc_set_overflow_interrupt_callback(&TCC0, tc_wrap);
    tc_set_wgm(&TCC0, TC_WG_NORMAL);
    tc_write_period(&TCC0, 65535);
    tc_set_overflow_interrupt_level(&TCC0, TC_INT_LVL_LO);
    tc_write_clock_source(&TCC0, TC_CLKSEL_DIV8_gc);
}

/**
 * Record a trace event
 * @param event
 * @param data
 */
void hf_trace(uint8_t event, uint8_t data) {
    irqflags_t irq;

    irq = cpu_irq_save();
    t_head->event = event;
    t_head->data = data;
    /* in 250nsec increments */
    t_head->time = tc_read_count(&TCC0);

    if (++t_head >= &trace_records[TRACE_RECORDS])
    t_head = trace_records;
    /* marker */
    t_head->event = 255;
    cpu_irq_restore(irq);
}

#endif /* INCLUDE_TRACING */

#if 0

/**
 * Initialize tracing on A3BU
 */
void a3bu_trace_init() {
    ioport_configure_port_pin(&PORTB, PIN0_bm, IOPORT_DIR_OUTPUT);
    ioport_configure_port_pin(&PORTB, PIN1_bm, IOPORT_DIR_OUTPUT);
    ioport_configure_port_pin(&PORTB, PIN2_bm, IOPORT_DIR_OUTPUT);
    ioport_configure_port_pin(&PORTB, PIN3_bm, IOPORT_DIR_OUTPUT);
    ioport_set_pin_level(SPIN_0, 0);
}

#endif /* 0 */
