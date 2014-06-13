
#include "main.h"

#ifdef INCLUDE_TRACING

struct tracerecord_t trace_records[TRACE_RECORDS];
struct tracerecord_t *t_head = trace_records;

static uint8_t t_overflow;

static void tc_wrap(void)
    {
    t_overflow++;
    }

void hf_trace_init()
    {
    tc_enable(&TCC0);
    tc_set_overflow_interrupt_callback(&TCC0, tc_wrap);
    tc_set_wgm(&TCC0, TC_WG_NORMAL);
    tc_write_period(&TCC0, 65535);
    tc_set_overflow_interrupt_level(&TCC0, TC_INT_LVL_LO);
    tc_write_clock_source(&TCC0, TC_CLKSEL_DIV8_gc);
    }

void hf_trace(uint8_t event, uint8_t data)
    {
    irqflags_t irq;

    irq = cpu_irq_save();
    t_head->event = event;
    t_head->data = data;
    t_head->time = tc_read_count(&TCC0);            // In 250nsec increments

    if (++t_head >= &trace_records[TRACE_RECORDS])
        t_head = trace_records;
    t_head->event = 255;                            // Marker
    cpu_irq_restore(irq);
    }

#endif


#if 0
void a3bu_trace_init()
    {
    ioport_configure_port_pin(&PORTB, PIN0_bm, IOPORT_DIR_OUTPUT);
    ioport_configure_port_pin(&PORTB, PIN1_bm, IOPORT_DIR_OUTPUT);
    ioport_configure_port_pin(&PORTB, PIN2_bm, IOPORT_DIR_OUTPUT);
    ioport_configure_port_pin(&PORTB, PIN3_bm, IOPORT_DIR_OUTPUT);
    ioport_set_pin_level(SPIN_0, 0); 
    }

#endif
