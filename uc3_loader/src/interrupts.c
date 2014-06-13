/* interrupts.c */

/*
    Copyright (c) 2013 HashFast Technologies LLC
*/

#include <stdio.h>
#include <stdint.h>
#include <avr32/io.h>

#include "preprocessor.h"

#include "config.h"
#include "interrupts.h"


/* imported from exception.S */
extern void _evba;
extern void _int0, _int1, _int2, _int3;


#define AVR32_INTC_MAX_NUM_IRQS_PER_GRP       32

#ifndef MAX
#define MAX(a,b)  ((a) > (b) ? (a) : (b))
#endif

/*
   I rather strongly dislike this sort of use of the preprocessor.  But
   this is what Atmel did in their interrupt handler and I just don't
   have the time at the moment to do this cleanly.
*/
#define DECLARE_GROUP_TABLE(GROUP, unused) \
    static volatile interruptHandler \
                    table##GROUP[MAX(AVR32_INTC_NUM_IRQS_PER_GRP##GROUP, 1)];
MREPEAT(AVR32_INTC_NUM_INT_GRPS, DECLARE_GROUP_TABLE, ~);

#define INSERT_TABLE(GROUP, unused) \
    {AVR32_INTC_NUM_IRQS_PER_GRP##GROUP, table##GROUP},

static const struct {
    unsigned int numIrqs;
    volatile interruptHandler *handlers;
} intHandlerTable[AVR32_INTC_NUM_INT_GRPS] = {
    MREPEAT(AVR32_INTC_NUM_INT_GRPS, INSERT_TABLE, ~)
};


__attribute__((__interrupt__)) static void unhandledInterrupt(void) {

}

interruptHandler _get_interrupt_handler(uint32_t intLevel) {
    uint32_t group;
    uint32_t req;

    group = AVR32_INTC.icr[AVR32_INTC_INT3 - intLevel];
    req = AVR32_INTC.irr[group];

    return (req ?
            intHandlerTable[group].handlers[32 - __builtin_clz(req) - 1] :
            NULL);
}

void interruptsInit(void) {
    uint32_t group;
    uint32_t req;

    __builtin_mtsr(AVR32_EVBA, (int32_t) &_evba);

    for (group = 0; group < AVR32_INTC_NUM_INT_GRPS; group++) {
        for (req = 0; req < intHandlerTable[group].numIrqs; req++)
            intHandlerTable[group].handlers[req] = &unhandledInterrupt;
        AVR32_INTC.ipr[group] = (AVR32_INTC_INT0 <<
                                 AVR32_INTC_IPR_INTLEVEL_OFFSET) |
                                ((int) &_int0 - (int) &_evba);
    }
}

void interruptsRegister(interruptHandler handler, uint32_t irq,
                        uint32_t level) {
    uint32_t group;

    group = irq / AVR32_INTC_MAX_NUM_IRQS_PER_GRP;

    intHandlerTable[group].handlers[irq % AVR32_INTC_MAX_NUM_IRQS_PER_GRP] =
        handler;

    switch (level) {
    case AVR32_INTC_INT0:
        AVR32_INTC.ipr[group] = (AVR32_INTC_INT0 <<
                                 AVR32_INTC_IPR_INTLEVEL_OFFSET) |
                                ((int) &_int0 - (int) &_evba);
        break;
    case AVR32_INTC_INT1:
        AVR32_INTC.ipr[group] = (AVR32_INTC_INT1 <<
                                 AVR32_INTC_IPR_INTLEVEL_OFFSET) |
                                ((int) &_int1 - (int) &_evba);
        break;
    case AVR32_INTC_INT2:
        AVR32_INTC.ipr[group] = (AVR32_INTC_INT2 <<
                                 AVR32_INTC_IPR_INTLEVEL_OFFSET) |
                                ((int) &_int2 - (int) &_evba);
        break;
    case AVR32_INTC_INT3:
        AVR32_INTC.ipr[group] = (AVR32_INTC_INT3 <<
                                 AVR32_INTC_IPR_INTLEVEL_OFFSET) |
                                ((int) &_int3 - (int) &_evba);
        break;
    }
}

uint32_t interruptsSaveAndDisable(void) {
    uint32_t flags;

    flags = __builtin_mfsr(AVR32_SR);
    interruptsDisable();

    return flags;
}

void interruptsRestore(uint32_t flags) {

    barrier();

    if (!(flags & AVR32_SR_GM_MASK))
        interruptsEnable();

    barrier();
}

