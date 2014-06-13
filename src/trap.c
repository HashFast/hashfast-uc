/* trap.c */

/*
    Copyright (c) 2014 HashFast Technologies LLC
*/

#include <stdint.h>
#include <avr32/io.h>

#include "main.h"
#include "cli.h"
#include "tty.h"
#include "trap.h"


void trap(uint32_t *sp, uint32_t addr, uint32_t cause) {
    /* sp[0] = sr, sp[1] = pc */
    /* r0 is in sp[-1], r1 in sp[-2], ..., r12 in sp[-13], lr in sp[-14] */
    /* addr is address of offending instruction */
    /* cause is contents of AVR32_ECR */
#ifdef FEATURE_DEBUG_CLI
    uint32_t start;
    int i;

    ttyInit();
    start = __builtin_mfsr(AVR32_COUNT);
    /* wait for more than one character time since we know txd has been high
       to be sure receiving uart gets in sync right away.  at 115200 baud a
       character is on the order of 100uS and processor cycles are at 60MHz. */
    while ((__builtin_mfsr(AVR32_COUNT) - start) < 10000)
        ;

    cliInit(&ttyRead, &ttyWrite);

    cliWriteString("\nTRAP ");
    cliWriteGawbleHex(cause);
    cliWriteString(" addr: ");
    cliWriteGawbleHex(addr);
    cliWriteString(" pc: ");
    cliWriteGawbleHex(sp[1]);
    cliWriteChar('\n');
    while (!cliFlush())
        ;

    cliWriteString(" sr: ");
    cliWriteGawbleHex(sp[0]);
    cliWriteString(" sp: ");
    cliWriteGawbleHex((uint32_t) sp);
    cliWriteChar('\n');
    while (!cliFlush())
        ;

    for (i = 0; i <= 12; i++) {
        cliWriteString(" r");
        if (i >= 10)
            cliWriteChar('0' + i / 10);
        cliWriteChar('0' + i % 10);
        cliWriteString(": ");
        if (i < 10)
            cliWriteChar(' ');
        cliWriteGawbleHex(sp[-1 - i]);
        cliWriteChar('\n');
        while (!cliFlush())
            ;
    }
    cliWriteString(" lr:  ");
    cliWriteGawbleHex(sp[-14]);
    cliWriteChar('\n');
    while (!cliFlush())
        ;

    while (1)
        cliTask();
#else /* FEATURE_DEBUG_CLI */
    while (1)
        ;
#endif /* FEATURE_DEBUG_CLI */
    /* not reached */
}

