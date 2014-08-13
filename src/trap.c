/** @file trap.c
 * @brief Trap program for state debugging
 *
 * It's a trap!
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

#include <stdint.h>
#include <avr32/io.h>

#include "main.h"
#include "cli.h"
#include "tty.h"
#include "trap.h"

/**
 * Trap the program
 * @param sp
 * @param addr
 * @param cause
 */
void trap(uint32_t *sp, uint32_t addr, uint32_t cause) {
    /*
     * sp[0] = sr, sp[1] = pc
     * r0 is in sp[-1], r1 in sp[-2], ..., r12 in sp[-13], lr in sp[-14]
     * addr is address of offending instruction
     * cause is contents of AVR32_ECR
     */

#ifdef FEATURE_DEBUG_CLI

    uint32_t start;
    int i;

    ttyInit();
    start = __builtin_mfsr(AVR32_COUNT);

    /*
     * Wait for more than one character time since we know txd has been high to
     * be sure receiving uart gets in sync right away. At 115200 baud a
     * character is on the order of 100uS and processor cycles are at 60MHz.
     */
    while ((__builtin_mfsr(AVR32_COUNT) - start) < 10000);

    cliInit(&ttyRead, &ttyWrite);

    cliWriteString("\nTRAP ");
    cliWriteGawbleHex(cause);
    cliWriteString(" addr: ");
    cliWriteGawbleHex(addr);
    cliWriteString(" pc: ");
    cliWriteGawbleHex(sp[1]);
    cliWriteChar('\n');
    while (!cliFlush());

    cliWriteString(" sr: ");
    cliWriteGawbleHex(sp[0]);
    cliWriteString(" sp: ");
    cliWriteGawbleHex((uint32_t) sp);
    cliWriteChar('\n');
    while (!cliFlush());

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
        while (!cliFlush());
    }
    cliWriteString(" lr:  ");
    cliWriteGawbleHex(sp[-14]);
    cliWriteChar('\n');
    while (!cliFlush());

    while (1) {
        cliTask();
    }

#else /* FEATURE_DEBUG_CLI */
    while (1)
    ;
#endif /* FEATURE_DEBUG_CLI */
    /* not reached */
}
