/** @file mpu.c
 * @brief Memory protection configuration
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
#include "mpu.h"

/**
 * MPU exception frame.
 */
static struct {
    uint32_t cause;     //!< Cause
    uint32_t addr;      //!< Address
    uint32_t sp;        //!< Stack pointer
    uint32_t sr;        //!< Service routine
    uint32_t pc;        //!< Program counter
    uint32_t regs[14];  //!< General registers
    unsigned int count;
} exceptionFrame;

/**
 * BISON this is a hack to meet an immediate debug need. Should select region(s)
 * in SRAM to protect intelligently rather than the hard coded stuff here that
 * will break when data usage changes.
 * @param protectHeap
 */
void mpuSetup(int protectHeap) {

#if !(defined(__AVR32_UC3B0512__) || defined(__AVR32_UC3B0256__) || defined(__AVR32_UC3B0128__))
#error /* at the very least the sizes of the below ranges need to be fixed */
#endif /* !defined(__AVR32_UC3B0512__) */

    __builtin_mtsr(AVR32_MPUCR, 0);

    exceptionFrame.count = 0;

    /* SRAM */
    __builtin_mtsr(AVR32_MPUAR0, AVR32_SRAM_ADDRESS             | (16 << AVR32_MPUAR0_SIZE_OFFSET) | AVR32_MPUAR0_V_MASK);
    __builtin_mtsr(AVR32_MPUPSR0, protectHeap ? 0x0000f7e0 : 0x0000c000);

    /* Flash */
    __builtin_mtsr(AVR32_MPUAR1, AVR32_FLASH_ADDRESS            | (18 << AVR32_MPUAR1_SIZE_OFFSET) | AVR32_MPUAR1_V_MASK);
    __builtin_mtsr(AVR32_MPUPSR1, 0);

    /* Flash User Page */
    __builtin_mtsr(AVR32_MPUAR2, AVR32_FLASHC_USER_PAGE_ADDRESS | (11 << AVR32_MPUAR2_SIZE_OFFSET) | AVR32_MPUAR2_V_MASK);
    __builtin_mtsr(AVR32_MPUPSR2, 0);

    /* usbdata */
    __builtin_mtsr(AVR32_MPUAR3, AVR32_USBB_SLAVE_ADDRESS       | (15 << AVR32_MPUAR3_SIZE_OFFSET) |AVR32_MPUAR3_V_MASK);
    __builtin_mtsr(AVR32_MPUPSR3, 0);

    /* Peripherals */
    __builtin_mtsr(AVR32_MPUAR4, 0xfffe0000                     | (16 << AVR32_MPUAR4_SIZE_OFFSET) | AVR32_MPUAR4_V_MASK);
    __builtin_mtsr(AVR32_MPUPSR4, 0);

    /* Unused */
    __builtin_mtsr(AVR32_MPUAR5, 0);
    __builtin_mtsr(AVR32_MPUAR6, 0);
    __builtin_mtsr(AVR32_MPUAR7, 0);

    __builtin_mtsr(AVR32_MPUAPRA, 0xaaa66777);
    __builtin_mtsr(AVR32_MPUAPRB, 0xaaa44555);

    __builtin_mtsr(AVR32_MPUCR, AVR32_MPUCR_E_MASK);
}

#ifdef FEATURE_DEBUG_CLI

/**
 * MPU dump
 * @param sp
 * @param addr
 * @param cause
 */
int mpuDump(int first, int parmCount, uint32_t *parms) {
    static int chunk;
    int done;

    if (first)
        chunk = 0;

    done = 0;
    switch (chunk++) {
    case 0:
        cliWriteString("exception count ");
        cliWriteGawbleHex(exceptionFrame.count);
        cliWriteChar('\n');
        if (exceptionFrame.count) {
            cliWriteString("cause ");
            cliWriteGawbleHex(exceptionFrame.cause);
            cliWriteString(" addr ");
            cliWriteGawbleHex(exceptionFrame.addr);
            cliWriteString(" sp ");
            cliWriteGawbleHex(exceptionFrame.sp);
            cliWriteChar('\n');
        } else {
            done = 1;
        }
        break;
    case 1:
        cliWriteString("pc ");
        cliWriteGawbleHex(exceptionFrame.pc);
        cliWriteString(" sr ");
        cliWriteGawbleHex(exceptionFrame.sr);
        cliWriteChar('\n');
        break;
    default:
        done = 1;
        break;
    }

    return done;
}
#endif /* FEATURE_DEBUG_CLI */

/**
 * MPU exception
 * @param sp
 * @param addr
 * @param cause
 */
void mpuException(uint32_t *sp, uint32_t addr, uint32_t cause) {
    /*
     * sp[0] = sr, sp[1] = pc
     * r0-r12 and lr are all in sp[-something]
     * addr is address of offending instruction
     * cause is contents of AVR32_ECR
     */
    switch (cause) {
    case AVR32_ECR_ECR_PROTECTION_R:
    case AVR32_ECR_ECR_PROTECTION_W:
    case AVR32_ECR_ECR_PROTECTION_X:
    case AVR32_ECR_ECR_TLB_MISS_R:
    case AVR32_ECR_ECR_TLB_MISS_W:
    case AVR32_ECR_ECR_TLB_MISS_X:
    case AVR32_ECR_ECR_TLB_MULTIPLE:
    default:
        /* disable mpu */
        __builtin_mtsr(AVR32_MPUCR, 0);
        break;
    }

    if (exceptionFrame.count == 0) {
        exceptionFrame.cause = cause;
        exceptionFrame.addr = addr;
        exceptionFrame.sp = (uint32_t) sp;
        exceptionFrame.sr = sp[0];
        exceptionFrame.pc = sp[1];
    }
    exceptionFrame.count++;
}
