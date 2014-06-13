/* mpu.c */

/*
    Copyright (c) 2014 HashFast Technologies LLC
*/

#include <stdint.h>
#include <avr32/io.h>

#include "main.h"
#include "cli.h"
#include "mpu.h"


static struct {
    uint32_t cause;
    uint32_t addr;
    uint32_t sp;
    uint32_t sr;
    uint32_t pc;
    uint32_t regs[14];
    unsigned int count;
} exceptionFrame;



/* BISON this is a hack to meet an immediate debug need.  should select
   region(s) in sram to protect intelligently rather than the hard coded
   stuff here that will break when data usage changes. */
void mpuSetup(int protectHeap) {

#if !defined(__AVR32_UC3B0512__)
#error /* at the very least the sizes of the below ranges need to be fixed */
#endif
    __builtin_mtsr(AVR32_MPUCR, 0);

    exceptionFrame.count = 0;

    /* sram */
    __builtin_mtsr(AVR32_MPUAR0,
                   AVR32_SRAM_ADDRESS |
                   (16 << AVR32_MPUAR0_SIZE_OFFSET) |
                   AVR32_MPUAR0_V_MASK);
    __builtin_mtsr(AVR32_MPUPSR0, protectHeap ? 0x0000f7e0 : 0x0000c000);

    /* flash */
    __builtin_mtsr(AVR32_MPUAR1,
                   AVR32_FLASH_ADDRESS |
                   (18 << AVR32_MPUAR1_SIZE_OFFSET) |
                   AVR32_MPUAR1_V_MASK);
    __builtin_mtsr(AVR32_MPUPSR1, 0);

    /* flash user page */
    __builtin_mtsr(AVR32_MPUAR2,
                   AVR32_FLASHC_USER_PAGE_ADDRESS |
                   (11 << AVR32_MPUAR2_SIZE_OFFSET) |
                   AVR32_MPUAR2_V_MASK);
    __builtin_mtsr(AVR32_MPUPSR2, 0);

    /* usbdata */
    __builtin_mtsr(AVR32_MPUAR3,
                   AVR32_USBB_SLAVE_ADDRESS |
                   (15 << AVR32_MPUAR3_SIZE_OFFSET) |
                   AVR32_MPUAR3_V_MASK);
    __builtin_mtsr(AVR32_MPUPSR3, 0);

    /* peripherals */
    __builtin_mtsr(AVR32_MPUAR4,
                   0xfffe0000 |
                   (16 << AVR32_MPUAR4_SIZE_OFFSET) |
                   AVR32_MPUAR4_V_MASK);
    __builtin_mtsr(AVR32_MPUPSR4, 0);

    /* unused */
    __builtin_mtsr(AVR32_MPUAR5, 0);
    __builtin_mtsr(AVR32_MPUAR6, 0);
    __builtin_mtsr(AVR32_MPUAR7, 0);

    __builtin_mtsr(AVR32_MPUAPRA, 0xaaa66777);
    __builtin_mtsr(AVR32_MPUAPRB, 0xaaa44555);

    __builtin_mtsr(AVR32_MPUCR, AVR32_MPUCR_E_MASK);
}

#ifdef FEATURE_DEBUG_CLI
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
        } else
            done = 1;
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

void mpuException(uint32_t *sp, uint32_t addr, uint32_t cause) {
    /* sp[0] = sr, sp[1] = pc */
    /* r0-r12 and lr are all in sp[-something] */
    /* addr is address of offending instruction */
    /* cause is contents of AVR32_ECR */

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

