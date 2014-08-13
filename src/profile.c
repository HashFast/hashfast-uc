/** @file profile.c
 * @brief uC profiler
 *
 * This measures run time between matched pairs of profileEnter/profileExit
 * calls. Execution times excluding the time spent in instrumented ISRs are
 * also available. This profiling code does of course incur some overhead, some
 * of which is attributed to the area being measured and some of which is not.
 * ISRs also have some entry/exit overhead that will be attributed to non-ISR
 * time. Both of these overheads can increase the measured times; for functions
 * long enough to be worth measuring they will not usually be significant,
 * unless interrupts are occurring at a very high rate.
 *
 * The only method provided at this time to retrieve profile data is through the
 * CLI "profile" command. In all cases that command shows all collected profile
 * data. If it is given a single non-zero parameter then after showing current
 * data it then resets it. Since the data prior to the start of mining isn't
 * usually relevant, the normal use would be to wait until mining has started,
 * enter "profile 1", then later another "profile" to examine results.
 *
 * The time units are cycles of the processor clock (60MHz on the rev 1 modules).
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
#include "profile.h"

#ifdef FEATURE_PROFILE

/**
 * Profile channels
 */
static struct {
    bool running;
    uint8_t flags;
    uint32_t start;
    uint32_t isrStart;
    uint32_t errors;
    uint32_t elapsedMin;
    uint32_t elapsedMax;
    uint32_t elapsedMaxIncludingISRs;
    uint64_t count;
    uint64_t elapsedTotal;
    uint64_t elapsedTotalIncludingISRs;
} channels[PROFILE_CHANNELS];

static uint32_t totalISR;

static const struct {
    unsigned int channel;
    uint8_t flags;
    const char *name;
} fixedChannels[] = {
    {PROFILE_CHANNEL_TC_ISR,        PROFILE_FLAGS_ISR, "tc isr"},
    {PROFILE_CHANNEL_GPIO_ISR,      PROFILE_FLAGS_ISR, "gpio isr"},
    {PROFILE_CHANNEL_SPIDMA_ISR,    PROFILE_FLAGS_ISR, "spi dma isr"},
    {PROFILE_CHANNEL_TWI_ISR,       PROFILE_FLAGS_ISR, "twi isr"},
    {PROFILE_CHANNEL_UART_ISR,      PROFILE_FLAGS_ISR, "uart isr"},
    {PROFILE_CHANNEL_UARTTXDMA_ISR, PROFILE_FLAGS_ISR, "uart tx dma isr"},
    {PROFILE_CHANNEL_UARTRXDMA_ISR, PROFILE_FLAGS_ISR, "uart rx dma isr"},
    {PROFILE_CHANNEL_USB_ISR,       PROFILE_FLAGS_ISR, "usb isr"},
    {PROFILE_CHANNEL_MAINLOOP,      0, "mainloop"}
};

/**
 * Initialize profiling
 */
void profileInit(void) {
    unsigned int i;

    memset(channels, 0, sizeof(channels));
    for (i = 0; i < sizeof(fixedChannels) / sizeof(fixedChannels[0]); i++)
        profileConfigure(fixedChannels[i].channel, fixedChannels[i].flags);
profileReset();
}

/**
 * Reset profiles
 */
void profileReset(void) {
    unsigned int i;
    irqflags_t irq;

    irq = cpu_irq_save();
    for (i = 0; i < sizeof(channels) / sizeof(channels[0]); i++) {
        channels[i].count = 0;
        channels[i].elapsedMin = ~(uint32_t) 0;
        channels[i].elapsedMax = 0;
        channels[i].elapsedMaxIncludingISRs = 0;
        channels[i].elapsedTotal = 0;
        channels[i].elapsedTotalIncludingISRs = 0;
    }
    cpu_irq_restore(irq);
}

/**
 * Configure profile channel
 * @param channel
 * @param flags
 */
void profileConfigure(unsigned int channel, uint8_t flags) {

    if (channel < PROFILE_CHANNELS)
        channels[channel].flags = flags;
}

/**
 * Mark entry into profile
 * @param channel
 */
void profileEnter(unsigned int channel) {
    irqflags_t irq;

    if (channel < PROFILE_CHANNELS) {
        irq = cpu_irq_save();
        if (!channels[channel].running) {
            channels[channel].start = __builtin_mfsr(AVR32_COUNT);
            channels[channel].isrStart = totalISR;
            channels[channel].running = true;
        } else
            channels[channel].errors++;
        cpu_irq_restore(irq);
    }
}

/**
 * Mark exit from profile
 * @param channel
 */
void profileExit(unsigned int channel) {
    irqflags_t irq;
    uint32_t elapsed;

    if (channel < PROFILE_CHANNELS) {
        irq = cpu_irq_save();
        if (channels[channel].running) {
            elapsed = __builtin_mfsr(AVR32_COUNT) - channels[channel].start;
            if (elapsed > channels[channel].elapsedMaxIncludingISRs)
            channels[channel].elapsedMaxIncludingISRs = elapsed;
            channels[channel].elapsedTotalIncludingISRs += (uint64_t) elapsed;
            elapsed -= totalISR - channels[channel].isrStart;
            if (channels[channel].flags & PROFILE_FLAGS_ISR)
            totalISR += elapsed;
            if (elapsed < channels[channel].elapsedMin)
            channels[channel].elapsedMin = elapsed;
            if (elapsed > channels[channel].elapsedMax)
            channels[channel].elapsedMax = elapsed;
            channels[channel].elapsedTotal += (uint64_t) elapsed;
            channels[channel].count++;
            channels[channel].running = false;
        } else
        channels[channel].errors++;
        cpu_irq_restore(irq);
    }
}

#ifdef FEATURE_DEBUG_CLI

/**
 * Dump profile
 * @param first
 * @param paramCount
 * @param params
 * @return success
 */
int profileCLI(int first, int parmCount, uint32_t *parms) {
    static unsigned int channel;
    static unsigned int chunk;
    unsigned int i;
    int done;

    if (first) {
        channel = 0;
        chunk = 0;
    }

    done = 0;
    if (channel < PROFILE_CHANNELS) {
        if (channels[channel].count) {
            switch (chunk++) {
            case 0:
                cliWriteByteHex(channel);
                for (i = 0;
                        i < sizeof(fixedChannels) / sizeof(fixedChannels[0]);
                        i++)
                    if (fixedChannels[i].channel == channel) {
                        cliWriteChar(' ');
                        cliWriteString(fixedChannels[i].name);
                        break;
                    }
                cliWriteString(":\n");
                break;
            case 1:
                cliWriteString("  calls ");
                cliWriteGawbleHex(channels[channel].count);
                cliWriteString(" errs ");
                cliWriteGawbleHex(channels[channel].errors);
                cliWriteString(" min ");
                cliWriteGawbleHex(channels[channel].elapsedMin);
                cliWriteString(" max ");
                cliWriteGawbleHex(channels[channel].elapsedMax);
                cliWriteString(" average ");
                cliWriteGawbleHex(channels[channel].elapsedTotal / channels[channel].count);
                cliWriteChar('\n');
                break;
            case 2:
                cliWriteString("  max with isrs ");
                cliWriteGawbleHex(channels[channel].elapsedMaxIncludingISRs);
                cliWriteString(" average with isrs ");
                cliWriteGawbleHex(channels[channel].elapsedTotalIncludingISRs / channels[channel].count);
                cliWriteChar('\n');
                channel++;
                chunk = 0;
                break;
            }
        } else
            channel++;
    } else {
        if (parmCount && parms[0])
            profileReset();
        done = 1;
    }

    return done;
}
#endif /* FEATURE_DEBUG_CLI */

#endif /* FEATURE_PROFILE */

