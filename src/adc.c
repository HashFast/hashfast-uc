/** @file adc.c
 * @brief ADC initialization and periodic task
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

#include <intc.h>
#include <sysclk.h>
#include <gpio.h>

#include "hf_protocol.h"
#include "hf_factory.h"
#include "usb_uart.h"
#include "hf_nvram.h"
#include "boardid.h"
#include "module_handler.h"
#include "gwq_handler.h"
#include "uc3b_peripherals.h"
#include "adc.h"

#define ADC_SAMPLE_PERIOD                 10

#define TEMPERATURE_OVERSAMPLING_LOG2      6
#define TEMPERATURE_SAMPLES_PER_OUTPUT    25

#define VOLTAGE_OVERSAMPLING_LOG2          3
#define VOLTAGE_SAMPLES_PER_OUTPUT         4

/**
 * channel struct
 */
static const struct {
    int channel;
    int pin;
    int function;
} channels[] = {
    {0, INDUCTOR_4_TEMP, AVR32_ADC_AD_0_FUNCTION},
    {1, INDUCTOR_3_TEMP, AVR32_ADC_AD_1_FUNCTION},
    {2, INDUCTOR_2_TEMP, AVR32_ADC_AD_2_FUNCTION},
    {3, INDUCTOR_1_TEMP, AVR32_ADC_AD_3_FUNCTION},
    {7, V_MID, AVR32_ADC_AD_7_FUNCTION }
};

/**
 * channel setup done mask
 */
static uint32_t srDoneMask;

/**
 * filter struct
 */
static struct {
    int32_t x[2];
    int32_t y[2];
} filters[sizeof(channels) / sizeof(channels[0])];

/**
 * ADC initialization
 */
void adcInit(void) {
    int i;

    for (i = 0; i < sizeof(channels) / sizeof(channels[0]); i++)
        gpio_enable_module_pin(channels[i].pin, channels[i].function);

    AVR32_ADC.cr = AVR32_ADC_CR_SWRST_MASK;

    AVR32_ADC.mr = ((3 - 3) << AVR32_ADC_MR_SHTIM_OFFSET) | ((13 - 1) << AVR32_ADC_MR_STARTUP_OFFSET) | ((6 - 1) << AVR32_ADC_MR_PRESCAL_OFFSET);

    srDoneMask = 0;
    for (i = 0; i < sizeof(channels) / sizeof(channels[0]); i++) {
        filters[i].x[0] = 512 << (16 - (TEMPERATURE_OVERSAMPLING_LOG2 + 1));
        filters[i].x[1] = 512 << (16 - (TEMPERATURE_OVERSAMPLING_LOG2 + 1));
        filters[i].y[0] = 512 << 16;
        filters[i].y[1] = 512 << 16;
        AVR32_ADC.cher = AVR32_ADC_CHER_CH0_MASK << channels[i].channel;
        srDoneMask |= AVR32_ADC_SR_EOC0_MASK << channels[i].channel;
    }
}

/**
 * ADC periodic task
 */
void adcTask(void) {
    static enum {idleAS, convertingAS} state = idleAS;
    static uint16_t pollTime = 0;
    static int samples = 0;
    int i;
    uint32_t d;

    switch (state) {
    case idleAS:
        if (elapsed_since(pollTime) >= ADC_SAMPLE_PERIOD && boardid != iraBID) {
            AVR32_ADC.cr = AVR32_ADC_CR_START_MASK;
            pollTime = msec_ticker;
            state = convertingAS;
        }
        break;
    case convertingAS:
        if ((AVR32_ADC.sr & srDoneMask) == srDoneMask) {
            for (i = 0; i < sizeof(channels) / sizeof(channels[0]); i++) {
                d = (((&AVR32_ADC.cdr0)[channels[i].channel]) & AVR32_ADC_CDR0_DATA_MASK) >> AVR32_ADC_CDR0_DATA_OFFSET;
                filters[i].x[0] = filters[i].x[1];
                filters[i].x[1] = d << (16 - (TEMPERATURE_OVERSAMPLING_LOG2 + 1));
                filters[i].y[0] = filters[i].y[1];
                filters[i].y[1] = (filters[i].x[0] + filters[i].x[1]) + filters[i].y[0] - (filters[i].y[0] >> TEMPERATURE_OVERSAMPLING_LOG2);
                if (filters[i].y[1] < 0)
                    filters[i].y[1] = 0;
                else if (filters[i].y[1] > (1023 << 16))
                    filters[i].y[1] = 1023 << 16;
            }
            /* update board temperature */
            if (++samples >= TEMPERATURE_SAMPLES_PER_OUTPUT) {
                for (i = 0; i < 4; i++)
                    gwq_update_board_temperature(i, filters[i].y[1] >> 16);
                samples = 0;
            }
            state = idleAS;
        }
        break;
    }
}

