/* adc.c */

/*
    Copyright (c) 2014 HashFast Technologies LLC
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


static const struct {
    int channel;
    int pin;
    int function;
} channels[] = {
    {0, INDUCTOR_4_TEMP, AVR32_ADC_AD_0_FUNCTION},
    {1, INDUCTOR_3_TEMP, AVR32_ADC_AD_1_FUNCTION},
    {2, INDUCTOR_2_TEMP, AVR32_ADC_AD_2_FUNCTION},
    {3, INDUCTOR_1_TEMP, AVR32_ADC_AD_3_FUNCTION},
    {7, V_MID, AVR32_ADC_AD_7_FUNCTION}
};

static uint32_t srDoneMask;

static struct {
    int32_t x[2];
    int32_t y[2];
} filters[sizeof(channels) / sizeof(channels[0])];


void adcInit(void) {
    int i;

    for (i = 0; i < sizeof(channels) / sizeof(channels[0]); i++)
        gpio_enable_module_pin(channels[i].pin, channels[i].function);

    AVR32_ADC.cr = AVR32_ADC_CR_SWRST_MASK;

    AVR32_ADC.mr = ((3 - 3) << AVR32_ADC_MR_SHTIM_OFFSET) |
                   ((13 - 1) << AVR32_ADC_MR_STARTUP_OFFSET) |
                   ((6 - 1) << AVR32_ADC_MR_PRESCAL_OFFSET);

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

void adcTask(void) {
    static enum {idleAS, convertingAS} state = idleAS;
    static uint16_t pollTime = 0;
    static int samples = 0;
    int i;
    uint32_t d;

    switch (state) {
    case idleAS:
        if (elapsed_since(pollTime) >= ADC_SAMPLE_PERIOD &&
            boardid != iraBID) {
            AVR32_ADC.cr = AVR32_ADC_CR_START_MASK;
            pollTime = msec_ticker;
            state = convertingAS;
        }
        break;
    case convertingAS:
        if ((AVR32_ADC.sr & srDoneMask) == srDoneMask) {
            for (i = 0; i < sizeof(channels) / sizeof(channels[0]); i++) {
                d = (((&AVR32_ADC.cdr0)[channels[i].channel]) &
                     AVR32_ADC_CDR0_DATA_MASK) >>
                    AVR32_ADC_CDR0_DATA_OFFSET;
                filters[i].x[0] = filters[i].x[1];
                filters[i].x[1] = d <<
                                  (16 - (TEMPERATURE_OVERSAMPLING_LOG2 + 1));
                filters[i].y[0] = filters[i].y[1];
                filters[i].y[1] = (filters[i].x[0] + filters[i].x[1]) +
                                  filters[i].y[0] -
                                  (filters[i].y[0] >>
                                   TEMPERATURE_OVERSAMPLING_LOG2);
                if (filters[i].y[1] < 0)
                    filters[i].y[1] = 0;
                else if (filters[i].y[1] > (1023 << 16))
                    filters[i].y[1] = 1023 << 16;
            }

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

