/** @file pll_calc.c
 * @brief Fills in an hf_pll_config structure that can the be sent off to the ASIC.
 *
 * Given a target hash clock rate and reference frequency, compute the necessary
 * PLL parameters.
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

#define USE_LOWEST_JITTER_RANGE

#ifdef STANDALONE

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "hf_protocol.h"

/**
 * PLL_ENTRY structure
 */
typedef struct {
    uint32_t freq;
    uint8_t F, R, Q, range;
} pll_entry_t;

#define DIMENSION(x) (sizeof(x)/sizeof(x[0]))

uint16_t pll_calc(struct hf_pll_config *, uint32_t, uint32_t);

#else /* STANDALONE */

#include "main.h"
//TODO remove this
#define printf(...)

#endif /* STANDALONE */

static float required_accuracy = 1.2;           //!< Look for frequency within 1.2% of request
const static uint32_t lowest_divref = 25000000; //!< Lowest acceptable divided reference in Hz

uint32_t last_pll_parameters = 0;

int8_t hcm_force_pll_r = 0;
int8_t hcm_force_pll_range = -1;

/**
 * Calculate PLL values
 * @param pll
 * @param target
 * @param ref
 * @return
 */
uint16_t pll_calc(struct hf_pll_config *pll, uint32_t target, uint32_t ref) {
    uint64_t vco;
    uint32_t vco_fb;
    uint32_t r, f;
    uint32_t divided_ref, divided_fb;
    uint32_t rounded_target;
    uint8_t q;
    uint8_t i;
    uint8_t doing_exact;
    uint8_t solved, approx;
    uint8_t exact, found_exact;
    uint8_t divq;
    uint8_t min_r, max_r;
    float actual_frequency;
#ifdef STANDALONE
    char *str;
#endif /* STANDALONE */

    if (ref <= 40000000) {
        /* restrict to direct reference */
        min_r = 1;
        max_r = 1;
        /* Not much granularity here, hit the nearest exact solution. */
        rounded_target = ((target + (ref / 4)) / (ref / 2)) * (ref / 2);
        /* slight slacken accuracy requirement */
        required_accuracy = 2.5;
    } else if (ref <= 62500000) {
        /* max divide by 2 */
        min_r = 1;
        max_r = 2;
        /* Not much granularity here, hit the nearest exact solution. */
        rounded_target = ((target + (ref / 8)) / (ref / 4)) * (ref / 4);
        required_accuracy = 1.5;
    } else {
        /* Wide open case, 125 Mhz will land here. */
        min_r = 1;
        max_r = 32;
        rounded_target = target;
    }

    if (hcm_force_pll_r) {
        min_r = hcm_force_pll_r;
        max_r = hcm_force_pll_r;
    }
    printf("Internal target: %d Hz\n", rounded_target);
    solved = 0;
    approx = 0;
    /*
     * Initially, pick the lowest possible Q diviser, for the lowest possible
     * VCO frequency that is within range.
     */
    for (q = 2, divq = 1; q <= 64; q = q << 1, divq++) {
        vco = (uint64_t) rounded_target * q;
        if (vco >= (uint64_t) 2000000000 && vco <= (uint64_t) 4000000000)
            break;
    }
    if (q > 64) {
        printf("Out of range, can't be done\n");
        return (0);
    }

    /*
     * Now we have a trial VCO frequency.
     * Figure out how to make it.
     */
    vco_fb = (uint32_t) (vco / 2);

    /* Try and find exact solution first, otherwise as close as we can get */
    for (i = 0, doing_exact = 1; i < 2; i++, doing_exact = 0) {
        for (r = min_r; r <= max_r; r++) {
            divided_ref = ref / r;
            if (divided_ref < lowest_divref)
                continue;
            for (f = 1; f <= 128; f++) {
                divided_fb = vco_fb / f;
                exact = (((r * divided_ref) == ref) && ((divided_fb * f) == vco_fb)) ? 1 : 0;
                if (exact && !found_exact) {
                    found_exact = 1;
                }
                if (exact && (divided_ref == divided_fb)) {
                    solved = 1;
                    actual_frequency = (float) rounded_target;
                    goto conclude;
                }
                if (!exact && !doing_exact) {
                    float error;
                    actual_frequency = ((float) ref / (float) r * (float) f * (float) 2.0 / (float) q);
                    error = (float) 100.0 * (((float) actual_frequency / (float) target) - (float) 1.0);
#ifdef STANDALONE
                    //printf("   Trying Solution: VCO %4d Mhz q %2d r %2d f %3d ref=%d Hz actual=%.4f Mhz (%.2f%% %s)\n",
                    //    vco / 1000000, q, r, f, divided_ref, actual_frequency / 1000000.0, error, str);
                    if (error < (float) 0.0) {
                        str = "low ";
                        error = -error;
                    } else {
                        str = "high";
                    }
                    if (error < required_accuracy) {
                        approx = 1;
                        printf("   INEXACT Solution: VCO %4d Mhz q %2d r %2d f %3d ref=%d Hz actual=%.4f Mhz (%.2f%% %s)\n",
                                vco / 1000000, q, r, f, divided_ref, actual_frequency / 1000000.0, error, str);
                    }
#else /* STANDALONE */
                    if (error < (float) 0.0) {
                        error = -error;
                    }
                    if (error < required_accuracy) {
                        /* This will do OK. */
                        approx = 1;
                        goto conclude;
                    }
#endif /* STANDALONE */
                }
            }
        }
    }

    conclude: if (solved || approx) {
        if (solved) {
            printf("     EXACT Solution: VCO %4d Mhz q %2d (divq %d) r %2d f %3d ref=%d Hz\n", vco / 1000000, q, divq, r, f, divided_ref);
        } else {
            printf("     No EXACT solution found, one or more approximate solutions found\n");
        }
        pll->pll_divr = r - 1;
        pll->pll_divf = f - 1;
        pll->pll_divq = divq;

        if (hcm_force_pll_range >= 0) {
            /* for loop filter characterization work */
            pll->pll_range = hcm_force_pll_range;
        } else {
#ifdef USE_LOWEST_JITTER_RANGE
            /* highest range for best jitter performance */
            pll->pll_range = 7;
#else /* USE_LOWEST_JITTER_RANGE */
            /* what Analog Bits tell you to do, which seems to be broken */
            if (104000000 <= divided_ref && divided_ref < 166000000)
                pll->pll_range = 6;
            else if (65000000 <= divided_ref && divided_ref < 104000000)
                pll->pll_range = 5;
            else if (42000000 <= divided_ref && divided_ref < 65000000)
                pll->pll_range = 4;
            else if (26000000 <= divided_ref && divided_ref < 42000000)
                pll->pll_range = 3;
            else if (16000000 <= divided_ref && divided_ref < 26000000)
                pll->pll_range = 2;
            else if (10000000 <= divided_ref && divided_ref < 16000000)
                pll->pll_range = 1;
            else if (166000000 <= divided_ref && divided_ref < 200000000)
                pll->pll_range = 7;
            else
                pll->pll_range = 0;
#endif /* USE_LOWEST_JITTER_RANGE */
        }

        last_pll_parameters = ((uint32_t) f - 1) | (((uint32_t) r - 1) << 8) | ((uint32_t) divq << 16) | ((uint32_t) pll->pll_range << 24);
        return ((uint16_t) ((uint32_t) actual_frequency / 1000000.0));
    }

    printf("No solution found\n");
    last_pll_parameters = (uint32_t) 0x80000000;
    return ((uint16_t) 0);
}

#ifdef STANDALONE

/**
 * Program entry point for standalone operation
 * @param argc
 * @param argv
 */
void main(int argc, char *argv[]) {
    struct hf_pll_config pll;
    uint32_t target;
    int i;

    for (i = 1; --argc; i++) {
        target = atoi(argv[i]);
        pll_calc(&pll, target * 1000000, 25000000);
    }
}

#endif /* STANDALONE */
