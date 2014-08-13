/** @file asic_handler.h
 * @brief ASIC initialization, chain initialization and periodic task
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

#ifndef _asic_handler_h
#define _asic_handler_h

#ifdef __cplusplus
extern "C" {
#endif

void asic_init(void);

void chain_init(uint8_t);

void chain_handler(void);

void display_chain_status(void);

void chain_usb_init(struct hf_usb_init_header *);

void shutdown_request(void);

int make_config_frame(struct hf_header *, uint8_t, struct hf_config_data *, uint16_t, uint16_t, uint8_t);

uint8_t send_core_map(struct hf_header *h);

void make_abort(struct hf_header *, uint8_t, uint8_t, uint16_t);

void set_mixed_slave_baudrate(void);

#ifdef FEATURE_COWARDLY_WORK_RESTART

bool gwq_work_restart_process(struct hf_header *);

void gwq_work_restart_check(uint8_t, uint16_t *);

#endif /* FEATURE_COWARDLY_WORK_RESTART */

typedef struct {
    uint8_t F, R, Q, range;
} pll_divisors_t;

typedef struct {
    uint32_t freq;
    uint8_t F, R, Q, range;
} pll_entry_t;

extern const pll_entry_t pll_table[];
extern const uint8_t pll_table_entries;

extern uint32_t last_pll_parameters;
extern bool dont_do_pll_tweakup;
extern int8_t hcm_force_pll_r;
extern int8_t hcm_force_pll_range;

extern uint32_t dynamic_nonce_range[MAX_DIE];

#ifdef __cplusplus
}
#endif

#endif /* _asic_handler_h */
