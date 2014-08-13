/** @file hf_nvram.h
 * @brief Everything to do with the USER flash page
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

#ifndef __HF_NVRAM_H__
#define __HF_NVRAM_H__

#define NVRAM_REVISION_LEVEL 1

#define G1_CORES                384
#define HF_NAME_SIZE            32
#define DEFAULT_REF_CLOCK       125     // Rev-0, Rev-1 Linear boards

/**
 * Serial number data
 * First 16 bytes of USER page
 */
typedef struct {
    uint32_t magic;
    unsigned char start_barrier[4];
    unsigned char unique_id[16];
    unsigned char stop_barrier[4];
} serial_number_t;

/**
 * Operational limits
 * 32 bytes
 */
typedef struct {
    uint16_t max_clock_rate;
    uint16_t max_thermal_trip;
    uint16_t max_temp_high;
    uint16_t max_temp_low;
    uint16_t min_voltage;
    uint16_t max_voltage;
    uint16_t spare1;
    uint16_t spare2;
    uint32_t spare3[4];
} op_limits_t;

/**
 * Operational history
 */
typedef struct {
    uint64_t hashes;
    uint64_t nonces;
    uint16_t max_clock_rate;
    uint16_t max_die_temp;
    uint16_t max_board_temp;
    uint16_t watchdog_timeouts;
} op_history_t;

/**
 * Fan settings
 */
typedef struct {
    uint8_t revision;
    uint8_t spare[3];
    struct {
        uint8_t mode;
        uint8_t opt[3];
    } fans[2];
} fan_settings_t;

void hf_nvram_check(void);
uint32_t hf_nvram_get_short_serial(void);
uint8_t hf_nvram_op_serial(struct hf_header *);
void hf_nvram_get_serial(serial_number_t *ptr);
void hf_nvram_set_usb_serial(void);
uint8_t hf_nvram_op_limits(struct hf_header *);
uint8_t hf_nvram_op_history(struct hf_header *);
bool hf_nvram_write_die_settings(uint8_t, const op_settings_t *);
void hf_nvram_read_die_settings(op_settings_t *);
uint8_t hf_nvram_op_die_settings(struct hf_header *);
bool hf_nvram_die_settings_valid(void);
op_settings_t *hf_nvram_die_settings(void);
void hf_nvram_init_die_settings(void);
void hf_nvram_get_slave_die_settings(void);
void hf_nvram_fan_settings_set(const fan_settings_t *fan);
uint8_t hf_nvram_fan_settings(struct hf_header *);
fan_settings_t *hf_nvram_get_fan_settings(void);
void hf_nvram_write_bad_core_bitmap(uint8_t, uint16_t);
void hf_nvram_read_bad_core_bitmap(uint8_t, uint16_t *);
uint16_t *hf_nvram_bad_core_bitmap(void);
bool hf_nvram_bad_core_bitmap_valid(void);
uint8_t hf_nvram_op_bad_core(struct hf_header *);
uint8_t hf_nvram_op_name(struct hf_header *);
void hf_nvram_name_set(char *);
char *hf_nvram_name(void);

#ifdef FEATURE_INITIALIZE_NON_VALID_DIE_SETTINGS
uint8_t hf_nvram_physical_die(uint8_t);
#endif /* FEATURE_INITIALIZE_NON_VALID_DIE_SETTINGS */

extern die_settings_t all_die_settings[];
extern uint8_t all_ref_clocks[];
extern uint8_t module_ref_clocks[];

#endif /* __HF_NVRAM_H__ */
