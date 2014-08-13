/** @file main.h
 * @brief Program entry point and main execution loop
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

#ifndef _MAIN_H_
#define _MAIN_H_

/* 0 is development firmware */
#define FIRMWARE_VERSION        0x0007          //!< v0.7.0

extern const char *iface_ver;
extern const char *firmware_ver;
extern const char *when_compiled;

/**
 * Version Fail Codes
 */
enum ver_fail {
    not_primary_board = 0,      //!< not_primary_board
    app_magic_bad,              //!< app_magic_bad
    unknown_ver_type,           //!< unknown_ver_type
    ver_string_too_large,       //!< ver_string_too_large
    bad_header                  //!< bad_header
};

/**
 * Version Type
 */
enum ver_type {
    interface_version = 0,      //!< interface_version
    firmware_version,           //!< firmware_version
    time_compiled,              //!< time_compiled
    firmware_crc32,             //!< firmware_crc32
    version_fail                //!< version_fail
};

#include "usb_protocol_cdc.h"

/**
 * LED Modes
 */
enum led_mode_t {
    LED_IDLE = 0,               //!< LED_IDLE
    LED_AUTO,                   //!< LED_AUTO
    LED_STATIC,                 //!< LED_STATIC
    LED_PLAID,                  //!< LED_PLAID
    LED_ERROR,                  //!< LED_ERROR
    LED_HASH_SLOW,              //!< LED_HASH_SLOW
    LED_HASH_REALLYSLOW,        //!< LED_HASH_REALLYSLOW
    LED_HASH_FAST               //!< LED_HASH_FAST
};

extern uint8_t led_mode;
extern uint16_t activity_led_counter;
extern uint16_t power_led_counter;
extern uint16_t led_counter;
extern void led_handler(void);

bool main_cdc_enable(uint8_t port);
void main_cdc_disable(uint8_t port);
void main_sof_action(void);
void main_suspend_action(void);
void main_resume_action(void);
void main_cdc_set_dtr(uint8_t port, bool b_enable);
void wdt_init(void);
void self_reset(void);
void fans_init(void);
void set_fan_speed(uint8_t which, uint8_t speed);
void fan_set(uint8_t module, uint8_t fan, uint16_t speed);
uint16_t fan_get(uint8_t module, uint8_t fan);
void thermal_control(void);
void convert_to_hex(unsigned char *data, char *s, int len);

#define ASSUME_POWER_GOOD
/* For all the uses of MIN, 125 is OK even if reference frequency is 25 */
#define MIN_HASH_CLOCK_MHZ                  125
#define MAX_HASH_CLOCK_MHZ                  1100
//#define BYPASS_BOARD_CHECKS
//#define INCLUDE_TRACING
//#define FEATURE_12V_STATIC_ON

/* TODO
 * Things to change for release candidates:
 * Notify the user that this firmware is for DEBUG use only, not production.
 */

/* TODO disable this before release */
//#define FEATURE_NOTIFY_HOST_DEBUG
/* TODO and enable these before release candidates: */
#define FEATURE_HW_WATCHDOG
#define FEATURE_HOST_WATCHDOG
#define FEATURE_STATUS_WATCHDOG

/* end of release candidate define changes */

#define STATUS_WATCHDOG_RECHARGE       15
#define HOST_WATCHDOG_RECHARGE         90

//#define FEATURE_CORE_BLABBER_HARDMAP
//#define FEATURE_CORE_BLABBER_MAP_REBOOT
#define FEATURE_CORE_BLABBER_SOFTMAP
#define FEATURE_CORE_BLABBER_ABORT
#define FEATURE_CORE_BLABBER_USB_SQUELCH

#define FEATURE_INITIALIZE_NON_VALID_DIE_SETTINGS

#define MAX_NOTIFY_MESSAGE_SIZE         100

/* TODO
 * Whenever we add encryption/authentication, make sure to turn the CLI off for
 * any builds to be released to customers, or restrict what the CLI can do to
 * avoid creating backdoors.
 */
#define FEATURE_DEBUG_CLI

/*
 * Collect code timing info; turn off for release builds since it would be
 * adding some overhead with no benefit.
 */
//#define FEATURE_PROFILE

/*
 * Don't turn this on any more, because it means clock rate changes won't work.
 * It also makes things blow up because cgminer sends the extra work
 * immediately, so we get work overruns and everything breaks down.
 */
//#define FEATURE_DISABLE_ABORTS

/* Don't turn this on for G-1 boards */
//#define FEATURE_VIOLENT_WORK_RESTART

/*
 * Enable this, restart procedure is:
 *   - Multicast abort all active jobs, all pending jobs become active
 *   - Unicast aborts to the first N cores (N set to 48)
 *   - Multicast abort the rest to collaps everything
 *   - Change hash clock rates if specified
 */
#define FEATURE_COWARDLY_WORK_RESTART

/* Include hf_factory.h and associated features */
#define FEATURE_FACTORY_TESTS
#define FEATURE_FORCE_FACTORY_MODE

/* Blink the Activity LED when an OP_HASH processed */
//#define FEATURE_ACTIVITY_LED_IS_HASH

/* Testing of ASIC under range of voltages, clock speeds, configurations */
#define FEATURE_CHARACTERIZATION

/* Standalone test pass criteria (apart from all board/asic tests working) */
#define MINIMUM_ACCEPTABLE_GOODCORES_PER_DIE        85

#ifdef FEATURE_CHARACTERIZATION
#define FEATURE_FACTORY_TESTS
#endif /* FEATURE_CHARACTERIZATION */

#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include "asf.h"
#include "uc3b_peripherals.h"
#include "hf_protocol.h"
#include "hf_factory.h"
#include "hf_trace.h"
#include "hf_util.h"
#include "hf_pid.h"
#include "hf_nvram.h"
#include "usb_uart.h"
#include "boardid.h"
#include "module_handler.h"
#include "gwq_handler.h"
#include "cores.h"
#include "asic_handler.h"
//#include "asic_emulator.h"
#include "spi_handler.h"
#include "twi_handler.h"
#include "profile.h"
#include "wdt.h"

void SetupHardware(void);

void CheckUsbIncoming(void);
void DispatchUsbIncoming(uint8_t *, uint8_t);
void CheckUartIncoming(void);
void CheckUartOutgoing(void);
void DisplayStatus(void);
uint16_t pll_calc(struct hf_pll_config *, uint32_t, uint32_t);
void notify_host(char *format, ...);

#define DIMENSION(x) (sizeof(x)/sizeof(x[0]))

/* AVR32 is Big endian */
#undef cpu_to_le16
#define cpu_to_le16(x)  __builtin_bswap_16(x)
#undef cpu_to_le32
#define cpu_to_le32(x)  __builtin_bswap_32(x)
#undef cpu_to_le64
#define cpu_to_le64(x)  ((((((uint64_t)__builtin_bswap_32((x)&0xffffffff))<<32)))|((uint64_t)__builtin_bswap_32((x)>>32)&0xffffffff))
#undef le16_to_cpu
#define le16_to_cpu(x)  __builtin_bswap_16(x)
#undef le32_to_cpu
#define le32_to_cpu(x)  __builtin_bswap_32(x)
#define BSWAP32_IF_BE(x) ((((x)>>24)&0xff) | (((x)>>8)&0xff00) | (((x)<<8)&0xff0000) | (((x)<<24)&0xff000000))

uint8_t op_version(struct hf_header *h);
uint8_t op_ping(struct hf_header *h);

extern __attribute__((__section__(".spurious1")))  uint32_t spurious1;

/* TODO
 * Crude debugging.
 * Comment out the function prototype and uncomment the #define to compile out
 * all uprintf's.
 */
#define uprintf(...)
//void uprintf(uint16_t, char *, ...);

#define UD_WORK_RESTART     0x1
#define UD_SEQUENCE         0x2
#define UD_STARTUP          0x4
#define UD_CHAR             0x8

//#define WORK_RESTART_DEBUG

#endif /* _MAIN_H_ */
