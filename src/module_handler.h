/** @file module_handler.h
 * @brief Handles module bringup and communication between modules
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

#ifndef _module_handler_h
#define _module_handler_h

#define MAX_SLAVES                      4   //!< Maximum number of slaves

/**
 * Modules status
 */
typedef struct {
    uint16_t inputMillivolts[4];    //!< Input voltage in mV
    uint16_t outputMillivolts[4];   //!< Output voltage in mV
} moduleStatusT;

extern moduleStatusT moduleStatus[MAX_SLAVES + 1];

/** Time after master power-up before slave power supplies are turned on */
#define SLAVE_POWERUP_COMMAND_DELAY_MS  1000
/** Time after master power-up before slaves are told to start up */
#define SLAVE_STARTUP_COMMAND_DELAY_MS  1300
/**
 * Time after slave startup before slave power status is checked
 * which happens in the slave at 1300+300 = 1600 msec after master power-up
 */
#define SLAVE_POWERUP_DELAY_MS          300
/** Time to wait after master power-up for everything to stabilize, msec */
#define MASTER_POWERUP_DELAY_MS         1800

void module_init(void);
void module_handler(void);

boardidT module_type(uint8_t module);
void module_serial(uint8_t module, serial_number_t *serial);
uint32_t fw_version(uint8_t module);
bool fw_crc(uint8_t module, uint32_t *crc);
void module_voltage_set(uint8_t module, uint8_t die, uint16_t millivolts);
void power_supply_on(void);
void usb_powerup_request(void);
void usb_powerdown_request(void);
bool system_powering_down(void);
bool system_off(void);
bool system_on(void);

bool fan_diagnostics(void);
void fpga_route_setup_single_die(int);
void fpga_route_setup(bool);
void fpga_reg_write(uint8_t, uint8_t);

extern uint8_t input_power_good;

extern uint16_t activity_led_counter;

extern uint8_t host_watchdog_clock;
extern bool host_watchdog_active;

extern uint8_t status_watchdog_clock;
extern bool status_watchdog_active;

extern volatile uint16_t msec_ticker;
extern volatile uint16_t sec_ticker;
void delay_msec(uint16_t);

void check_watchdog(void);

void bleep(int);
void bleep_once(int status);

/*
 * Elapsed msec
 */
#define elapsed_since(start)   (((msec_ticker) < (start)) ? \
                               (((uint16_t)65535 - (start)) + (msec_ticker) + (uint16_t)1) :    /* Wrapped case */ \
                               ((msec_ticker) - (start)))                                       /* Most of the time */

/*
 * Elapsed seconds
 */
#define seconds_elapsed_since(start)    (((sec_ticker) < (start)) ? \
                                        (((uint16_t)65535 - (start)) + (sec_ticker) + (uint16_t)1) : \
                                        ((sec_ticker) - (start)))

/*
 * Chain configurations
 */
#define CC_UNCONFIGURED                 0   //!< Not yet configured
#define CC_NONE                         1   //!< Standalone module
#define CC_MIDDLE                       2   //!< In the middle of a chain
#define CC_OPEN_UP                      3   //!< Modules exist "UP" of me, I'm the "DOWN"-est
#define CC_OPEN_DOWN                    4   //!< Modules exist "DOWN" of me, I'm the "UP"-est
#define CC_LOOPBACK                     5   //!< Single module with a loopback cable in place (factory test)

#endif /* _module_handler_h */
