/** @file hf_factory.h
 * @brief Factory codes and structures
 *
 * Secret operation codes and structures, that are only used for Factory test
 * and internal diagnostics.
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

/* Factory OP codes */
#define OP_SERIAL               50  //!< Serial number read/write
#define OP_LIMITS               51  //!< Operational limits read/write
#define OP_HISTORY              52  //!< Read operational history data
#define OP_CHARACTERIZE         53  //!< Characterize one or more die
#define OP_CHAR_RESULT          54  //!< Characterization result
#define OP_SETTINGS             55  //!< Read or write settings
#define OP_FAN_SETTINGS         56
#define OP_POWER                57
#define OP_BAD_CORE             58  //!< Set or clear bad core status
/* Diagnostic power on/off */
#define DIAGNOSTIC_POWER_ON     0x1
#define DIAGNOSTIC_POWER_OFF    0x2

/* USER page magic number */
#define U_MAGIC     0x42aa

/**
 * OP_CHARACTERIZE usb data
 */
struct hf_characterize {
    uint16_t dac_low, dac_high;             //!< DAC settings
    uint8_t v_low, v_high;                  //!< Core voltage settings, in mV*10, 81 = 0.81 volts
    uint8_t dac_incr, v_incr;               //!< Only one allowed - either DAC or voltage
    uint16_t f_low, f_high;                 //!< Frequency range, inclusive (MHz)
    uint8_t f_incr;                         //!< Frequency increment
    uint8_t die_low, die_high, die_incr;    //!< Which die to step through, inclusive
    uint8_t temp_low, temp_high, temp_incr; //!< Die temperature setpoint and steppings
    uint8_t thermal_override;               //!< Thermal overload limit if not zero
    uint16_t flags;
    uint8_t other[2];
}__attribute__((packed,aligned(4)));

/* Flags in OP_CHARACTERIZE */
#define F_TEST_WITH_ALL_DIE_CORES_HASHING       0x1
#define F_TEST_WITH_ALL_ASIC_CORES_HASHING      0x2
#define F_RETURN_CORE_MAPS                      0x4
#define F_RETURN_PLL_PARAMETERS                 0x8
#define F_DONT_DO_PLL_TWEAKUP                   0x10
#define F_HASH_STANDALONE                       0x20
#define F_FORCE_PLL_R                           0x40    //!< R in other[0]
#define F_FORCE_PLL_RANGE                       0x80    //!< Range in other[1]
#define F_PLL_TABLE_SWEEP                       0x100   //!< Sweep frequency across PLL table between set ranges

/**
 * OP_CHARACTERIZE results.
 * As many of these as are implied by the test range will be returned as the
 * characterization run progresses.
 * 20 bytes (+ core_map if present)
 */
struct hf_characterize_result {
    uint16_t dac_setting;           //!< Either from dac_nnn or as a result of setting v_nnn
    uint16_t frequency;             //!< MHz
    uint16_t die_temperature;       //!< As an ADC count, convert with usual macro
    uint8_t core_voltage;           //!< As set
    uint8_t die_index;
    uint8_t error_code;
    uint8_t measured_voltage;       //!< As measured
    uint8_t spare2;
    uint8_t spare3;
    uint8_t good_cores_low_speed;   //!< How many cores were good in the low frequency isolated test (this die)
    uint8_t good_core_count;        //!< How many cores were good in this test case (this die)
    uint8_t good_half_cores;        //!< How many cores were "half" good for this test case (this die)
    uint8_t flags;
    uint32_t other_data;            //!< Optional requested data
}__attribute__((packed,aligned(4)));

/* Flags in OP_CHARACTERIZE_RESULT */
#define F_FAILURE               0x1
#define F_LAST_THIS_DIE         0x2
#define F_LAST                  0x4
#define F_CORE_MAP_TO_FOLLOW    0x8
#define F_PLL_DATA_RETURNED     0x10

/**
 * Frequency and voltage for each die in pre-configured operating settings.
 */
typedef struct {
    uint16_t frequency; //!< In Mhz
    uint16_t voltage;   //!< In mV. 0 = die not enabled. (Rev-1 kludge right now - If b15 set = dac_setting)
} die_settings_t;

/**
 * Pre-configured operating settings, held in nvram (user page @see hf_nvram.c)
 * These may be determined and set up with a manufacturing or installation
 * "characterization" run
 * Note for chained modules:
 *   Master uC will need to get frequency for each die from the slave modules,
 *   since the master is what does the PLL configuration, and people could swap
 *   modules out to repair systems, so the settings need to stay as part of the
 *   module itself.
 * 20 bytes total
 */
typedef struct {
    uint8_t revision;       //!< Revision of settings to follow
    uint8_t ref_frequency;  //!< Reference clock frequency
    uint16_t magic;         //!< Extra validation
    die_settings_t die[4];  //!< Frequency and voltage for each die
} op_settings_t;

#define DIE_SETTINGS_REVISION           1
#define VALID_DIE_SETTINGS(x)           ((x).revision != 0 && (x).revision != 255 && (x).revision <= DIE_SETTINGS_REVISION && (x).magic == U_MAGIC)

/* For forcing setting into field Rev-0 and Rev-1 boards */
#define REV1_DIE_SETTINGS_REVISION      1

#define DEFAULT_LINEAR_DAC_SETTING      1150
#define DEFAULT_G1_HASHCLOCK            550

#define DS_DACSETTING                   0x8000
#define DS_DEFAULT_VOLTAGE              (DS_DACSETTING | DEFAULT_LINEAR_DAC_SETTING)
#define DS_DISABLED                     0x0
#define DS_NEVER_SET                    0xffff
