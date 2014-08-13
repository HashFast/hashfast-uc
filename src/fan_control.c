/** @file fan_control.c
 * @brief Everything to do with driving FANs
 *
 * Each module controls its own radiator fan, based on die temperature
 * Load shedding for that module is used if control is out of range.
 *
 * Both Sierra and Baby Jet
 * The pump fan is connected to J8.  (PWM Channel: N/A)
 *
 * Baby Jet
 * The baby jet radiator fan is connected to J9.  (PWM channel: 4)
 * The baby jet rear chassis fan is connected to J10.  (PWM channel: N/A)
 * The baby jet front chassis fans are direct connect to PSU.  (PWM channel: N/A)
 *
 * Sierra
 * The Sierra radiator fan is connected to J11.  (PWM channel: 0)
 *
 * The "master" module, i.e. the one with an attached USB link, controls
 * radiator fans (all of 'em), based on board temperature sensors. A really
 * dumb apprach is taken. The highest board temperature is selected as the
 * "input", and all fans are slaved to the same speed. If this speed is pinned
 * at 100%, then load shedding occurs to maintain board conditions within limits.
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

#include "main.h"

static void set_J11_fan_speed(int);
static void set_J9_fan_speed(int);

static hf_pid_t pid_J11; //!< J11 fan PID control
static hf_pid_t pid_J9;  //!< J09 fan PID control

#define MAX_FANS       (sizeof(((fan_settings_t *) 0)->fans) / \
                        sizeof(((fan_settings_t *) 0)->fans[0]))

static struct {
    hf_pid_t maxDiePID;
    hf_pid_t maxRegulatorPID;
    struct {
        struct {
            bool set;
            uint8_t speed;
        } fans[MAX_FANS];
    } modules[MAX_SLAVES + 1];
} fanState;

/* mode values for fan_settings_t */
#define FAN_MODE_FIXED          1
#define FAN_MODE_ADAPTIVE_BLOCK 2
/* PWN channels */
#define J9_FAN                  4 //!< J11 PWM channel #
#define J11_FAN                 0 //!< J09 PWM channel #
/* PWN config */
#define PWM_PERIOD              300
#define INITIAL_J9_FAN_PWM      300
#define INITIAL_J11_FAN_PWM     100
/* Baby Jet radiator fans need higher PWM to effectively cool */
#define MINIMUM_J9_FAN_PWM      200
/* Sierra units can handle a much lower fan speed */
#define MINIMUM_J11_FAN_PWM     50
/* PID sample */
#define PID_SAMPLE_TIME_MS      100
/* J11 initial values */
#define KP_init_J11             1.0
#define KI_init_J11             0.5
#define KD_init_J11             0.1
/* J09 initial values */
#define KP_init_J9              1.0
#define KI_init_J9              0.5
#define KD_init_J9              0.1

static avr32_pwm_channel_t pwm_channel;

/**
 * Initialize fans
 */
void fans_init(void) {
    struct ucinfo_t *info = &ucinfo;
    pwm_opt_t pwm_opt;
    /* configure GPIO */
    gpio_configure_pin(FAN1, GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
    gpio_enable_module_pin(PWM1, AVR32_PWM_4_0_FUNCTION);
    gpio_configure_pin(FAN2, GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
    gpio_enable_module_pin(PWM2, AVR32_PWM_0_0_FUNCTION);
    /* PWM options */
    pwm_opt.diva = AVR32_PWM_DIVA_CLK_OFF;
    pwm_opt.divb = AVR32_PWM_DIVB_CLK_OFF;
    pwm_opt.prea = AVR32_PWM_PREA_MCK;
    pwm_opt.preb = AVR32_PWM_PREB_MCK;
    /* PWM initialize */
    pwm_init(&pwm_opt);
    /* PWM channel configuration */
    pwm_channel.CMR.calg = PWM_MODE_LEFT_ALIGNED;       // Channel mode
    pwm_channel.CMR.cpol = PWM_POLARITY_HIGH;           // Channel polarity
    pwm_channel.CMR.cpd = PWM_UPDATE_DUTY;
    pwm_channel.CMR.cpre = AVR32_PWM_CPRE_MCK_DIV_8;    // Channel clock
    /* duty cycle needs to be < CPRD */
    pwm_channel.cdty = INITIAL_J9_FAN_PWM;              // Channel duty cycle
    pwm_channel.cprd = PWM_PERIOD;                      // Channel period
    pwm_channel.cupd = 0;

    /*
     * Fan configuration based on clocks
     * 60 Mhz / 8 / PWM_PERIOD => 25 kHz fan pwm outputs
     */

    /* set FAN1 configuration */
    pwm_channel_init(J9_FAN, &pwm_channel);
    /* duty cycle needs to be < CPRD */
    pwm_channel.cdty = INITIAL_J11_FAN_PWM;             // Channel duty cycle
    /* set FAN2 configuration */
    pwm_channel_init(J11_FAN, &pwm_channel);
    /* start fans */
    pwm_start_channels((1 << J11_FAN) | (1 << J9_FAN));

    /* initialize J11 fan control */
    memset(&pid_J11, 0, sizeof(pid_J11));
    hf_pid_init(&pid_J11, info->target_die_temperature_x10, info->highest_die_temperature_x10, (float) INITIAL_J11_FAN_PWM, KP_init_J11, KI_init_J11, KD_init_J11, true);
    hf_pid_change_sample_time(&pid_J11, PID_SAMPLE_TIME_MS);
    hf_pid_set_output_limits(&pid_J11, (float) PWM_PERIOD * 0.3, (float) PWM_PERIOD);
    set_J11_fan_speed(INITIAL_J11_FAN_PWM);

    /* initialize J09 fan control */
    memset(&pid_J9, 0, sizeof(pid_J9));
    hf_pid_init(&pid_J9, info->target_board_temperature_x10, info->highest_board_temperature_x10, (float) INITIAL_J9_FAN_PWM, KP_init_J9, KI_init_J9, KD_init_J9, true);
    hf_pid_change_sample_time(&pid_J9, PID_SAMPLE_TIME_MS);
    hf_pid_set_output_limits(&pid_J9, (float) PWM_PERIOD * 0.3, (float) PWM_PERIOD);
    set_J9_fan_speed(INITIAL_J9_FAN_PWM);

}

/**
 * Sierra radiator fan
 * @param pwm
 */
static void set_J11_fan_speed(int pwm) {
    pwm_channel.cupd = pwm > MINIMUM_J11_FAN_PWM ? pwm : MINIMUM_J11_FAN_PWM;
    pwm_channel.cupd = pwm == 0 ? 0 : pwm_channel.cupd;
    pwm_async_update_channel(J11_FAN, &pwm_channel);
}

/**
 * Baby Jet radiator fan
 * @param pwm
 */
static void set_J9_fan_speed(int pwm) {
    pwm_channel.cupd = pwm > MINIMUM_J9_FAN_PWM ? pwm : MINIMUM_J9_FAN_PWM;
    pwm_channel.cupd = pwm == 0 ? 0 : pwm_channel.cupd;
    pwm_async_update_channel(J9_FAN, &pwm_channel);
}

/**
 * Set fan speed
 * @param which
 * @param speed
 */
void set_fan_speed(uint8_t which, uint8_t speed) {
    uint16_t pwm;

    pwm = ((uint32_t) speed * PWM_PERIOD) / 255;
    switch (which) {
    case 0:
        set_J9_fan_speed(pwm);
        break;
    case 1:
        set_J11_fan_speed(pwm);
        break;
    }
}

/**
 * Set fan speed for later commit
 * Allowed to be called from ISR
 * @param module
 * @param fan
 * @param speed
 */
void fan_set(uint8_t module, uint8_t fan, uint16_t speed) {
    unsigned int i;

    if (fan < MAX_FANS && (speed & 0xff00) == 0) {
        for (i = 0; i < MAX_SLAVES + 1; i++)
            if (module == i || module == 0xff) {
                fanState.modules[i].fans[fan].speed = speed;
                fanState.modules[i].fans[fan].set = true;
            }
    }
}

/**
 * Get fan speed
 * @param module
 * @param fan
 * @return
 */
uint16_t fan_get(uint8_t module, uint8_t fan) {
    uint16_t v;

    if (fan < MAX_FANS && module < MAX_SLAVES + 1)
        v = fanState.modules[module].fans[fan].speed;
    else
        v = 0;

    return v;
}

/**
 * Periodic fan control task
 * Maintain good operating temperature by control of airflow
 */
void thermal_control(void) {
    static fan_settings_t *fanSettings;
    static enum {
        resetTS,
        idleTS,
        waitTWICompleteTS
    } state = resetTS;
    static uint8_t txBuffer[3];
    static twiRequestT req;
    static unsigned int module;
    static unsigned int fan;

    switch (state) {
    case resetTS:
        if (ucinfo.board_initialized) {
            fanSettings = hf_nvram_get_fan_settings();
            if (fanSettings->revision == 0) {
                for (fan = 0; fan < MAX_FANS; fan++) {
                    if (fanSettings->fans[fan].mode == FAN_MODE_FIXED) {
                        for (module = 0; module < MAX_SLAVES + 1; module++) {
                            fanState.modules[module].fans[fan].speed = fanSettings->fans[fan].opt[0];
                            fanState.modules[module].fans[fan].set = true;
                        }
                    }
                }
            }
            module = 0;
            fan = 0;
            state = idleTS;
        }
        break;
    case idleTS:
        if (!ucinfo.board_initialized)
            state = resetTS;
        else if (ucinfo.master) {
            if (++fan >= MAX_FANS) {
                fan = 0;
                if (++module >= ucinfo.num_slaves + 1)
                    module = 0;
            }
            if (fanState.modules[module].fans[fan].set) {
                fanState.modules[module].fans[fan].set = false;
                if (module == 0)
                    set_fan_speed(fan, fanState.modules[module].fans[fan].speed);
                else {
                    req.addr = TWI_SLAVE_STARTADDR + module - 1;
                    txBuffer[0] = TWICMD_FAN_SET;
                    txBuffer[1] = fan;
                    txBuffer[2] = fanState.modules[module].fans[fan].speed;
                    req.tx = txBuffer;
                    req.txLength = 3;
                    req.rx = NULL;
                    req.rxLength = 0;
                    twiQueueRequest(TWI_BUS_UC, &req);
                    state = waitTWICompleteTS;
                }
            }
        }
        break;
    case waitTWICompleteTS:
        if (!req.pending)
            state = idleTS;
        break;
    }

#if 0
    struct ucinfo_t *info = &ucinfo;
    int pwm;

    /* die temperature is controlled by radiator fan speed */
    if (hf_pid_compute(&pid_J11, info->highest_die_temperature_x10)) {
        pwm = (int) pid_J11.output;
        set_J11_fan_speed(pwm);
        /*
         * If the die temperature is excessive, we may need to shed load.
         * Especially if the radiator fan is "flat out"
         * XXX to be sorted out...
         */
    }

    /*
     * Board (power supply) temperature is controlled by radiator fan speed.
     * Only the master does this control.
     */
    if (info->master && hf_pid_compute(&pid_J9, info->highest_board_temperature_x10)) {
        pwm = (int) pid_J9.output;
        set_J9_fan_speed(pwm);
        // XXX slave fan control
        //twi_set_slave_fans(pwm);
        /*
         * If the board temperature is excessive, we may need to shed load.
         */
    }
#if 0
    /* determined above */
    if (shed_load > 0) {
        /*
         * Start sending OP_ABORTs
         */
    }
#endif /* 0 */
#endif /* 0 */
}

/**
 * Fan diagnostic state
 */
enum fan_diagnostic_state_t {
    FD_IDLE = 0,    //!< FD_IDLE
    FD_CHECK_TACHOS,//!< FD_CHECK_TACHOS
    FD_DONE         //!< FD_DONE
};

//static uint8_t fd_state = FD_IDLE;

/**
 * FAN diagnostics, run at startup
 * @return success
 */
bool fan_diagnostics() {
    uint16_t fd_time;

    fd_time = msec_ticker;
    set_J11_fan_speed(PWM_PERIOD);
#if 0
    while (elapsed_since(fd_time) < 500)
    ;
    // XXX Check tacho
#endif /* 0 */
    set_J11_fan_speed(INITIAL_J11_FAN_PWM);

    return (true);
}
