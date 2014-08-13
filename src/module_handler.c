/** @file module_handler.c
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

#include "main.h"
#include "ir3566b.h"
#include "boardid.h"
#include "hf_loader_p.h"

/* Number of seconds of no work sent before forced reset. */
#define WORK_FAILURE_CLOCK 15
#define WORK_SLOW_CLOCK 4

static bool power_button_pressed;
static bool power_down_request;
static bool reconfig_button_pressed;
bool real_powerbutton_pressed = false;
bool power_up_only = false;
bool do_asic_reset = false;
moduleStatusT moduleStatus[MAX_SLAVES + 1];

/**
 * Module functions
 */
static struct {
    uint16_t millivolts[4];     //!< Module voltage in mV
    bool voltageSet[4];         //!< Set voltage in mV
} moduleFuncs[MAX_SLAVES + 1];

/**
 * Slave information
 */
static struct {
    serial_number_t serial;     //!< Serial number
    uint32_t version;           //!< Version
    uint32_t crc;               //!< CRC
    bool crc_valid;             //!< CRC valid
    boardidT type;              //!< BoardID
} slave_info[MAX_SLAVES];

static bool board_diagnostics(void);
static void fpga_diagnostics(void);
static void set_board_voltage(void);
static void msec_tick(void);

/**
 * GPIO interrupt routine
 */
__attribute__((__interrupt__)) static void gpio_interrupt_handler(void) {
    profileEnter(PROFILE_CHANNEL_GPIO_ISR);
    /* clear the pin change interrupt flag */
    power_button_pressed = gpio_get_pin_interrupt_flag(POWER_BUTTON);
    reconfig_button_pressed = gpio_get_pin_interrupt_flag(RECONFIG_BUTTON);

    if (power_button_pressed)
        gpio_clear_pin_interrupt_flag(POWER_BUTTON);
    if (reconfig_button_pressed)
        gpio_clear_pin_interrupt_flag(RECONFIG_BUTTON);
#if 1
    // XXX Temp to get standalone test working today before I leave for the airport.
    power_button_pressed = reconfig_button_pressed;
    real_powerbutton_pressed = reconfig_button_pressed;
#endif /* 1 */
    profileExit(PROFILE_CHANNEL_GPIO_ISR);
}

/* Triggers a reboot if it hasn't heard from the host. */
bool host_watchdog_active = 0;
/* Wait for a timeout period allow init before invoking watchdog. */
uint8_t host_watchdog_clock = HOST_WATCHDOG_RECHARGE;
static bool host_watchdog_pending;
static bool host_watchdog_timeout;

/* Triggers a reboot if it hasn't heard from the ASIC. */
bool status_watchdog_active = 0;
/* Not activated until ASIC chain is initialized. */
uint8_t status_watchdog_clock = STATUS_WATCHDOG_RECHARGE;
static bool status_watchdog_pending;
static bool status_watchdog_timeout;

/*
 * A timer/counter is used to generate the 1 msec ticker
 */

volatile uint16_t msec_ticker = 0;      //!< millisecond ticker
volatile uint16_t sec_ticker = 0;       //!< second ticker

/**
 * Ticks millisecond and second counters
 */
static void msec_tick() {
    msec_ticker++;
    if (msec_ticker && ((msec_ticker & 1023) == 0)) {
        /* near enough */
        sec_ticker++;

        if (host_watchdog_active && host_watchdog_clock >= 1) {
            if (host_watchdog_clock == 2)
                host_watchdog_pending = true;
            host_watchdog_clock--;
            if (host_watchdog_clock == 0)
                host_watchdog_timeout = true;
        }

        if (status_watchdog_active && status_watchdog_clock) {
            if (--status_watchdog_clock == 2)
                status_watchdog_pending = true;
            if (status_watchdog_clock == 1)
                status_watchdog_timeout = true;
        }
    }
}

/**
 * Delay by number of milliseconds
 * @param m
 */
void delay_msec(uint16_t m) {
    uint16_t s = msec_ticker;
    while (elapsed_since(s) < m);
}

/**
 * Check watchdog
 */
void check_watchdog() {
#if defined(FEATURE_STATUS_WATCHDOG) || defined(FEATURE_HOST_WATCHDOG)
    const char *fmt = "WARNING: %s watchdog timeout";
    const char *pending = "Pending";
    const char *host = "Host";

    if (ucinfo.inhibit_watchdogs) {
        return;
    }

#ifdef FEATURE_STATUS_WATCHDOG
    if (status_watchdog_pending) {
        status_watchdog_pending = 0;
        notify_host(fmt, pending);
    }
#endif /* FEATURE_STATUS_WATCHDOG */

#ifdef FEATURE_HOST_WATCHDOG
    if (host_watchdog_pending) {
        host_watchdog_pending = 0;
        notify_host(fmt, host);
    }
#endif /* FEATURE_HOST_WATCHDOG */

    if (0
#ifdef FEATURE_STATUS_WATCHDOG
            || status_watchdog_timeout
#endif /* FEATURE_STATUS_WATCHDOG */
#ifdef FEATURE_HOST_WATCHDOG
            || host_watchdog_timeout
#endif /* FEATURE_HOST_WATCHDOG */
    ) {
        /* all synchronous operations so they'll complete */
        fpga_reg_write(FA_REG_ENABLE, 0);
        if (ucinfo.num_slaves) {
            /* tell slaves to reboot (sync write so will complete) */
            twi_broadcast(TWICMD_REBOOT, 0);
        }
        self_reset();
    }
#endif /* defined(FEATURE_STATUS_WATCHDOG) || defined(FEATURE_HOST_WATCHDOG) */
}

/*
 * The following functions called by modified udi_cdc.c to ensure it can't loop
 * forever when host driver goes away. We put a 200 msec timeout on it.
 */

void hashfast_init_udi_cdc_write_timeout(void);
bool hashfast_udi_cdc_timeout(void);

#define USB_WRITE_TIMEOUT 200

static uint16_t udi_cdc_start;
static uint32_t udi_cdc_loops;

/**
 * Initialize CDC write timeout
 */
void hashfast_init_udi_cdc_write_timeout() {
    udi_cdc_loops = 0;
    udi_cdc_start = msec_ticker;
}

/**
 * Check if CDC has timedout
 * @return if timeout occurs
 */
bool hashfast_udi_cdc_timeout() {
    if (++udi_cdc_loops >= 1000000) {
        //for(;;);        // XXX trap
    }
    if (elapsed_since(udi_cdc_start) > USB_WRITE_TIMEOUT) {
        return true;
    } else {
        return false;
    }
}

/* TODO this shouln't be down here */
#define TC_CHANNEL  0

/**
 * TC interrupt routine
 */
__attribute__((__interrupt__)) static void tc_irq(void) {
    profileEnter(PROFILE_CHANNEL_TC_ISR);
    /* increment the ms seconds counter */
    msec_tick();
    /*
     * Clear the interrupt flag. This is a side effect of reading the TC SR.
     */
    tc_read_sr(&AVR32_TC, TC_CHANNEL);
    profileExit(PROFILE_CHANNEL_TC_ISR);
}

/**
 * Module initialization
 */
void module_init() {
    volatile avr32_tc_t *tc = &AVR32_TC;

    /**
     * Options for waveform generation
     */
    static const tc_waveform_opt_t WAVEFORM_OPT = {
        .channel = TC_CHANNEL,                          //!< Channel selection.
        .bswtrg = TC_EVT_EFFECT_NOOP,                   //!< Software trigger effect on TIOB.
        .beevt = TC_EVT_EFFECT_NOOP,                    //!< External event effect on TIOB.
        .bcpc = TC_EVT_EFFECT_TOGGLE,                   //!< RC compare effect on TIOB.
        .bcpb = TC_EVT_EFFECT_TOGGLE,                   //!< RB compare effect on TIOB.

        .aswtrg = TC_EVT_EFFECT_NOOP,                   //!< Software trigger effect on TIOA.
        .aeevt = TC_EVT_EFFECT_NOOP,                    //!< External event effect on TIOA.
        .acpc = TC_EVT_EFFECT_TOGGLE,                   //!< RC compare effect on TIOA: toggle.
        .acpa = TC_EVT_EFFECT_TOGGLE,                   //!< RA compare effect on TIOA: toggle (other possibilities are none, set and clear).

        .wavsel = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,   //!< Waveform selection: Up mode with automatic trigger(reset) on RC compare.
        .enetrg = false,                                //!< External event trigger enable.
        .eevt = 0,                                      //!< External event selection.
        .eevtedg = TC_SEL_NO_EDGE,                      //!< External event edge selection.
        .cpcdis = false,                                //!< Counter disable when RC compare.
        .cpcstop = false,                               //!< Counter clock stopped with RC compare.

        .burst = false,                                 //!< Burst signal selection.
        .clki = false,                                  //!< Clock inversion.
        .tcclks = TC_CLOCK_SOURCE_TC3                   //!< Internal source clock 3, connected to fPBA / 8.
    };

    /**
     * Options for TC interrupt
     */
    static const tc_interrupt_t TC_INTERRUPT = {
        .etrgs = 0,
        .ldrbs = 0,
        .ldras = 0,
        .cpcs = 1,
        .cpbs = 0,
        .cpas = 0,
        .lovrs = 0,
        .covfs = 0
    };

    /* configure I/O pins */
    gpio_configure_pin(GOT_DOWN, GPIO_DIR_INPUT);
    gpio_configure_pin(GOT_UP, GPIO_DIR_INPUT);
    gpio_configure_pin(HAVE_USB, GPIO_DIR_OUTPUT);
    gpio_configure_pin(USB_DOWN, GPIO_DIR_INPUT);
    gpio_configure_pin(SPARE_DOWN, GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
    gpio_configure_pin(SPARE_UP, GPIO_DIR_INPUT);
    gpio_configure_pin(A_12V_DETECT, GPIO_DIR_INPUT);
    gpio_configure_pin(B_12V_DETECT, GPIO_DIR_INPUT);
    gpio_configure_pin(GOT_AC_POWER, GPIO_DIR_INPUT);
    gpio_configure_pin(POWER_GOOD, GPIO_DIR_INPUT);
    gpio_configure_pin(POWER_BUTTON, GPIO_DIR_INPUT);
    gpio_configure_pin(RECONFIG_BUTTON, GPIO_DIR_INPUT);

    if (boardid != iraBID) {
        /* pin is hardwired high on ir-a, at least for the moment */
        gpio_configure_pin(ENABLE_IO_VDD, GPIO_DIR_OUTPUT);
    }

    /* LEDs ON initially */
    gpio_configure_pin(POWER_LED, GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
    gpio_configure_pin(ACTIVITY_LED, GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);

#ifdef FEATURE_12V_STATIC_ON
    gpio_configure_pin(PWR_ON, GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
#else /* FEATURE_12V_STATIC_ON */
    gpio_configure_pin(PWR_ON, GPIO_DIR_INPUT);
#endif /* FEATURE_12V_STATIC_ON */

    /** @page evk1101
     * EVK1101 boards require removal of the 32 khz rtc xtal
     */
    gpio_enable_pin_interrupt(POWER_BUTTON, GPIO_FALLING_EDGE);
    gpio_enable_pin_interrupt(RECONFIG_BUTTON, GPIO_FALLING_EDGE);

    INTC_register_interrupt(&gpio_interrupt_handler, AVR32_GPIO_IRQ_0 + (POWER_BUTTON / 8), AVR32_INTC_INT0);
    INTC_register_interrupt(&gpio_interrupt_handler, AVR32_GPIO_IRQ_0 + (RECONFIG_BUTTON / 8), AVR32_INTC_INT0);

    /* initialize the timer/counter waveform */
    tc_init_waveform(tc, &WAVEFORM_OPT);

    /*
     * Set the compare triggers. TC counter is 16-bits.
     *      We want (1/(fPBA/8)) * RC = 0.001 s
     *      therefore RC = (fPBA/8) / 1000 = 7500 to get an interrupt every 1 ms
     */
    tc_write_rc(tc, TC_CHANNEL, (60000000 / 8) / 1000);     // Set RC value.
    tc_write_ra(tc, TC_CHANNEL, 0);                         // Set RA value.
    tc_write_rb(tc, TC_CHANNEL, 1900);                      // Set RB value.

    tc_configure_interrupts(tc, TC_CHANNEL, &TC_INTERRUPT);
    tc_start(tc, TC_CHANNEL);

    INTC_register_interrupt(&tc_irq, AVR32_TC_IRQ0, AVR32_INTC_INT0);
}

/*
 * Module run time handler functions, called in the main loop.
 */

/**
 * Module state
 */
enum module_state_t {
    BS_POWERED_OFF = 0,             //!< BS_POWERED_OFF
    BS_POWERUP_SLAVES,              //!< BS_POWERUP_SLAVES
    BS_STARTUP_SLAVES,              //!< BS_STARTUP_SLAVES
    BS_POWERUP_DELAY,               //!< BS_POWERUP_DELAY
    BS_CHECK_SLAVES,                //!< BS_CHECK_SLAVES
    BS_GET_SLAVE_VERSIONS,          //!< BS_GET_SLAVE_VERSIONS
    BS_CHECK_SLAVE_POWER,           //!< BS_CHECK_SLAVE_POWER
    BS_12V_POWER_IS_UP,             //!< BS_12V_POWER_IS_UP
    BS_BRINGUP_CORE_POWER,          //!< BS_BRINGUP_CORE_POWER
    BS_CORE_POWER_UP,               //!< BS_CORE_POWER_UP
    BS_BOARD_DIAGNOSTICS_COMPLETE,  //!< BS_BOARD_DIAGNOSTICS_COMPLETE
    BS_RUNNING,                     //!< BS_RUNNING
    BS_FAULT,                       //!< BS_FAULT
    BS_POWERDOWN_DELAY              //!< BS_POWERDOWN_DELAY
};

static uint8_t module_state = BS_POWERED_OFF;
static int saved_chain_configuration = CC_UNCONFIGURED;
static uint16_t start_time;
static uint8_t core_power_mask;
uint8_t input_power_good;

/**
 * Check if the input power is good
 * @return
 */
static uint8_t check_input_power_good(void) {
    uint8_t good;
    unsigned int i;
    uint8_t tx;
    uint8_t rx;

    good = 0xff;
    switch (boardid) {
    case iraBID:
        tx = IR3566B_REG_VIN_SUPPLY;
        for (i = 0; i < 4; i++) {
            /*
             * The ir3566b parts appear to be in "mobile" mode. I didn't find
             * anything about this in the IR docs, except that the vin scale is
             * 1/8V in mobile mode and 1/16V otherwise.
             */
#ifdef IRA3566B_MOBILE_MODE
            if ((i == 0 || all_die_settings[i].voltage != DS_DISABLED) && (!twi_sync_rw(TWI_BUS_IR3566B, TWI_IR3566B_STARTADDR + i, &tx, 1, &rx, 1) || rx < 9 * 8))
#else /* IRA3566B_MOBILE_MODE */
            if ((i == 0 || all_die_settings[i].voltage != DS_DISABLED) && (!twi_sync_rw(TWI_BUS_IR3566B, TWI_IR3566B_STARTADDR + i, &tx, 1, &rx, 1) || rx < 9 * 16))
#endif /* IRA3566B_MOBILE_MODE */
                good &= ~((uint8_t) 1 << i);
        }
        break;
    case rev0_1_11_12_15BID:
    default:
        if (gpio_pin_is_low(A_12V_DETECT))
            good &= ~0x01;
        if (gpio_pin_is_low(B_12V_DETECT))
            good &= ~0x02;
        break;
    }
    return good;
}

/**
 * Module functions
 */
static void handleModuleFuncs(void) {
    static uint8_t module;
    static uint8_t die;
    static enum {
        idleMFS,
        waitTWICompleteMFS,
        waitSPICompleteMFS
    } state = idleMFS;
    static twiRequestT req;
    static spiRequestT spiReq;
    static uint8_t txBuffer[4];
    irqflags_t irq;
    uint16_t mv;
    uint16_t dac;

    switch (state) {
    case idleMFS:
        if (++die >= 4) {
            die = 0;
            module++;
        }
        if (module >= ucinfo.num_slaves + 1 || !ucinfo.master)
            module = 0;
        if (moduleFuncs[module].voltageSet[die]) {
            irq = cpu_irq_save();
            moduleFuncs[module].voltageSet[die] = false;
            mv = moduleFuncs[module].millivolts[die];
            cpu_irq_restore(irq);
            if (module == 0) {
                switch (boardid) {
                case unknownBID:
                    break;
                case rev0_1_11_12_15BID:
                    if (mv != DS_DISABLED) {
                        if (mv & DS_DACSETTING)
                            dac = mv & ~DS_DACSETTING;
                        else if (mv >= 1250)
                            dac = 0;
                        else if (mv <= 510)
                            dac = 2047;
                        else
                            dac = (uint16_t) ((1250 - mv) * (uint32_t) 512 / 185);

                        if (dac > 2047)
                            dac = DEFAULT_LINEAR_DAC_SETTING;
#if 1
                        /* TODO
                         * BISON doesn't seem to be working. Needs some
                         * investigation.
                         */
                        usbctrlDebugStreamPrintf("setting dac 0x%04x (mv %u)\n", (unsigned int) dac, (unsigned int) mv);
                        dac_write(true, dac);
#else /* 1 */
                        dac |= 0x3000;
                        txBuffer[0] = dac >> 8;
                        txBuffer[1] = dac & 0xff;
                        spiReq.which = 2;
                        spiReq.txA = txBuffer;
                        spiReq.txB = NULL;
                        spiReq.countA = 2;
                        spiReq.countB = 0;
                        spiReq.rxA = txBuffer;
                        spiReq.rxB = NULL;
                        spiQueue(&spiReq);
                        state = waitSPICompleteMFS;
#endif /* 1 */
                    }
                    break;
                case habaneroBID:
                    break;
                case iraBID:
                    if (mv >= 250 && mv <= 1500) {
                        req.addr = TWI_IR3566B_STARTADDR + die;
                        txBuffer[0] = IR3566B_REG_L1_MANUAL_VID;
                        txBuffer[1] = (mv - 245) / 5;
                        req.tx = txBuffer;
                        req.txLength = 2;
                        req.rx = NULL;
                        req.rxLength = 0;
                        twiQueueRequest(TWI_BUS_IR3566B, &req);
                        state = waitTWICompleteMFS;
                    }
                    break;
                }
            } else {
                req.addr = TWI_SLAVE_STARTADDR + module - 1;
                txBuffer[0] = TWICMD_VOLTAGE_SET;
                txBuffer[1] = die;
                txBuffer[2] = mv >> 8;
                txBuffer[3] = mv & 0xff;
                req.tx = txBuffer;
                req.txLength = 4;
                req.rx = NULL;
                req.rxLength = 0;
                twiQueueRequest(TWI_BUS_UC, &req);
                state = waitTWICompleteMFS;
            }
        }
        break;
    case waitTWICompleteMFS:
        if (!req.pending)
            state = idleMFS;
        break;
    case waitSPICompleteMFS:
        if (!spiReq.pending)
            state = idleMFS;
        break;
    }
}

/**
 * Periodic module task
 */
void module_handler() {
    struct ucinfo_t *info = &ucinfo;
    static uint8_t startup_loops = 0;
    static bool twi_setup = false;
    static bool fpga_is_programmed;
    static bool ir3566b_is_programmed;
    int slave_address;
    bool all_slave_power_good;
    bool all_good;
    bool did_start;
    uint8_t reg_status;
    int i;
    uint8_t tx_buf[1];
    uint8_t rx_buf[16];

    switch (module_state) {
    case BS_POWERED_OFF:
        if (!fpga_is_programmed) {
            fpga_is_programmed = fpga_programmer();
            do_asic_reset = true;
        }

        if (fpga_is_programmed && !ir3566b_is_programmed) {
            switch (boardid) {
            case iraBID:
                ir3566b_is_programmed = ir3566b_programmer();
                break;
            case rev0_1_11_12_15BID:
            default:
                ir3566b_is_programmed = true;
                break;
            }
            do_asic_reset = true;
        }

        /* ensure ASIC is reset */
        if (do_asic_reset) {
            do_asic_reset = false;
            fpga_reg_write(FA_ASIC_CONTROL, F_ASIC_PLL_BYPASS | (info->asic_baud_rate_code & F_ASIC_BAUD_MASK));
        }

        /* continuously assess chaining situation */
        if (gpio_pin_is_high(GOT_DOWN) && gpio_pin_is_high(GOT_UP)) {
            /* no chain connectors */
            info->no_slaves = true;
            info->master = true;
            info->chain_configuration = CC_NONE;
        } else if (gpio_pin_is_low(GOT_DOWN) && gpio_pin_is_low(GOT_UP)) {
            /*
             * We're either:
             *      In the middle of an open chain of modules, or
             *      In a looped back module, for factory test
             * We tell between these combinations because for an open chain the
             * master will assert SPARE_DOWN, so we examine it here.
             */
            if (gpio_pin_is_low(SPARE_UP)) {
                /* propagate this information DOWN */
                gpio_set_pin_low(SPARE_DOWN);
                info->chain_configuration = CC_MIDDLE;
                info->no_slaves = false;
                info->master = false;
            } else {
                /* factory test loopback */
                info->chain_configuration = CC_LOOPBACK;
                info->no_slaves = true;
                info->master = true;
            }

            host_watchdog_active = 0;
            host_watchdog_clock = HOST_WATCHDOG_RECHARGE;

            status_watchdog_active = 0;
            status_watchdog_clock = STATUS_WATCHDOG_RECHARGE;

        } else if (gpio_pin_is_low(GOT_DOWN) && gpio_pin_is_high(GOT_UP)) {
            /* an open chain in the down direction */
            info->no_slaves = false;
            info->master = true;
            /* tell other modules they have a master */
            gpio_set_pin_low(SPARE_DOWN);
            info->chain_configuration = CC_OPEN_DOWN;
        } else {
            /*
             * An open chain in the up direction.
             * GOT_DOWN must be high, GOT_UP must be low.
             */
            info->no_slaves = false;
            info->master = false;
            info->chain_configuration = CC_OPEN_UP;

            host_watchdog_active = 0;
            host_watchdog_clock = HOST_WATCHDOG_RECHARGE;

            status_watchdog_active = 0;
            status_watchdog_clock = STATUS_WATCHDOG_RECHARGE;
        }

        if (startup_loops < 5) {
            /* Take a few loops to establish the configuration. */
            startup_loops++;
            saved_chain_configuration = info->chain_configuration;
        } else {
            if (saved_chain_configuration != CC_UNCONFIGURED && saved_chain_configuration != info->chain_configuration) {
                /*
                 * Someone's playing games with the cables while there is
                 * standby power on. Revert the slave configuration and capture
                 * a new situation.
                 */
                twi_setup = false;
                startup_loops = 0;
                saved_chain_configuration = CC_UNCONFIGURED;
                break;
            }

            if (!info->master && !twi_setup) {
                /* Addressing cycle will get the slave addresses unique. */
                twi_slave_setup(TWI_SLAVE_STARTADDR);
                twi_setup = true;
            }

            if (power_button_pressed) {
                info->fault_code = 0;
                /* Place DAC in floating output mode. */
                dac_write(false, 0);

                if (!fpga_is_programmed) {
                    info->fault_code = E_BOARD_1;
                    module_state = BS_FAULT;
                    usbctrlDebugStreamPrintf("BS_POWERED_OFF > BS_FAULT fault_code %d\n", info->fault_code);
                    break;
                }
                if (!ir3566b_is_programmed) {
                    info->fault_code = E_IR_PROG_FAILURE;
                    module_state = BS_FAULT;
                    usbctrlDebugStreamPrintf("BS_POWERED_OFF > BS_FAULT fault_code %d\n", info->fault_code);
                    break;
                }

                /* Set up enabled die from settings if they are valid. */
                if (hf_nvram_die_settings_valid()) {
                    op_settings_t *s;

                    for (i = 0, s = hf_nvram_die_settings(); i < 4; i++) {
                        if (s->die[i].voltage != 0) {
                            info->die_enable[i] = 1;
                        } else {
                            info->die_enable[i] = 0;
                        }
                    }
                } else {
                    /*
                     * Older boards out there with no die settings,
                     * or retail boards that have not been characterized.
                     */
                    info->die_enable[0] = 1;
                    info->die_enable[1] = 1;
                    info->die_enable[2] = 1;
                    info->die_enable[3] = 1;
                }
                if (info->master) {
                    /*
                     * Attempt to power up. Leave SPARE_DOWN asserted during
                     * this time, to leave no doubt who is in charge.
                     */
#ifdef ASSUME_POWER_GOOD
                    if (1) {
#else
                    if (gpio_pin_is_high(GOT_AC_POWER)) {
#endif
                        gpio_configure_pin(PWR_ON, GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
                        start_time = msec_ticker;
                        module_state = BS_POWERUP_SLAVES;
                        usbctrlDebugStreamPrintf("BS_POWERED_OFF > BS_POWERUP_SLAVES\n");
                    }
                    twi_master_setup();
                } else {
                    /* I'm a slave, and I must have got a TWICMD_STARTUP. */
#ifdef ASSUME_POWER_GOOD
                    if (1) {
#else
                    if (gpio_pin_is_high(GOT_AC_POWER)) {
#endif
                        /* Turn on my attached power supply. */
                        gpio_configure_pin(PWR_ON, GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
                    }
                    start_time = msec_ticker;
                    module_state = BS_POWERUP_DELAY;
                    usbctrlDebugStreamPrintf("BS_POWERED_OFF > BS_POWERUP_DELAY\n");
                }
                power_button_pressed = false;
            }
        }
        break;

    case BS_POWERUP_SLAVES:
        if (info->no_slaves) {
            /* no slaves */
            module_state = BS_POWERUP_DELAY;
            usbctrlDebugStreamPrintf("BS_POWERUP_SLAVES > BS_POWERUP_DELAY\n");
        } else if (elapsed_since(start_time) >= SLAVE_POWERUP_COMMAND_DELAY_MS) {
            /*
             * Give slaves some time to see SPARE_UP being low, and for that to
             * ripple down. Also sequence power supply start ups so they don't
             * both (all) slam on at the same time.
             */
            twi_broadcast(TWICMD_POWERUP, 0);
            module_state = BS_STARTUP_SLAVES;
            usbctrlDebugStreamPrintf("BS_POWERUP_SLAVES > BS_STARTUP_SLAVES\n");
        }
        break;

    case BS_STARTUP_SLAVES:
        /*
         * "Middle" modules may have no active uC until other slave(s) that do
         * have standby power turn their power supplies on. This delay is here
         * to ensure that ALL slave uC's are woken up before we tell them all to
         * actually start up.
         */
        if (elapsed_since(start_time) >= SLAVE_STARTUP_COMMAND_DELAY_MS) {
            twi_broadcast(TWICMD_STARTUP, 0);
            module_state = BS_POWERUP_DELAY;
            usbctrlDebugStreamPrintf("BS_STARTUP_SLAVES > BS_POWERUP_DELAY\n");
        }
        break;

    case BS_POWERUP_DELAY:
        input_power_good = check_input_power_good();
        if ((info->master && elapsed_since(start_time) >= MASTER_POWERUP_DELAY_MS) || (!info->master && elapsed_since(start_time) >= SLAVE_POWERUP_DELAY_MS)) {
            // TODO don't need this any more
            gpio_set_pin_high(SPARE_DOWN);
            info->powered_up = true;

            if (input_power_good != 0xff) {
                /* failure */
                info->fault_code = E_MAIN_POWER_BAD;
                info->fault_extra = input_power_good;
                module_state = BS_FAULT;
                usbctrlDebugStreamPrintf("BS_POWERUP_DELAY > BS_FAULT fault_code %d\n", info->fault_code);
            } else {
                /* power is okay */
                if (info->master) {
                    if (info->no_slaves == false) {
                        module_state = BS_CHECK_SLAVES;
                        usbctrlDebugStreamPrintf("BS_POWERUP_DELAY > BS_CHECK_SLAVES\n");
                    } else {
                        info->physical_die_count = 4;
                        module_state = BS_12V_POWER_IS_UP;
                        usbctrlDebugStreamPrintf("BS_POWERUP_DELAY > BS_12V_POWER_IS_UP\n");
                    }
                } else {
                    module_state = BS_12V_POWER_IS_UP;
                    usbctrlDebugStreamPrintf("BS_POWERUP_DELAY > BS_12V_POWER_IS_UP\n");
                }
            }
        }
        break;

    case BS_CHECK_SLAVES:
        /*
         * We need to do a slave address cycle, to get all the slave addresses
         * set up. We don't know how many slaves there are right now, this
         * figures it out.
         */
        info->num_slaves = 0;
        delay_msec(3);
        for (slave_address = 0; slave_address < MAX_SLAVES; slave_address++) {
            gpio_set_pin_low(SPARE_DOWN);
            twi_broadcast(TWICMD_ADDRESS, TWI_SLAVE_STARTADDR + slave_address);
            info->num_slaves++;
            delay_msec(3);
            /*
             * When the address gets to the end of the chain, the end module
             * pulls HAVE_USB low, and this ripples back to us so we know we
             * just did the last address cycle.
             */
            if (gpio_pin_is_low(USB_DOWN))
                break;
        }
        /* Number of physical die, needed later in asic_handler(). */
        info->physical_die_count = (info->num_slaves + 1) * 4;
        gpio_set_pin_high(SPARE_DOWN);
        if (info->num_slaves == 0) {
            /*
             * No slaves are out there, despite what the cabling says.
             * Cabling error.
             */
            info->no_slaves = true;
            info->fault_code = E_NO_SLAVES;
            module_state = BS_FAULT;
            usbctrlDebugStreamPrintf("BS_CHECK_SLAVES > BS_FAULT fault_code %d\n", info->fault_code);
        } else {
            twi_broadcast(TWICMD_ADDRESSING_COMPLETE, info->num_slaves);
            delay_msec(5);
            module_state = BS_GET_SLAVE_VERSIONS;
            usbctrlDebugStreamPrintf("BS_CHECK_SLAVES > BS_GET_SLAVE_VERSIONS\n");
        }
        break;

    case BS_GET_SLAVE_VERSIONS:
        tx_buf[0] = TWICMD_VERSION;
        for (slave_address = 0; slave_address < info->num_slaves && !info->fault_code; slave_address++) {
            if (twi_sync_rw(TWI_BUS_UC, TWI_SLAVE_STARTADDR + slave_address, tx_buf, 1, rx_buf, 10)) {
                slave_info[slave_address].version = ((uint32_t) rx_buf[0] << 24) | ((uint32_t) rx_buf[1] << 16) | ((uint32_t) rx_buf[2] << 8) | ((uint32_t) rx_buf[3] << 0);
                slave_info[slave_address].crc_valid = rx_buf[4] ? true : false;
                slave_info[slave_address].crc = ((uint32_t) rx_buf[5] << 24) | ((uint32_t) rx_buf[6] << 16) | ((uint32_t) rx_buf[7] << 8) | ((uint32_t) rx_buf[8] << 0);
                slave_info[slave_address].type = (boardidT) rx_buf[9];
            } else {
                info->fault_code = E_SLAVE_COMM;
                module_state = BS_FAULT;
                usbctrlDebugStreamPrintf("BS_GET_SLAVE_VERSIONS > BS_FAULT fault_code %d\n", info->fault_code);
            }
        }
        tx_buf[0] = TWICMD_SERIAL_NUMBER;
        for (slave_address = 0; slave_address < info->num_slaves && !info->fault_code; slave_address++) {
            if (!twi_sync_rw(TWI_BUS_UC, TWI_SLAVE_STARTADDR + slave_address, tx_buf, 1, (uint8_t *) &slave_info[slave_address].serial, sizeof(slave_info[slave_address].serial))) {
                info->fault_code = E_SLAVE_COMM;
                module_state = BS_FAULT;
                usbctrlDebugStreamPrintf("BS_GET_SLAVE_VERSIONS > BS_FAULT fault_code %d\n", info->fault_code);
            }
        }

        if (!info->fault_code)
            module_state = BS_CHECK_SLAVE_POWER;
        usbctrlDebugStreamPrintf("BS_GET_SALVE_VERSIONS > BS_CHECK_SLAVE_POWER\n");
        break;

    case BS_CHECK_SLAVE_POWER:
        all_slave_power_good = true;
        for (slave_address = 0; slave_address < info->num_slaves; slave_address++) {
            if (twi_get_slave_data(TWI_SLAVE_STARTADDR + slave_address, TWICMD_POWER_STATUS, rx_buf, 1) == true) {
                if (rx_buf[0] == 0x3 || rx_buf[0] == 0xff) {
                    /* TODO
                     * Slave has all 12V supplies up - all good.
                     * Check for 3 is for backwards compatibility, could go away
                     * if all older firmware is gone.
                     */
                } else {
                    uprintf(UD_STARTUP, "Slave %d power sts 0x%02x\n", slave_address, rx_buf[0]);
                    all_slave_power_good = false;
                    break;
                }
            } else {
                info->fault_code = E_SLAVE_COMM;
                module_state = BS_FAULT;
                usbctrlDebugStreamPrintf("BS_CHECK_SLAVE_POWER > BS_FAULT fault_code %d\n", info->fault_code);
            }
        }
        if (!info->fault_code) {
            if (all_slave_power_good == true) {
                hf_nvram_get_slave_die_settings();
                module_state = BS_12V_POWER_IS_UP;
                usbctrlDebugStreamPrintf("BS_CHECK_SLAVE_POWER > BS_12V_POWER_IS_UP\n");
            } else {
                info->fault_code = E_SECONDARY_POWER_BAD;
                module_state = BS_FAULT;
                usbctrlDebugStreamPrintf("BS_CHECK_SLAVE_POWER > BS_FAULT fault_code %d\n", info->fault_code);
            }
        }
        break;

/* TODO this should not be burried here */
#define FEATURE_RETAIN_FIXED_REV1_PLAIN_BAUD_RATES

    case BS_12V_POWER_IS_UP:
        /*
         * Power is up.
         * If I'm a slave, wait for master to finish addressing.
         */
        if (!info->master && !info->addressing_complete)
            break;
        core_power_mask = 0x0;

#ifdef FEATURE_RETAIN_FIXED_REV1_PLAIN_BAUD_RATES
        if (info->master && !info->mixed_reference_clocks && info->dynamic_baud_rate && hf_nvram_die_settings_valid() && hf_nvram_die_settings()->ref_frequency == DEFAULT_REF_CLOCK) {
            /*
             * Operate 125 Mhz Rev-0/1 equipment that is already in the field in
             * exactly the same way it was first tested. Don't try to use
             * dynamic baud rate settings.
             */
            info->dynamic_baud_rate = 0;
            info->asic_baud_rate_code = 7;
            uart_set_baudrate(BAUD_RATE_PWRUP_7);
            /* Ensure all ASIC's everywhere are correctly dealt with. */
            fpga_reg_write(FA_ASIC_CONTROL, F_ASIC_PLL_BYPASS | (info->asic_baud_rate_code & F_ASIC_BAUD_MASK));
            if (info->num_slaves)
                twi_broadcast(TWICMD_FPGA_ASIC_CTL, F_ASIC_PLL_BYPASS | (info->asic_baud_rate_code & F_ASIC_BAUD_MASK) | F_FORCE_BAUD);
            uprintf(UD_STARTUP, "Using B/C FIXED baud rates\n");
            delay_msec(5);
            break;
        }
#endif /* FEATURE_RETAIN_FIXED_REV1_PLAIN_BAUD_RATES */

        /* Run board diagnostics. */
        if (board_diagnostics()) {
            gpio_set_pin_high(ENABLE_IO_VDD);
            delay_msec(5);
            set_board_voltage();
            start_time = msec_ticker;
            module_state = BS_BRINGUP_CORE_POWER;
            usbctrlDebugStreamPrintf("BS_12V_POWER_IS_UP > BS_BRINGUP_CORE_POWER\n");
        }
        break;

    case BS_BRINGUP_CORE_POWER:
        if (hf_spi_read_block(1, FA_REG_STATUS, &reg_status, 1) == STATUS_OK) {
            did_start = false;
            for (i = 0; i < 4; i++) {
                if (info->die_enable[i] && !(reg_status & (1 << i))) {
                    did_start = true;
                    core_power_mask |= (1 << i);
                    fpga_reg_write(FA_REG_ENABLE, core_power_mask);
                    break;
                }
            }
            if (did_start)
                break;

            /* check supply status */
            for (i = 0, all_good = true; i < 4; i++) {
                /* check pgood */
                if (info->die_enable[i] && !(reg_status & (1 << (i + 4)))) {
                    all_good = false;
                    break;
                }
            }
            if (all_good) {
                module_state = BS_CORE_POWER_UP;
                usbctrlDebugStreamPrintf("BS_BRINGUP_CORE_POWER > BS_CORE_POWER_UP\n");
            } else if (elapsed_since(start_time) > 50) {
                info->fault_code = E_CORE_POWER_FAULT;
                info->fault_extra = reg_status;
                module_state = BS_FAULT;
                usbctrlDebugStreamPrintf("BS_BRINGUP_CORE_POWER > BS_FAULT fault_code %d\n", info->fault_code);
            }
        } else {
            /* Disable any supplies that are on. */
            fpga_reg_write(FA_REG_ENABLE, 0);
            /* FPGA SPI Read failed */
            info->fault_code = E_BOARD_2;
            break;
        }
        break;

    case BS_CORE_POWER_UP:
        module_state = BS_BOARD_DIAGNOSTICS_COMPLETE;
        usbctrlDebugStreamPrintf("BS_CORE_POWER > BS_BOARD_DIAGNOSTICS_COMPLETE\n");
        break;

    case BS_BOARD_DIAGNOSTICS_COMPLETE:
        if (power_up_only == true) {
            udi_cdc_write_buf((uint8_t *) ucinfo.usb_init_header, sizeof(struct hf_header));
            /* Just an OP_POWER, get rid of it. */
            ucinfo.usb_init_header = NULL;
        } else {
            /*
             * This turns on the ASIC Initialization in asic_handler.c
             */
            info->board_initialized = true;
            usbctrlDebugStreamPrintf("BS_BOARD_DIAGNOSTICS_COMPLETE: info->board_initialized = true\n");
        }
        /* That has, up until now, been the starting point. */
        module_state = BS_RUNNING;
        usbctrlDebugStreamPrintf("BS_BOARD_DIAGNOSTICS_COMPLETE > BS_RUNNING\n");
        break;

    case BS_RUNNING:
        if (power_down_request == true) {
            /* shut down everything */
            if (info->master && !info->no_slaves) {
                //twi_broadcast(TWICMD_POWERDOWN, 0);
                twi_broadcast(TWICMD_REBOOT, 0);
                delay_msec(10);
            }
            gpio_configure_pin(PWR_ON, GPIO_DIR_INPUT);
            info->board_initialized = false;
            info->num_slaves = 0;
            start_time = msec_ticker;
            module_state = BS_POWERDOWN_DELAY;
            usbctrlDebugStreamPrintf("BS_RUNNING > BS_POWERDOWN_DELAY powerdown_request\n");
        } else {
            handleModuleFuncs();
        }
        break;

    /*
     * A fault occurred during bringup.
     */
    case BS_FAULT:
        /* Disable any supplies that are on. */
        fpga_reg_write(FA_REG_ENABLE, 0);    //
        if (info->master && !info->no_slaves) {
            //twi_broadcast(TWICMD_POWERDOWN, 0);
            twi_broadcast(TWICMD_REBOOT, 0);
            delay_msec(2);
        }
        /* power down */
        gpio_configure_pin(PWR_ON, GPIO_DIR_INPUT);
        /* so asic_handler() generates a reply to OP_USB_INIT */
        info->board_initialized = true;
        start_time = msec_ticker;
        module_state = BS_POWERDOWN_DELAY;
        usbctrlDebugStreamPrintf("BS_FAULT > BS_POWERDOWN_DELAY\n");
        break;

    case BS_POWERDOWN_DELAY:
        /* So the OP_USB_INIT reply can get through. */
        if (elapsed_since(start_time) > 200) {
            /* TODO
             * When we get reset.
             * If we're master, do a system reset.
             * Make sure core power is off
             */
            fpga_reg_write(FA_REG_ENABLE, 0);
            delay_msec(5);
            twi_setup = false;
            startup_loops = 0;
            saved_chain_configuration = CC_UNCONFIGURED;
            info->master = false;

            usbctrlDebugStreamPrintf("BS_POWERDOWN_DELAY > self_reset()\n");
            /* never return */
            self_reset();
        }
        break;

    default:
        module_state = BS_RUNNING;
        usbctrlDebugStreamPrintf("BS_UNKNOWN > BS_RUNNING\n");
        break;
    }
}

/**
 * Called if we're a slave, we have standby power, and the master told us to
 * power up.
 */
void power_supply_on() {
    gpio_configure_pin(PWR_ON, GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
}

/**
 * Called when an OP_USB_INIT happens (could be just on standby or VBUS power).
 * Allowed to be called by ISR.
 */
void usb_powerup_request() {
    power_button_pressed = true;
}

static int pdr;

/**
 * Powerdown request.
 * allowed to be called by ISR.
 */
void usb_powerdown_request() {
    pdr++;
    /*
     * The state machine doesn't pay attention to power_down_request if we're
     * already off. A consequence of that is that a powerdown request made while
     * we're off will have a delayed effect, after the next power up. That's
     * rather annoying, so we'll ignore powerdown requests when we're already
     * off.
     * Maybe we should instead add a check for power_down_request in the
     * BS_POWERED_OFF state so we can - after a bit of a delay - reboot just
     * like if we had been on.
     */
    if (module_state != BS_POWERED_OFF || power_button_pressed)
        power_down_request = true;
}

/**
 * Is the system powering down
 * @return
 */
bool system_powering_down() {
    return (power_down_request == true);
}

/**
 * Is the system off
 * @return
 */
bool system_off() {
    return (module_state == BS_POWERED_OFF);
}

/**
 * Is the system on
 * @return
 */
bool system_on() {
    return (module_state == BS_RUNNING);
}

/*
 * Board diagnostics
 */

/**
 * Board Diagnostic State
 */
enum board_diagnostic_state_t {
    BD_IDLE = 0,        //!< BD_IDLE
    BD_TEST_FPGA,       //!< BD_TEST_FPGA
    BD_FANS,            //!< BD_FANS
    BD_SENSORS,         //!< BD_SENSORS
    BD_DONE             //!< BD_DONE
};

static uint8_t bd_state = BD_IDLE;

/**
 * Run board level diagnostics.
 * Called continuously while diagnostics are running, until done.
 * @return
 */
static bool board_diagnostics() {
    struct ucinfo_t *info = &ucinfo;
    bool done = false;

    switch (bd_state) {
    case BD_IDLE:
        bd_state = BD_TEST_FPGA;
        break;

    /*
     * Check that the FPGA seems OK, and set up initial registers and routing.
     */
    case BD_TEST_FPGA:
        fpga_diagnostics();
        if (info->fault_code) {
            done = true;
            bd_state = BD_IDLE;
        } else {
            bd_state = BD_FANS;
        }
        break;

    /*
     * Check fan(s) if connected.
     */
    case BD_FANS:
        // TODO not implemented yet
        //if (fan_diagnostics()) {}
        bd_state = BD_SENSORS;
        break;

    /*
     * Check sensors are all sensible.
     */
    case BD_SENSORS:
        // TODO not implemented yet
        //if (sensor_diagnostics()) {}
        bd_state = BD_DONE;
        break;

    case BD_DONE:
        done = true;
        break;

    default:
        break;
    }

    return (done);
}

/**
 * Run diagnostics on the FPGA, and do an initial register setup.
 */
static void fpga_diagnostics() {
    struct ucinfo_t *info = &ucinfo;
    status_code_t sts;
    uint8_t buf[16];
    int i;

    if ((sts = hf_spi_read_block(1, FA_MAGIC, buf, 3)) == STATUS_OK) {
        if (buf[0] == F_MAGIC) {
            /* got the right magic number */
            info->switches = buf[2];
            /* try a write-read test */
            for (i = 0; i < 8; i++) {
                /* 4 R/W registers available here */
                buf[0] = FA_OC_REFERENCE;
                buf[1] = 0xaa;
                buf[2] = 0x55;
                buf[3] = i + 42;
                buf[4] = i + 43;
                if (hf_spi_write_read(1, buf, 5, 0, 0) == STATUS_OK) {
                    /* read back */
                    if (hf_spi_read_block(1, FA_OC_REFERENCE, buf, 4) == STATUS_OK) {
                        if (!(buf[0] == 0xaa && buf[1] == 0x55 && buf[2] == (i + 42) && buf[3] == (i + 43))) {
                            /* FPGA read/write test mismatch */
                            info->fault_code = E_BOARD_5;
                            break;
                        }
                    } else {
                        /* FPGA SPI read failed */
                        info->fault_code = E_BOARD_2;
                        break;
                    }
                } else {
                    /* FPGA SPI write failure */
                    info->fault_code = E_BOARD_4;
                    break;
                }
            }
        } else {
            /* Bad FPGA magic number */
            info->fault_code = E_BOARD_3;
        }
    } else {
        /* FPGA SPI read failed */
        info->fault_code = E_BOARD_2;
    }

    /* return if FPGA error */
    if (info->fault_code)
        return;

    /*
     * Here for a good FPGA. Do the initial programming.
     * Reset the ASIC.
     */

    fpga_reg_write(FA_CONFIG, F_BIG_ENDIAN);
    fpga_reg_write(FA_REG_STATUS, 0);

    fpga_route_setup(false);

    return;
}

/**
 * Set up serial link routing, for a single die.
 * @param single_die
 */
void fpga_route_setup_single_die(int single_die) {
    struct ucinfo_t *info = &ucinfo;
    uint8_t buf[8];
    status_code_t sts;
    bool done;
    bool no_die_enabled;
    int i, j;
    bool force_local = true;

    no_die_enabled = false;

    if (info->master || force_local) {
        /* set up micro-controller input */
        buf[0] = FA_ROUTE_UC_SIN;
        if (info->no_slaves || force_local) {
            /* route from last enabled die */
            for (i = 3; i >= 0; i--) {
                if (info->die_enable[i] && (i == single_die))
                    break;
            }
            buf[1] = ROUTE_FROM_DIE0_SOUT + i;
        }
        sts = hf_spi_write_read(1, buf, 2, NULL, 0);

        /* set up each die */
        for (i = 3, done = false; i >= 0 && !done; i--) {
            buf[0] = FA_ROUTE_DIE0_SIN + i;
            for (j = i - 1; j >= 0; j--)
                if (info->die_enable[j] && (i == single_die))
                    break;
            if (j >= 0) {
                /* found an enabled die */
                buf[1] = ROUTE_FROM_DIE0_SOUT + j;
            } else {
                /* no previous active, route from micro-controller */
                buf[1] = ROUTE_FROM_UC_SOUT;
                done = true;
            }
            if (!no_die_enabled) {
                sts = hf_spi_write_read(1, buf, 2, NULL, 0);
            }
        }
    }
}

/**
 * Set up serial link routing, according to
 *      - Whether we're a master or a slave
 *      - Whether we're alone, the up-est, middle or down-est module
 *      - Which info->die_enable[] are turned on
 * @param force_local
 */
void fpga_route_setup(bool force_local) {
    struct ucinfo_t *info = &ucinfo;
    uint8_t buf[8];
    status_code_t sts;
    bool done;
    bool no_die_enabled;
    int i, j;

    no_die_enabled = false;

    if (info->master || force_local) {
        /* set up micro-controller input */
        buf[0] = FA_ROUTE_UC_SIN;
        if (info->no_slaves || force_local) {
            /* route from last enabled die */
            for (i = 3; i >= 0; i--) {
                if (info->die_enable[i])
                    break;
            }
            buf[1] = ROUTE_FROM_DIE0_SOUT + i;
        } else {
            /* route from chain-up link */
            buf[1] = ROUTE_FROM_DOWN_SOUT;
        }
        sts = hf_spi_write_read(1, buf, 2, NULL, 0);

        /* set up down-sout if necessary */
        if (!force_local && !info->no_slaves) {
            buf[0] = FA_ROUTE_DOWN_SIN;
            for (i = 3; i >= 0; i--)
                if (info->die_enable[i])
                    break;
            if (i >= 0) {
                /* route from last enabled die to down-sout */
                buf[1] = ROUTE_FROM_DIE0_SOUT + i;
            } else {
                /*
                 * A master with no die enabled.
                 * Route from uC sout.
                 */
                no_die_enabled = true;
                buf[1] = ROUTE_FROM_UC_SOUT;
            }
            hf_spi_write_read(1, buf, 2, NULL, 0);
        }

        /* set up each die */
        for (i = 3, done = false; i >= 0 && !done; i--) {
            buf[0] = FA_ROUTE_DIE0_SIN + i;
            for (j = i - 1; j >= 0; j--)
                if (info->die_enable[j])
                    break;
            if (j >= 0) {
                /* found an enabled die */
                buf[1] = ROUTE_FROM_DIE0_SOUT + j;
            } else {
                /* no previous active, route from micro-controller */
                buf[1] = ROUTE_FROM_UC_SOUT;
                done = true;
            }
            if (!no_die_enabled) {
                sts = hf_spi_write_read(1, buf, 2, NULL, 0);
            }
        }
    } else {
        /* slave, either a CC_MIDDLE or CC_OPEN_DOWN */
        buf[0] = FA_ROUTE_UP_SIN;
        if (info->chain_configuration == CC_MIDDLE) {
            /* return path wire from DOWN to UP */
            buf[1] = ROUTE_FROM_DOWN_SOUT;
            hf_spi_write_read(1, buf, 2, NULL, 0);

            buf[0] = FA_ROUTE_DOWN_SIN;
            /* return path to DOWN comes from last enabled die */
            for (i = 3; i >= 0; i--)
                if (info->die_enable[i])
                    break;
            if (i >= 0) {
                /* route from last enabled die */
                buf[1] = ROUTE_FROM_DIE0_SOUT + i;
            } else {
                /* no die enabled, route from up-sin */
                no_die_enabled = true;
                buf[1] = ROUTE_FROM_UP_SOUT;
            }
            hf_spi_write_read(1, buf, 2, NULL, 0);

        } else {
            /* return path to UP comes from last enabled die */
            for (i = 3; i >= 0; i--)
                if (info->die_enable[i])
                    break;
            if (i >= 0) {
                /* route from last enabled die */
                buf[1] = ROUTE_FROM_DIE0_SOUT + i;
            } else {
                /* no die enabled, route from up-sin */
                no_die_enabled = true;
                buf[1] = ROUTE_FROM_UP_SOUT;
            }
            hf_spi_write_read(1, buf, 2, NULL, 0);
        }

        /* now wire up the chain of die */
        for (i = 3, done = false; i >= 0 && !done; i--) {
            buf[0] = FA_ROUTE_DIE0_SIN + i;
            for (j = i - 1; j >= 0; j--)
                if (info->die_enable[j])
                    break;
            if (j >= 0) {
                /* found an enabled die */
                buf[1] = ROUTE_FROM_DIE0_SOUT + j;
            } else {
                /* no previous active, route from up_sout */
                buf[1] = ROUTE_FROM_UP_SOUT;
                done = true;
            }
            hf_spi_write_read(1, buf, 2, NULL, 0);
        }
    }
}

/**
 * Write to an FPGA byte wide register.
 * @param addr
 * @param value
 */
void fpga_reg_write(uint8_t addr, uint8_t value) {
    uint8_t buf[2];

    buf[0] = addr;
    buf[1] = value;

    hf_spi_write_read(1, buf, 2, NULL, 0);
}

/**
 * Linear boards only.
 */
static void set_board_voltage() {
    int i;
    uint16_t v, dac;

    switch (boardid) {
    case rev0_1_11_12_15BID:
        /*
         * Only one DAC setting per 4 die on Linear boards.
         * Use the first non-disabled die voltage setting we can find.
         */
        for (i = 0; i < 4; i++) {
            if ((v = all_die_settings[i].voltage) != DS_DISABLED)
                break;
        }

        if (v == DS_DISABLED) {
            /* All die disabled! */
            dac = DEFAULT_LINEAR_DAC_SETTING;
        } else if (v & DS_DACSETTING) {
            /* Direct DAC setting by user. */
            dac = v & ~DS_DACSETTING;
        } else {
            /*
             * A voltage setting, given in mV. We have to derive the DAC setting.
             *      DAC = 1024 corresponds to 0.88 volts bulk (on the board).
             *      Each 512 bits above that lowers voltage by 185 mV
             *      Each 512 bits below that raises voltage by 185 mV
             * We allow 50 mV difference between bulk and core voltage, that's
             * 138 bits.
             */
            if (v > 1200) {
                dac = 0;
            } else {
                dac = (uint16_t) (((uint32_t) (1250 - v) * (uint32_t) 512) / 185);
                if (dac > 2047) {
                    dac = DEFAULT_LINEAR_DAC_SETTING;
                }
            }
        }
        //notify_host("set_board_voltage: die %d nvram 0x%04x (%d), DAC %04x (%d)", i, v, v & ~DS_DACSETTING, dac, dac);
        dac_write(true, dac);
        break;

    default:
        break;
    }
}

/**
 * Module type for given module.
 * Allowed to be called by ISR.
 * @param module
 * @return type
 */
boardidT module_type(uint8_t module) {
    boardidT type;

    if (module == 0) {
        type = boardid;
    } else if (module <= MAX_SLAVES) {
        type = slave_info[module - 1].type;
    } else {
        type = unknownBID;
    }
    return type;
}

/**
 * Module serial for given module.
 * Allowed to be called by ISR.
 * @param module
 * @param serial
 */
void module_serial(uint8_t module, serial_number_t *serial) {

    if (module == 0) {
        hf_nvram_get_serial(serial);
    } else if (module <= MAX_SLAVES) {
        memcpy(serial, &slave_info[module - 1].serial, sizeof(serial_number_t));
    }
}

/**
 * Module version for given module.
 * Allowed to be called by ISR.
 * @param module
 * @return version
 */
uint32_t fw_version(uint8_t module) {
    uint32_t version;

    if (module == 0) {
        version = FIRMWARE_VERSION;
    } else if (module <= MAX_SLAVES) {
        version = slave_info[module - 1].version;
    } else {
        version = 0;
    }
    return version;
}

/**
 * Module CRC for given module.
 * Allowed to be called by ISR.
 * @param module
 * @param crc
 * @return
 */
bool fw_crc(uint8_t module, uint32_t *crc) {
    hfLoaderAppSuffixT *suffix;
    bool valid;

    if (module == 0) {
        suffix = (hfLoaderAppSuffixT *) (AVR32_FLASH + AVR32_FLASH_SIZE - sizeof(hfLoaderAppSuffixT));
        valid = (suffix->magic == HF_LOADER_SUFFIX_MAGIC) ? true : false;
        if (valid && crc) {
            *crc = suffix->crc;
        }
    } else if (module < MAX_SLAVES) {
        valid = slave_info[module - 1].crc_valid;
        if (crc) {
            *crc = slave_info[module - 1].crc;
        }
    } else {
        valid = false;
    }

    return valid;
}

/**
 * Set module voltage for given module.
 * Allowed to be called by ISR.
 * @param module
 * @param die
 * @param millivolts
 */
void module_voltage_set(uint8_t module, uint8_t die, uint16_t millivolts) {
    if (module < sizeof(moduleFuncs) / sizeof(moduleFuncs[0]) && die < sizeof(moduleFuncs[module].millivolts) / sizeof(moduleFuncs[module].millivolts[0])) {
        moduleFuncs[module].millivolts[die] = millivolts;
        moduleFuncs[module].voltageSet[die] = true;
    }
}
