
#include "main.h"
//#include <util/delay.h>

static void chain_address(void);
static void chain_config(void);
static uint8_t chain_setup_groups(void);
#if 0
static void chain_clockgate_all(uint8_t, bool, uint16_t *);
static void single_clockgate(uint8_t, uint8_t, bool);
#endif
static uint8_t send_hash(uint8_t, uint8_t, const struct hf_hash_serial *, bool, bool, uint8_t);
static void send_abort(uint8_t, uint8_t, uint16_t);
static int8_t local_receive(uint8_t, struct hf_header **, uint16_t);
static int16_t chain_test_cores(void);
static void asic_unreset(void);
void chain_init(uint8_t);
static uint16_t chain_setup_pll(bool, uint16_t, uint32_t, bool);
static void chain_usb_init_reply(void);
static void system_init(void);
static void hash_clocks_init(uint16_t);
static void hash_clocks_adjust(uint16_t);
static void set_dynamic_nonce_ranges(void);
static void chain_baud(void);
#ifdef FEATURE_CHARACTERIZATION
static uint8_t characterization_test(void);
#endif


// This is for diagnostic use only right now so I'm not putting it in main.h
//#define FEATURE_TURN_ON_GPO_LEDS

#ifdef FEATURE_TURN_ON_GPO_LEDS
static void chain_gpo_leds_on(void);
#endif


enum chain_state_t {
    CS_IDLE = 0,
    CS_UNRESET_DELAY,
    CS_RESET_SENT,
    CS_RESET_DELAY,
    CS_ADDRESS_SENT,
    CS_BAUD_SENT,
    CS_MIXED_BAUD,
    CS_VERIFY_CHAIN,
    CS_CLOCKGATE_ALL_OFF,
    CS_CORE_TEST_LOW_SPEED,
    CS_SETUP_PLL,
    CS_WAIT_PLL,
    CS_CORE_TEST_ATSPEED,
    CS_CONFIG_SENT,
    CS_SETUP_GROUPS,
    CS_WAIT_CLEAR,
    CS_SHUTTING_DOWN,
#ifdef FEATURE_CHARACTERIZATION
    CS_CHARACTERIZE_DONE,
    CS_CHARACTERIZE_LOOP,
#endif
    CS_ALL_DONE
    };

#define REPLY_TIMEOUT_MS        20

#define RESET_WAIT_TIME         10                 // Time to wait after OP_RESET
#define MAX_RETRIES             5

// The state of all clock gates
//static uint16_t clockgate_state[MAX_CORES/16] __attribute__((aligned(2)));

static uint16_t start_time;
static uint16_t elapsed_time;

static uint8_t chain_state = CS_IDLE;
static uint8_t retries;
static uint8_t try_forever = 0;
static bool    self_shutdown_request = false;

// Variables that effect how chain_test_cores() behaves
static bool    ct_while_hashing = false;
static uint8_t ct_core_first;
static uint8_t ct_core_last;
static uint8_t ct_die_first;
static uint8_t ct_die_last;
static bool    ct_characterize = false;

// Clock rates per die
uint16_t initial_hash_clock_settings[MAX_DIE];          // Initial hash clock settings, from last OP_USB_INIT
uint32_t dynamic_hash_clock_settings[MAX_DIE];          // Dynamic hash clock settings, can be adjusted during work restarts
uint32_t dynamic_nonce_range[MAX_DIE] = {0};            // Dynamic nonce ranges, to balance work times across die with different clocks

extern bool real_powerbutton_pressed;

// Keep track of startup times for development purposes
static struct init_times {
    uint16_t start_time;
    uint16_t reset_retries;
    uint16_t reset_back;
    uint16_t reset_delay;
    uint16_t address_retries;
    uint16_t address_back;
    uint16_t baud_retries;
    uint16_t sent_clockgate_off;
    uint16_t got_clockgate_back;
    uint16_t clockgate_retries;
    uint16_t setup_pll_start;
    uint16_t got_config_back;
    uint16_t config_retries;
    uint16_t all_done;
    } init_time;

void chain_handler()
    {
    struct hf_header *h;
    struct ucinfo_t *info = &ucinfo;
    struct hf_usb_init_header *hu = ucinfo.usb_init_header;
    int16_t good_cores;
    int8_t sts;

    switch (chain_state)
        {
        case CS_IDLE:
            retries = 0;
            h = (struct hf_header *)info->usb_init_header;
            if (h)
                {
                if (h->operation_code == OP_USB_INIT && info->board_initialized)
                    {
                    // We've been asked by the host to initialize.
                    uprintf(UD_STARTUP, "CS_IDLE:  OP_USB_INIT\n");
                    memset(&init_time, 0, sizeof(init_time));
                    init_time.start_time = msec_ticker;
                    uprintf(UD_STARTUP, "CS_IDLE: OP_USB_INIT %.3f secs\n", (float)msec_ticker / 1000.0);
                    if (info->fault_code)
                        {
                        // The board/box didn't initialize properly.
                        chain_state = CS_ALL_DONE;
                        }
                    else
                        {
                        hu->hash_clock = le16_to_cpu(hu->hash_clock);
                        system_init();
                        gwq_reinit();
                        if (info->dynamic_baud_rate)
                            uart_set_default_baudrate();
                        hash_clocks_init(hu->hash_clock);
                        asic_unreset();
                        chain_state = CS_UNRESET_DELAY;
                        }
                    }
                else if (h->operation_code == OP_USB_SHUTDOWN)
                    {
                    // Power down cores
                    fpga_reg_write(FA_REG_ENABLE, 0);       // Turn core power off
                    start_time = msec_ticker;
                    //chain_init(0);
                    chain_state = CS_SHUTTING_DOWN;
                    }
#ifdef FEATURE_CHARACTERIZATION
                else if (h->operation_code == OP_CHARACTERIZE && info->board_initialized)
                    {
                    // Launch a characterization run now that the module hardware is up and running
                    memset(&init_time, 0, sizeof(init_time));
                    init_time.start_time = msec_ticker;
                    if (info->fault_code)
                        {
                        // The board/box didn't initialize properly.
                        chain_state = CS_CHARACTERIZE_DONE;
                        }
                    else
                        {
                        system_init();
                        gwq_reinit();
                        if (info->dynamic_baud_rate)
                            uart_set_default_baudrate();
                        asic_unreset();
                        chain_state = CS_UNRESET_DELAY;
                        }
                    }
#endif
                }
            else if (self_shutdown_request)
                {
                fpga_reg_write(FA_REG_ENABLE, 0);           // Turn core power off
                start_time = msec_ticker;
                //chain_init(0);
                chain_state = CS_SHUTTING_DOWN;
                }
            else if (real_powerbutton_pressed && info->board_initialized && info->factory_mode)
                {
                // Power button must have been pressed, on an board under test at the factory.
                // We allow a no-host mode, which can produce a go / no-go test as indicated by the LEDs
                //
                // So place the board on a current limited power supply.
                // Press the power button.
                //
                // Self tests run, and "GREEN" means that over MINIMUM_ACCEPTABLE_CORES_PER_DIE are good, and
                // "No light" means there is either a board or ASIC fault, or less than MINIMUM_ACCEPTABLE_CORES_PER_DIE
                // on average are good.
                //
                // Someone can change this to do some LED flashing things I guess.
                //
                // One time test, power down is necessary to initiate again.
                //
                // Fake out an initialization block
                real_powerbutton_pressed = false;
                info->fake_standalone_test = true;
                memset(&init_time, 0, sizeof(init_time));
                init_time.start_time = msec_ticker;
                if (info->fault_code)
                    {
                    // The board/box didn't initialize properly.
                    chain_state = CS_ALL_DONE;

                    POWER_LED_OFF
                    ACTIVITY_LED_ON
                    activity_led_counter = 0xffff;
                    led_mode = LED_AUTO;

                    }
                else
                    {
                    system_init();
                    gwq_reinit();
                    if (info->dynamic_baud_rate)
                        uart_set_default_baudrate();
                    asic_unreset();
                    chain_state = CS_UNRESET_DELAY;
                    }
                }
            break;

        case CS_UNRESET_DELAY:
            // Give reset in the slaves (via the twi link) a chance to happen
            if (elapsed_since(init_time.start_time) > 10)
                {
                chain_init(1);
                chain_state = CS_RESET_SENT;
                }
            break;

        case CS_RESET_SENT:
            if (asic_get_receive_count())
                {
                h = (struct hf_header *)asic_get_receive();
                if (h->operation_code == OP_RESET)
                    {
                    uprintf(UD_STARTUP, "Got reset back, %d retries\n", retries);
                    retries = 0;
                    init_time.reset_back = elapsed_since(init_time.start_time);
                    chain_state = CS_RESET_DELAY;
                    }
                asic_pop_receive();
                }
            else if ((elapsed_time = elapsed_since(start_time)) > REPLY_TIMEOUT_MS)
                {
                if (!try_forever && ++retries > MAX_RETRIES)
                    {
                    info->fault_code = E_RESET_TIMEOUT;
                    chain_state = CS_ALL_DONE;
                    }
                else
                    {
                    init_time.reset_retries++;
                    chain_init(1);
                    }
                }
            break;

        case CS_RESET_DELAY:
            if ((elapsed_time = elapsed_since(start_time)) > RESET_WAIT_TIME)
                {
                if (info->usb_init_header->operation_code == OP_USB_SHUTDOWN)
                    {
                    asic_enable_host();
                    chain_state = CS_IDLE;
                    info->usb_init_header = NULL;
                    }
                else
                    {
                    init_time.reset_delay = elapsed_since(init_time.start_time);
                    chain_address();
                    chain_state = CS_ADDRESS_SENT;
                    }
                }
            break;

        case CS_ADDRESS_SENT:
            if (asic_get_receive_count())
                {
                h = (struct hf_header *)asic_get_receive();
                hu = ucinfo.usb_init_header;
                if (h->operation_code == OP_ADDRESS)
                    {
                    init_time.address_back = elapsed_since(init_time.start_time);
                    // Rip information out of the returned packet
                    info->die_count = h->chip_address;
                    info->core_count = h->core_address;
                    info->device_type = (uint8_t)le16_to_cpu(h->hdata);

                    info->ref_frequency = 0;
                    if (hf_nvram_die_settings_valid())
                        {
                        // This is for the MASTER. Slaves could be different, unfortunately.
                        info->ref_frequency = hf_nvram_die_settings()->ref_frequency;
                        }
                    if (!info->ref_frequency)
                        {
                        info->ref_frequency = (uint8_t)((le16_to_cpu(h->hdata))>>8);    // Believe the ASIC
                        }

                    uprintf(UD_STARTUP, "Got address back, %d die %d cores, valid %d ref %d\n", h->chip_address, h->core_address, hf_nvram_die_settings_valid(), info->ref_frequency);

                    // Set default (slow) core test parameters
                    ct_while_hashing = false;
                    ct_core_first = 0;
                    ct_core_last = info->core_count - 1;
                    ct_die_first = 0;
                    ct_die_last = info->die_count - 1;

                    //if (info->device_type == HFD_VC709)
                    //    info->hash_loops = (uint64_t)0x40000000;            // FPGA Emulator
                    //else
                        info->hash_loops = (uint64_t)0x100000000;           // Assume G-1

                    if (hu && hu->operation_code == OP_USB_INIT && hu->data_length)
                        {
                        struct hf_usb_init_options *ho = (struct hf_usb_init_options *)(hu+1);

                        if (!(info->ntime_roll_total = le16_to_cpu(ho->group_ntime_roll)))
                            info->ntime_roll_total = DEFAULT_NTIME_ROLL;
                        if (!(info->ntime_roll_per_core = le16_to_cpu(ho->core_ntime_roll)))
                            info->ntime_roll_per_core = DEFAULT_NTIME_ROLL_PER_CORE;
                        }
                    else
                        {
                        info->ntime_roll_total = DEFAULT_NTIME_ROLL;
                        info->ntime_roll_per_core = DEFAULT_NTIME_ROLL_PER_CORE;
                        }

                    if (h->chip_address == 0)
                        {
                        info->fault_code = E_ADDRESS_FAILURE;
                        chain_state = CS_ALL_DONE;
                        break;
                        }

                    // Pre-compute some numbers
                    info->total_cores = (uint16_t)info->die_count * (uint16_t)info->core_count;
                    if ((info->ntime_roll_total / info->ntime_roll_per_core) > info->total_cores)
                        {
                        // Simple solution for small core #'s
                        info->ntime_roll_per_core = 1;
                        info->ntime_roll_total = info->total_cores / 2;
                        }
                    info->cores_per_group = (uint16_t)info->ntime_roll_total / (uint16_t)info->ntime_roll_per_core;

                    if (info->cores_per_group > ((uint16_t)info->ntime_roll_total / (uint16_t)info->ntime_roll_per_core))
                        {
                        // XXX Change this to deal with fractional group cycles
                        info->cores_per_group = ((uint16_t)info->ntime_roll_total / (uint16_t)info->ntime_roll_per_core);
                        info->cores_per_group_cycle = info->cores_per_group * ((uint16_t)info->ntime_roll_total / (uint16_t)info->ntime_roll_per_core);
                        }
                    else
                        {
                        info->cores_per_group_cycle = info->total_cores;
                        }
                    info->groups = info->total_cores / (uint16_t)info->cores_per_group;
                    info->groups_per_group_cycle = info->cores_per_group_cycle / info->cores_per_group;

                    info->group_core_offset = info->cores_per_group_cycle / (uint16_t)info->cores_per_group;
#ifndef G1_5_OR_LATER
                    info->group_mask = (uint16_t)0xff;
                    info->group_shift = 8;
#endif


                    if (info->ntime_roll_total > 1)
                        {
                        // We're ntime rolling. Check that it is a valid configuration.
                        if (info->groups > MAX_GROUPS)
                            {
                            info->fault_code = E_TOO_MANY_GROUPS;
                            chain_state = CS_ALL_DONE;
                            break;
                            }
                        }

                    gwq_init();

                    if (info->mixed_reference_clocks)
                        {
                        // Mixed reference clock rates in a chained configuration. Tell all slaves to
                        // take over their chains and set dynamic baud rate.
                        uprintf(UD_STARTUP, "Mixed reference clocks\n");
                        twi_broadcast(TWICMD_MIXED_BAUD, 0);
                        // While that's going on, do our own chain in the master.
                        fpga_route_setup(true);
                        chain_baud();
                        chain_state = CS_BAUD_SENT;
                        }
                    else if (info->dynamic_baud_rate)
                        {
                        // Switch the chain baud rate
                        chain_baud();
                        chain_state = CS_BAUD_SENT;
                        }
                    else
                        chain_state = CS_CORE_TEST_LOW_SPEED;
                    }
                asic_pop_receive();
                }
            else if ((elapsed_time = elapsed_since(start_time)) > REPLY_TIMEOUT_MS)
                {
                if (!try_forever && ++retries > MAX_RETRIES)
                    {
                    retries = 0;
                    info->fault_code = E_ADDRESS_TIMEOUT;
                    chain_state = CS_ALL_DONE;
                    }
                else
                    {
                    init_time.address_retries++;
                    chain_address();
                    }
                }
            break;

        case CS_BAUD_SENT:
            if (asic_get_receive_count())
                {
                h = (struct hf_header *)asic_get_receive();
                if (h->operation_code == OP_BAUD)
                    {
                    // Switch UART baud rate now
                    uprintf(UD_STARTUP, "Got OP_BAUD back, baud rate is %d\n", info->dynamic_baud_rate);
                    uart_set_baudrate((unsigned long)info->dynamic_baud_rate);
                    delay_msec(2);      // Give the ASIC's time to switch as well

                    // Now proceed at the new baud rate
                    if (info->mixed_reference_clocks)
                        {
                        start_time = msec_ticker;
                        chain_state = CS_MIXED_BAUD;
                        }
                    else
                        chain_state = CS_CORE_TEST_LOW_SPEED;
                    }
                asic_pop_receive();
                }
            else if ((elapsed_time = elapsed_since(start_time)) > REPLY_TIMEOUT_MS)
                {
                if (!try_forever && ++retries > MAX_RETRIES)
                    {
                    retries = 0;
                    info->fault_code = E_BAUD_TIMEOUT;
                    chain_state = CS_ALL_DONE;
                    }
                else
                    {
                    init_time.baud_retries++;
                    chain_baud();
                    }
                }
            break;

        case CS_MIXED_BAUD:
            if (info->slave_autonomous)
                {
                // I'm a slave! Switch my routing back to the master, and go back to sleep
                fpga_route_setup(false);
                info->slave_autonomous = false;
                chain_state = CS_IDLE;
                }
            else if (elapsed_since(start_time) > 10)                     // Ensure all slaves have re-connected
                {
                fpga_route_setup(false);
                chain_address();                                        // Send a new OP_ADDRESS
                retries = 0;
                chain_state = CS_VERIFY_CHAIN;
                }
            break;

        case CS_VERIFY_CHAIN:
            // Make sure, after a mixed reference baud rate change, the chain is still intact and as we expect it.
            sts = local_receive(OP_ADDRESS, &h, 10);
            if (sts > 0)
                {
                uprintf(UD_STARTUP, "VERIFY_CHAIN: %d dies %d cores\n", h->chip_address, h->core_address);
                if (h->chip_address != info->die_count)
                    {
                    // Chain is not back together properly, this will be a bug
                    info->fault_code = E_MIXED_MISMATCH;
                    chain_state = CS_ALL_DONE;
                    }
                else
                    {
                    // Right to go, at the final dynamic baud rate
                    chain_state = CS_CORE_TEST_LOW_SPEED;
                    }
                asic_pop_receive();
                }
            else if (sts < 0)
                {
                // Timed out
                ++retries;
                if (!try_forever && ++retries > MAX_RETRIES)
                    {
                    retries = 0;
                    info->fault_code = E_MIXED_TIMEOUT;
                    chain_state = CS_ALL_DONE;
                    }
                else
                    chain_address(); 
                }
            break;

#if 0
        // Turn all clock gates OFF (for manufacturing test with no heat sink)
        case CS_CLOCKGATE_ALL_OFF:
            if (asic_get_receive_count())
                {
                h = (struct hf_header *)asic_get_receive();
                if (h->operation_code == OP_CLOCKGATE)
                    {
                    init_time.got_clockgate_back = elapsed_since(init_time.start_time);
                    chain_state = CS_CORE_TEST_LOW_SPEED;
                    }
                asic_pop_receive();
                }
            else if ((elapsed_time = elapsed_since(start_time)) > REPLY_TIMEOUT_MS)
                {
                if (!try_forever && ++retries > MAX_RETRIES)
                    {
                    retries = 0;
                    info->fault_code = E_CLOCKGATE_TIMEOUT;
                    chain_state = CS_ALL_DONE;
                    }
                else
                    {
                    init_time.clockgate_retries++;
                    //chain_clockgate_all(HF_BROADCAST_ADDRESS, false, NULL);
                    chain_clockgate_all(HF_BROADCAST_ADDRESS, true, NULL);
                    }
                }
            break;
#endif
        // Test each core individually
        case CS_CORE_TEST_LOW_SPEED:
            if ((good_cores = chain_test_cores()))
                {
                if (good_cores > 0)
                    {
                    uprintf(UD_STARTUP, "Finished core test, good_cores %d out of %d\n", good_cores, ucinfo.die_count * 96);
                    info->total_good_cores = good_cores;
                    chain_setup_pll(true, 0, ~0, true);                       // Gotta get internal pll_bypass set first
                    chain_state = CS_SETUP_PLL;
                    }
                else
                    {
                    // No good cores? Device totally broken?
                    uprintf(UD_STARTUP, "Total core failure\n");
                    info->fault_code = E_TOTAL_CORE_FAILURES;
                    chain_state = CS_ALL_DONE;
                    }
#ifdef FEATURE_CHARACTERIZATION
                if (hu && hu->operation_code == OP_CHARACTERIZE)
                    {
                    // Override, enter characterization loop
                    chain_state = CS_CHARACTERIZE_LOOP;
                    }
#endif
                }
            break;

        case CS_SETUP_PLL:
            if (!(asic_get_transmit_count()) && elapsed_since(start_time) > 5)  // Wait for the internal bypass to be set up
                {
                if (!info->fake_standalone_test && hu && !hu->pll_bypass && (hu->hash_clock == 1 || hu->hash_clock >= info->ref_frequency))
                    {
                    chain_setup_pll(false, hu->hash_clock, ~0, true);     // Now allow the PLL to go
                    }

                init_time.setup_pll_start = elapsed_since(init_time.start_time);
                chain_state = CS_WAIT_PLL;
                }
            break;

        case CS_WAIT_PLL:
            // Allow some time for the PLL to lock
            if (elapsed_since(start_time) > 3)
                {
                if (!info->fake_standalone_test && hu && !hu->pll_bypass && (hu->hash_clock == 1 || hu->hash_clock >= info->ref_frequency)
                    && hu->do_atspeed_core_tests)
                    {
                    memcpy(core_good_slow, core_good, sizeof(core_good));
                    chain_state = CS_CORE_TEST_ATSPEED;
                    }
                else
                    {
                    chain_config();
#ifdef FEATURE_TURN_ON_GPO_LEDS
                    chain_gpo_leds_on();
#endif
                    chain_state = CS_CONFIG_SENT;
                    }
                }
            break;

        // Test each core individually, at speed
        case CS_CORE_TEST_ATSPEED:
            if ((good_cores = chain_test_cores()))
                {
                if (good_cores > 0)
                    {
                    info->total_good_cores = good_cores;
                    chain_config();
                    chain_state = CS_CONFIG_SENT;
                    }
                else
                    {
                    // No good cores? Device totally broken?
                    info->fault_code = E_TOTAL_CORE_FAILURES;
                    chain_state = CS_ALL_DONE;
                    }
                }
            break;

        case CS_CONFIG_SENT:
            if (asic_get_receive_count())
                {
                h = (struct hf_header *)asic_get_receive();
                if (h->operation_code == OP_CONFIG)
                    {
                    init_time.got_config_back = elapsed_since(init_time.start_time);
                    asic_enable_host();                         // Revert back to host control
                    chain_state = CS_SETUP_GROUPS;
                    }
                asic_pop_receive();
                }
            else if ((elapsed_time = elapsed_since(start_time)) > REPLY_TIMEOUT_MS)
                {
                if (!try_forever && ++retries > MAX_RETRIES)
                    {
                    uprintf(UD_STARTUP, "No config frames back, %d retries in %d msec\n", retries, elapsed_since(start_time));
                    retries = 0;
                    info->fault_code = E_CONFIG_TIMEOUT;
                    chain_state = CS_IDLE;
                    }
                else
                    {
                    init_time.config_retries++;
                    chain_config();
                    }
                }
            break;

        case CS_SETUP_GROUPS:
            // Set up groups
#ifdef G1_5_OR_LATER
            if (chain_setup_groups())
                chain_state = CS_WAIT_CLEAR;
#else
            while (!chain_setup_groups())
                ;
            chain_state = CS_WAIT_CLEAR;
#endif
            break;

        case CS_WAIT_CLEAR:
            if (!(asic_get_transmit_count()))
                {
                init_time.all_done = elapsed_since(init_time.start_time);
                chain_state = CS_ALL_DONE;
                }
            break;

        case CS_SHUTTING_DOWN:
            h = (struct hf_header *)info->usb_init_header;

            status_watchdog_clock = 0;

            if (!(asic_get_transmit_count()) || elapsed_since(start_time) > 25)
                {
                usb_powerdown_request();
                }
            //if (h && ((le16_to_cpu(h->hdata) & 0x1) == 0))
            info->usb_init_header = NULL;
            break;

        case CS_ALL_DONE:
            chain_state = CS_IDLE;
            hu = info->usb_init_header;
            if (hu && hu->operation_code == OP_USB_INIT)
                {
                asic_enable_host();
                chain_usb_init_reply();
                status_watchdog_clock = STATUS_WATCHDOG_RECHARGE;
                }
            else if (info->fake_standalone_test)
                {
                // XXX Figure out test pass criteria
                int total_die = 0;

                for (int i = 0; i < 4; i++)
                    if (info->die_enable[i])
                        total_die++;

                led_mode = LED_STATIC;
                if (info->total_good_cores >= total_die * MINIMUM_ACCEPTABLE_GOODCORES_PER_DIE) {
                    POWER_LED_OFF;
                    ACTIVITY_LED_ON;
                } else {
                    POWER_LED_OFF;
                    ACTIVITY_LED_OFF;
                }

                fpga_reg_write(FA_REG_ENABLE, 0);       // Turn core power off
                delay_msec(5);
                gpio_set_pin_high(ENABLE_IO_VDD);       // Turn off I/O power

                // XXX Do something sensible now, test completed
                }
#ifdef FEATURE_CHARACTERIZATION
            else if (hu && hu->operation_code == OP_CHARACTERIZE)
                {
                chain_state = CS_CHARACTERIZE_LOOP;
                }
#endif
            break;


#ifdef FEATURE_CHARACTERIZATION
        case CS_CHARACTERIZE_LOOP:
            chain_state = characterization_test();
            break;

        case CS_CHARACTERIZE_DONE:
            // XXX Generate response if error occurred
            chain_state = CS_IDLE;
            break;

#endif          // FEATURE_CHARACTERIZATION

        default:
            chain_state = CS_IDLE;
            break;
        }
    }

//
// Module initialization
//
void system_init()
    {
    struct hf_usb_init_header *hu = ucinfo.usb_init_header;

    if (hu)
        {
        if (!hu->no_asic_initialization)
            chain_init(1);
        }
    }

//
// Canned operations.
//
const struct hf_header reset_op = {HF_PREAMBLE, OP_RESET, HF_BROADCAST_ADDRESS, HF_BROADCAST_ADDRESS, (uint16_t)0, (uint8_t)0, (uint8_t)0};
const struct hf_header address_op = {HF_PREAMBLE, OP_ADDRESS, 0, 0, (uint16_t)0, (uint8_t)0, (uint8_t)0};

//
//
//
static void asic_unreset()
    {
    fpga_reg_write (FA_ASIC_CONTROL, F_ASIC_UNRESET | F_ASIC_PLL_BYPASS | ucinfo.asic_baud_rate_code);
    if (ucinfo.master == true && ucinfo.no_slaves == false)
        {
        twi_broadcast(TWICMD_FPGA_ASIC_CTL, F_ASIC_UNRESET | F_ASIC_PLL_BYPASS |
            ucinfo.asic_baud_rate_code | ((ucinfo.dynamic_baud_rate) ? 0 : F_FORCE_BAUD));
        delay_msec(2);
        }
    }

//
// Initialize the chain
//
void chain_init(uint8_t startup)
    {
    struct hf_usb_init_header *hu = ucinfo.usb_init_header;
    struct hf_header *h;

    asic_enable_local();

    if (hu)
        {
        // I've got a host, and he told me to do this. Find out what
        // the host wants done, and do it.
        }

    // Send out a reset
    if (!(h = (struct hf_header *)asic_get_tx_buffer()))
        return;
    memcpy(h, &reset_op, sizeof(reset_op));
    h->crc8 = hf_crc8((uint8_t *)h);

    asic_queue_transmit();

    start_time = msec_ticker;
    }

//
// Send an OP_ADDRESS command
//
static void chain_address()
    {
    struct hf_header *h;
    static uint8_t passes = 0;

    asic_enable_local();

    if (!(h = (struct hf_header *)asic_get_tx_buffer()))
        return;
    memcpy(h, &address_op, sizeof(address_op));
    h->crc8 = hf_crc8((uint8_t *)h);
    asic_queue_transmit();

    passes++;
    start_time = msec_ticker;
    }

//
// Send an OP_BAUD command to set a dynamic baud rate
//
static void chain_baud()
    {
    struct ucinfo_t *info = &ucinfo;
    struct hf_header *h;
    static uint8_t passes = 0;

    uint32_t baud_delay;

    baud_delay = (info->ref_frequency * 1000000) / info->dynamic_baud_rate - 1;

    asic_enable_local();

    if (!(h = (struct hf_header *)asic_get_tx_buffer()))
        return;
    memcpy(h, &reset_op, sizeof(reset_op));
    h->operation_code = OP_BAUD;
    h->core_address = 3;                                    // Watchdog timeout, 3 seconds
    h->hdata = cpu_to_le16((uint16_t)baud_delay);
    h->crc8 = hf_crc8((uint8_t *)h);

    asic_queue_transmit();

    passes++;
    start_time = msec_ticker;
    }

//
// Only called if I'm a slave. A master in a mixed reference clock system
// hash told all slaves to set their dynamic baud rates.
//
void set_mixed_slave_baudrate(void)
    {

    if (!ucinfo.dynamic_baud_rate)
        return;
    ucinfo.mixed_reference_clocks = 1;       // So CS_BAUD_SENT knows what is going on
    ucinfo.slave_autonomous = true;

    fpga_route_setup(true);                 // Switch die serial chain to local uC loop
    chain_baud();                           // Send out the OP_BAUD
    chain_state = CS_BAUD_SENT;
    }

#ifdef FEATURE_TURN_ON_GPO_LEDS
//
// Turn on the ASIC's gpo[] LED functions, for diagnostics
//
static void chain_gpo_leds_on()
    {
    struct hf_header *h;

    asic_enable_local();

    // Enable the blinkies
    if (!(h = (struct hf_header *)asic_get_tx_buffer()))
        return;
    memcpy(h, &reset_op, sizeof(reset_op));
    h->operation_code = OP_GPIO;
    h->core_address = 2;
    h->hdata = cpu_to_le16((uint16_t)0x8ff);
    h->crc8 = hf_crc8((uint8_t *)h);
    asic_queue_transmit();

    // Now turn on output enables
    if (!(h = (struct hf_header *)asic_get_tx_buffer()))
        return;
    memcpy(h, &reset_op, sizeof(reset_op));
    h->operation_code = OP_GPIO;
    h->core_address = 0;
    h->hdata = cpu_to_le16((uint16_t)0xff00);
    h->crc8 = hf_crc8((uint8_t *)h);
    asic_queue_transmit();
    }
#endif

#if 0
//
// Slam all clock gates either OFF or ON, or in accordance with a map
//
static void chain_clockgate_all(uint8_t die, bool on, uint16_t *map)
    {
    struct hf_header *h;
    struct ucinfo_t *info = &ucinfo;
    uint16_t *cg, *bg;
    uint16_t j;

    if (!(h = (struct hf_header *)asic_get_tx_buffer()))
        return;
    memcpy(h, &reset_op, sizeof(reset_op));
    h->chip_address = die;
    h->operation_code = OP_CLOCKGATE;

    cg = clockgate_state;
    if (die < info->die_count)
        cg += (((uint16_t)die * (uint16_t)info->core_count) >> 4);       // Point to start of clockgate bits
    // else die must be the broadcast address

    if (info->core_count <= 16)
        {
        // A short packet, clock gate data in hdata field
        if (map)
            {
            h->hdata = cpu_to_le16(*map);
            }
        else if (on)
            {
            h->hdata = (uint16_t)0xffff;
            *cg = (uint16_t)0xffff;
            if (die == HF_BROADCAST_ADDRESS)
                {
                for (j = 1, bg = cg; j < info->die_count; j++)
                    {
                    bg += ((uint16_t)info->core_count >> 4);
                    *bg = (uint16_t)0xffff;
                    }
                }
            }
        else
            {
            h->hdata = (uint16_t)0;
            *cg = (uint16_t)0;
            if (die == HF_BROADCAST_ADDRESS)
                {
                for (j = 1, bg = cg; j < info->die_count; j++)
                    {
                    bg += ((uint16_t)info->core_count >> 4);
                    *bg = (uint16_t)0x0;
                    }
                }
            }
        }
    else
        {
        // A long packet
        h->data_length = (info->core_count + 31) >> 5;
        }
    h->crc8 = hf_crc8((uint8_t *)h);
    if (info->core_count > 16)
        {
        uint16_t *p = (uint16_t *)(h+1);
        uint8_t i, nb;

        for (i = 0, nb = 0; i < info->core_count; i += 16)
            {
            if (map)
                {
                *p++ = cpu_to_le16(*map++);
                }
            else if (on)
                {
                *p++ = (uint16_t)0xffff;
                *cg = (uint16_t)0xffff;
                if (die == HF_BROADCAST_ADDRESS)
                    {
                    for (j = 1, bg = cg; j < info->die_count; j++)
                        {
                        bg += ((uint16_t)info->core_count >> 4);
                        *bg = (uint16_t)0xffff;
                        }
                    }
                cg++;
                }
            else
                {
                *p++ = (uint16_t)0x0;
                *cg = (uint16_t)0x0;
                if (die == HF_BROADCAST_ADDRESS)
                    {
                    for (j = 1, bg = cg; j < info->die_count; j++)
                        {
                        bg += ((uint16_t)info->core_count >> 4);
                        *bg = (uint16_t)0x0;
                        }
                    }
                cg++;
                }
            nb += 2;
            }
        hf_crc32((uint8_t *)(h+1), nb, 1);
        }

    asic_queue_transmit();
    start_time = msec_ticker;
    }

//
// Modify a single clockgate, keeping the clockgate array up to date.
//
static void single_clockgate(uint8_t die, uint8_t core, bool on)
    {
    struct hf_header *h;
    struct ucinfo_t *info = &ucinfo;
    uint16_t *cg;

    if (!(h = (struct hf_header *)asic_get_tx_buffer()))
        return;
    memcpy(h, &reset_op, sizeof(reset_op));
    h->chip_address = die;
    h->operation_code = OP_CLOCKGATE;

    cg = clockgate_state;
    cg += (((uint16_t)die * (uint16_t)info->core_count) >> 4);       // Point to start of clockgate bits

    if (info->core_count <= 16)
        {
        // A short packet, clock gate data in hdata field
        if (on)
            h->hdata = cpu_to_le16( *cg | ((uint16_t)1 << core));
        else
            h->hdata = cpu_to_le16(*cg & ~((uint16_t)1 << core));
        *cg = le16_to_cpu(h->hdata);
        }
    else
        {
        // A long packet
        h->data_length = (info->core_count + 31) >> 5;
        }
    h->crc8 = hf_crc8((uint8_t *)h);
    if (info->core_count > 16)
        {
        uint16_t *p = (uint16_t *)(h+1);
        uint8_t i, nb;

        for (i = 0, nb = 0; i < info->core_count; i += 16)
            {
            if (i <= core && core < i+16)
                {
                if (on)
                    *p = cpu_to_le16(*cg | ((uint16_t)1 << core));
                else
                    *p = cpu_to_le16(*cg & ~((uint16_t)1 << core));
                *cg = le16_to_cpu(*p);
                p++;
                cg++;
                }
            else
                *p++ = cpu_to_le16(*cg++);
            nb += 2;
            }
        hf_crc32((uint8_t *)(h+1), nb, 1);
        }
    asic_queue_transmit();
    }
#endif

//
// Setup the PLL, and thereby the hash search clock
//
static uint16_t chain_setup_pll(bool bypass, uint16_t specified_hash_clock, uint32_t mask, bool reset)
    {
    struct ucinfo_t *info = &ucinfo;
    struct hf_pll_config *h;
    die_settings_t *s = NULL;
    uint16_t actual = 0;
    uint32_t hash_clock;
    uint32_t reference;
    int i;
    int die;                                                // Physical die
    int vdie;                                               // Virtual die
    int die_low;
    bool first = true;

    if (ucinfo.mixed_reference_clocks || (mask != (uint32_t)~0))
        die_low = 0;                                        // Need to use separate OP_PLL_CONFIG's
    else
        die_low = (unsigned)HF_BROADCAST_ADDRESS;           // Backward compatible, single broadcast operation

    if (bypass == false && specified_hash_clock == 1)
        {
        // Use user page die settings.
        die_low = 0;
        }

    //notify_host("chain_setup_pll: die_low %d physical die count %d mask 0x%08x", die_low, ucinfo.physical_die_count, mask);
    for (i = 0, s = &all_die_settings[0], die = vdie = die_low; i < info->physical_die_count; i++, s++)
        {
        if (s->voltage == DS_DISABLED)
            {
            if (die != HF_BROADCAST_ADDRESS)
                {
                die++;
                mask >>= 1;
                continue;                                   // Die is disabled so we don't have to do anything with it
                }
            }
        if (die != HF_BROADCAST_ADDRESS)
            {
            if (!(mask & 0x1))
                {
                die++;
                mask >>= 1;
                continue;                                   // No adjustment occurred
                }
            mask >>= 1;

            if (specified_hash_clock == 1)
                {
                if (dynamic_hash_clock_settings[die] == 0)
                    uprintf(UD_STARTUP, "Ooops... die %d\n", die);
                hash_clock = dynamic_hash_clock_settings[die];
                }
            else
                hash_clock = (uint32_t)specified_hash_clock * 1000000;
            reference = (uint32_t)module_ref_clocks[vdie]* 1000000;
            }
        else
            {
            hash_clock = (uint32_t)specified_hash_clock * 1000000;
            reference = (uint32_t)info->ref_frequency * 1000000;
            }

        if (!(h = (struct hf_pll_config *)asic_get_tx_buffer()))
            {
            notify_host("chain_setup_pll: ERROR: No send buffer available!");
            return(0);                                      // This would be a bug - we're only supposed to call chain_setup_pll()
            }                                               // when there are plenty of send buffers available.
        memset(h, 0, sizeof(*h)); 

        if (bypass)
            {
            h->pll_bypass = 1;
            actual = info->ref_frequency;
            //uprintf(UD_CHAR, "pll_calc: bypass: actual %d MHz\n", actual);
            }
        else
            {
            actual = pll_calc(h, hash_clock, reference);
            uprintf(UD_CHAR, "pll_calc: i %d die %d act %d Mhz vs %d Hz, ref %d Mhz\n", i, die, actual, hash_clock, reference);
            }
        h->pll_fse = 1;
        h->pll_reset = (reset) ? 1 : 0;
        h->preamble = HF_PREAMBLE;
        h->operation_code = OP_PLL_CONFIG;
        h->chip_address = die;
        h->crc8 = hf_crc8((uint8_t *)h);
        //notify_host("PLL: die %2d: %d Mhz ref %d byp %s, divr %d divf %d divq %d rng %d", die, actual, reference/1000000, (h->pll_bypass) ? "yes" : "no ", h->pll_divr, h->pll_divf, h->pll_divq, h->pll_range);

        if (!h->pll_bypass && actual)
            {
            info->core_frequency = actual;
            // Clear external PLL Bypass (internal will still be active)
            if (first)
                {
                first = false;
                fpga_reg_write (FA_ASIC_CONTROL, F_ASIC_UNRESET | info->asic_baud_rate_code);
                if (ucinfo.master == true && ucinfo.no_slaves == false)
                    {
                    twi_broadcast(TWICMD_FPGA_ASIC_CTL, F_ASIC_UNRESET | ucinfo.asic_baud_rate_code);
                    delay_msec(2);
                    }
                }
            }
        else
            info->core_frequency = info->ref_frequency;

        asic_queue_transmit();

        vdie++;
        if (die++ == HF_BROADCAST_ADDRESS)
            break;
        }

    start_time = msec_ticker;
    return(actual);
    }


//
// Send an OP_CONFIG command
//
// For the common reference clock case, we send a single broadcast.
// For the mixed reference clock case, we send the broadcast and then follow up with a unicast to each die that has
// a "different" reference clock than the master.
//
// This is necessary because the "one_usec" field in the OP_CONFIG frame is different for different reference clock values.
//
static void chain_config()
    {
    struct hf_usb_init_header *hu = ucinfo.usb_init_header;
    struct hf_usb_init_base *hb;
    struct hf_config_data *hc;

    struct hf_header *h;
    struct hf_config_data *c;
    struct ucinfo_t *info = &ucinfo;
    uint8_t i;

    if (hu && hu->operation_code == OP_USB_INIT && hu->user_configuration)
        c = (struct hf_config_data *)(hu+1);                // User specified config data
    else
        c = (struct hf_config_data *)NULL;                  // Internally specified config data

    // Activity is keyed off periodic OP_STATUS frames, which won't happen if a
    // transmission error causes the OP_CONFIG to get lost, so we send two. If they
    // both get lost, a watchdog will recover.
    h = (struct hf_header *)asic_get_tx_buffer();
    make_config_frame(h, HF_BROADCAST_ADDRESS, c, 0, 0, 0);
    asic_queue_transmit();

    h = (struct hf_header *)asic_get_tx_buffer();
    make_config_frame(h, HF_BROADCAST_ADDRESS, c, 0, 0, 0);
    asic_queue_transmit();

    if (ucinfo.mixed_reference_clocks)
        {
        // Gotta fix up the die with a different reference clock to us.
        for (i = 0; i < ucinfo.die_count; i++)
            {
            if (module_ref_clocks[i] != info->ref_frequency)
                {
                if ((h = (struct hf_header *)asic_get_tx_buffer()))
                    {
                    make_config_frame(h, i, c, 0, 0, module_ref_clocks[i]);         // Override reference clock value
                    asic_queue_transmit();
                    }
                // else no buffers - there's not much we can do
                }
            }
        }

    chain_state = CS_CONFIG_SENT;
    start_time = msec_ticker;

    if (hu)
        {
        // Start filling in a few usb_base_init fields
        hb = (struct hf_usb_init_base *)(hu+1);
        memset(hb, 0, sizeof(*hb));
        if (info->ntime_roll_total > 1)
            hb->inflight_target = cpu_to_le16(2 * (uint16_t)info->groups);
        else
            hb->inflight_target = cpu_to_le16(2 * (uint16_t)info->total_good_cores);
        // Figure sequence modulus, minimum 256
        for (i = 8; i < 16; i++)
            {
            info->num_sequence = (uint16_t)1 << i;
            if (info->num_sequence > (2 * le16_to_cpu(hb->inflight_target)))
                break;
            }
        hb->sequence_modulus = cpu_to_le16(info->num_sequence);

        // Return whatever config was used to user
        hc = (struct hf_config_data *)(hb+1);
        memcpy(hc, c, sizeof(struct hf_config_data));
        }
    }

//
// Construct a complete OP_CONFIG frame in a specified buffer
//
int make_config_frame(struct hf_header *h, uint8_t die_address, struct hf_config_data *user, uint16_t batch_delay_override, uint16_t status_period_override, uint8_t ref_clock_override)
    {
    struct hf_usb_init_header *hu = ucinfo.usb_init_header;
    struct ucinfo_t *info = &ucinfo;
    struct hf_config_data *c;
    uint16_t *p;
    uint32_t *q;

    memcpy(h, &address_op, sizeof(address_op));
    h->operation_code = OP_CONFIG;
    h->chip_address = die_address;
    h->data_length = sizeof(*c)/4;

    // Develop the OP_CONFIG packet header "hdata" field
    h->hdata = cpu_to_le16((uint16_t)0x8000);                                   // Write registers
    if (info->thermal_trip_limit)                                               // This needs to be compulsory
        h->hdata |= cpu_to_le16((uint16_t)0x4000 | info->thermal_trip_limit);
    //if (info->tacho_enable)
    //    h->hdata |= cpu_to_le16((uint16_t)0x2000);
    h->crc8 = hf_crc8((uint8_t *)h);

    c = (struct hf_config_data *)(h+1);

    if (user)
        {
        // A host told me to use his configuration. Move it in and hope he got it right.
        memcpy(c, (struct hf_config_data *)(hu+1), sizeof(struct hf_config_data));
        }
    else
        {
        memset(c, 0, sizeof(*c));
        if (batch_delay_override || status_period_override)
            {
            // This is a work restart, or test code. In the case of a restart, I'm about to
            // abort all frames and I want to know when they're all done, without hanging around
            // for up to half a second for the next OP_STATUS frame. In the case of test code, we
            // are always hammering on a single core, so we want its idle status "quickly".
            c->send_status_on_core_idle = (batch_delay_override) ? 1 : 0;
            c->status_batch_delay = batch_delay_override;
            c->enable_periodic_status = (status_period_override) ? 1 : 0;
            c->status_period = (uint16_t)status_period_override;
            }
        else
            {
            // Not a work restart, revert to normal status frame configuration
            if (info->status_period_10ms)
                {
                c->enable_periodic_status = 1;
                c->status_period = (uint16_t)info->status_period_10ms*10;
                }
            if (info->status_batch_delay_10ms)
                {
                c->send_status_on_pending_empty = 1;
                c->status_batch_delay = (uint16_t)info->status_batch_delay_10ms*10;
                }
            }
        c->rx_header_timeout = 20;
        c->rx_data_timeout = 20;
        c->one_usec = (uint32_t) ((ref_clock_override) ? ref_clock_override : info->ref_frequency);
        c->max_nonces_per_frame = 1;
        c->stats_interval = 10;                             // OP_STATISTICS every 10 seconds if any non-zero stats

        //c->rx_ignore_data_crc=1;
        //c->forward_all_packets=1;

        c->measure_interval = 100;                          // Temperature measure every 100 msec
        c->watchdog = info->asic_watchdog;

        c->voltage_sample_points = 0x1f;

#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
        // Swap fields. Isn't this fun!
        p = (uint16_t *)c;
        *p = cpu_to_le16(*p);
        q = (uint32_t *)c;
        q += 2;
        *q = cpu_to_le32(*q);
        q++;
        p = (uint16_t *)q;
        *p = cpu_to_le16(*p);
        p++;
        *p = cpu_to_le16(*p);
#endif

        }
    hf_crc32((uint8_t *)c, sizeof(*c), 1);                  // Plug in the CRC-32

    return (sizeof(*h) + sizeof(*c) + 4);
    }

//
// Called repeatedly to set up all of the multicast groups
//

static uint8_t chain_setup_groups()
    {
#ifdef G1_5_OR_LATER
    struct hf_header *h;
    struct hf_group_data *g;
#endif
    struct ucinfo_t *info = &ucinfo;
    static uint8_t die;
    static uint8_t core;
    static uint8_t group;
    static uint8_t ntime;
    uint8_t done = 0;

    if (info->usb_init_header && info->usb_init_header->protocol != PROTOCOL_GLOBAL_WORK_QUEUE)
        return(1);                                  // no groups

#ifdef G1_5_OR_LATER
    if (!(h = (struct hf_header *)asic_get_tx_buffer()))
        return(0);                                  // Try again later

    if (asic_get_receive_count() > RX_BUFFERS/2)    // XXX Crude flow control
        return(0);

    //SCOPE_2_ON;
    memcpy(h, &address_op, sizeof(address_op));
    h->operation_code = OP_GROUP;
    h->chip_address = die;
    h->core_address = core;
    h->hdata = cpu_to_le16((uint16_t)0x100 | (uint16_t)GROUP_ID(group));     // Set group
    h->data_length = U32SIZE(*g);
    h->crc8 = hf_crc8((uint8_t *)h);

    g = (struct hf_group_data *)(h+1);

    g->nonce_msoffset = 0;
    g->ntime_offset = ntime;

#endif

    gwq_set_group_valid(group);                     // Set the group as being valid

    if (++group >= info->groups)
        {
        group = 0;
        if (++ntime >= info->groups_per_group_cycle)
            ntime = 0;
        }
    if (++core >= info->core_count)
        {
        core = 0;
        if (++die >= info->die_count)
            {
            die = 0;
            group = 0;
            done = 1;
            }
        }

#ifdef G1_5_OR_LATER
    hf_crc32((uint8_t *)g, sizeof(*g), 1);          // Plug in the CRC-32
    asic_queue_transmit();
    //SCOPE_2_OFF;
#endif

    return(done);
    }

////////////////////////////////////////////////////////////////////////////////
//
// Test cores
//
// This function is called repeatedly, and only ever returns non-zer when all
// tests have been completed.
//
////////////////////////////////////////////////////////////////////////////////

// Test case is block #163066, expected nonce is 1907958618 (0x71b9235a)
// Search difficulty is 54 leading 0's.
const struct hf_hash_serial test_hash = { {                                                 // Midstate
                                          0x4b, 0x50, 0xad, 0x7b, 0x58, 0x7f, 0x42, 0xa9,
                                          0xd2, 0xba, 0xa6, 0x01, 0x7b, 0x8b, 0xea, 0xd6,
                                          0x8a, 0x59, 0xf4, 0x48, 0xbb, 0x7c, 0x5a, 0x9b,
                                          0xda, 0xe7, 0x9d, 0xe8, 0x02, 0x73, 0xad, 0xc2,
                                          },
                                          { 0x6d, 0x06, 0xad, 0xc8 },                       // Merkle residual
                                          BSWAP32_IF_BE(0x02b3194f),                        // Timestamp BE
                                          BSWAP32_IF_BE(0xd7690d1a),                        // Bits
                                          BSWAP32_IF_BE(0x71b92352),                        // Starting nonce
                                          BSWAP32_IF_BE(0x00000200),                        // 512 nonce loops
                                          0x0000,                                           // ntime loops
                                          0x36,                                             // Search difficulty
                                          0x00,                                             // option
                                          0x00,                                             // group
                                          { 0x00, 0x00, 0x00 }                              // spare3
                                          //0xd69e8414                                      // crc32
                                        };

const static uint32_t test_hash_result = 0x71b9235a;
const static uint16_t test_hash_ntime_offset = 0;

static uint16_t ct_die_temperature;         // The last die temperature returned in hdata of an OP_NONCE during core test
static uint8_t  ct_core_voltage;

static uint8_t good_cores_per_die[MAX_DIE];
static uint8_t half_cores_per_die[MAX_DIE];

#define G1_CORES_PER_DIE    96              // XXX Find a home for this

uint16_t core_good_die[G1_CORES_PER_DIE/16];

#define WORKAROUND_FOGBUGZ_7196
#define RETRIES_7196                5

enum core_test_t {
    CT_IDLE = 0,
    CT_SEND_TESTCASES,
    CT_FLUSH_TX_BUFFERS,
    CT_WAIT_FOR_NONCES,
    CT_FINISH
    };

static int16_t chain_test_cores()
    {
    struct hf_header *h;
    struct hf_candidate_nonce *n;
    struct ucinfo_t *info = &ucinfo;
    static uint8_t die = 0;
    static uint8_t core = 0;
    static uint8_t ct_state = CT_IDLE;
    static uint8_t sequence;
    static uint16_t idx;
    static int16_t good_cores;
    static int16_t good_cores_this_die;
    static int16_t half_cores_this_die;
    static uint8_t good;
    static uint8_t gotback;
    static uint8_t timeout;
    static uint16_t ntime_roll_total_save;
    static uint16_t ntime_roll_per_core_save;
    static uint8_t first = 1;
    static uint16_t bad_nonce_value, bad_nonce_sequence;
#ifdef WORKAROUND_FOGBUGZ_7196
    static uint8_t core_retries;
    static uint8_t retard;
#endif

    bool bypass_core;
    uint8_t done = 0;
    int8_t sts;

    if (first)
        {
        // Switch these out so the core tests work OK
        ntime_roll_total_save = info->ntime_roll_total;
        ntime_roll_per_core_save = info->ntime_roll_per_core;
        info->ntime_roll_total = 1;
        info->ntime_roll_per_core = 1;
        first = 0;
        }

    switch (ct_state)
        {
        case CT_IDLE:
            die = ct_die_first;
            core = ct_core_first;
            idx = (uint16_t) 0;
            bad_nonce_value = 0;
            bad_nonce_sequence = 0;
            if (ct_characterize == false)
                {
                if (hf_nvram_bad_core_bitmap_valid())
                    {
                    uint16_t *p = &core_good[0], *q;
                    uint8_t bitmap_words_per_die = ucinfo.core_count/(sizeof(core_good[0])*8);
                    uint16_t i;

                    // Assemble a complete initial core bitmap, based on the bad core bitmap's. We do it
                    // all here because we have to skip over the bitmaps for any disabled die.
                    for (i = 0; i <= info->num_slaves; i++)
                        hf_nvram_read_bad_core_bitmap(i, &core_good[i*bitmap_words_per_die*4]);
                    // Collapse holes. Fogbugz 8281.
                    for (i = 0, p = &core_good[0]; i < ucinfo.physical_die_count; i++, p += bitmap_words_per_die)
                        {
                        if (all_die_settings[i].voltage == DS_DISABLED)
                            {
                            // Shift the entire bitmap to overwrite the hole.
                            // Not very efficient but this is startup code so speed is not an issue.
                            q = p + bitmap_words_per_die;
                            memmove(p, q, (ucinfo.physical_die_count - i - 1) * bitmap_words_per_die * sizeof(core_good[0]));
                            }
                        }
                    /* currently our softmap is equivalent to our hardmap */
                    memcpy(&core_good_persist[0], &core_good[0], info->num_slaves*bitmap_words_per_die*4);
                    }
                else
                    for (uint16_t i = 0; i < (info->total_cores+15)/16; i++)
                        core_good[i] = (uint16_t)0xffff;
                }
            good_cores = 0;
            good_cores_this_die = 0;
            half_cores_this_die = 0;
#ifdef WORKAROUND_FOGBUGZ_7196
            core_retries = 0;
            retard = 0;
#endif
            ct_state = CT_SEND_TESTCASES;
            break;

        case CT_SEND_TESTCASES:
            // We don't bother testing the core if it is already marked bad in the bad core bitmap
            if (ct_characterize == false && !(core_good[idx>>4] & ((uint16_t)1 << (idx&0xf))))
                bypass_core = true;
            else
                bypass_core = false;

            if (bypass_core == false)
                {
                if (ct_while_hashing == true)
                    send_abort(die, core, 0x3);                                             // Free up the core to test
                sequence = send_hash(die, core, &test_hash, false, false, retard);          // Even offset to nonce
                send_hash(die, core, &test_hash, true, false, retard);                      // Odd offset to nonce
                }
            good = 0;
            gotback = 0;
            timeout = 0;
            ct_state = CT_FLUSH_TX_BUFFERS;
            break;

        case CT_FLUSH_TX_BUFFERS:
            if (asic_get_transmit_count() == 0)
                {
                start_time = msec_ticker;
                ct_state = CT_WAIT_FOR_NONCES;
                }
            break;

        case CT_WAIT_FOR_NONCES:
            sts = local_receive(OP_NONCE, &h, 10);
            if (sts > 0)
                {
                // Got an OP_NONCE
                n = (struct hf_candidate_nonce *)(h+1);
                //ct_die_temperature = h->hdata;
                if (le16_to_cpu(n->sequence) == sequence || le16_to_cpu(n->sequence) == ((sequence+1)&0xff))
                    {
                    if (le32_to_cpu(n->nonce) == test_hash_result && le16_to_cpu(n->ntime) == 0)
                        ++good;
                    else
                        bad_nonce_value++;
                    ++gotback;
                    if (h->data_length > U32SIZE(*n))
                        {
                        n++;
                        if (le32_to_cpu(n->nonce) == test_hash_result && le16_to_cpu(n->ntime) == 0)
                            ++good;
                        else
                            bad_nonce_value++;
                        ++gotback;
                        }
                    }
                else
                    bad_nonce_sequence++;
                asic_pop_receive();
                }
            else if (sts < 0)
                {
                // Timed out
                ++timeout;
                }

            if ((gotback + timeout) == 2)
                {
#ifdef WORKAROUND_FOGBUGZ_7196
                if (good < 2 && core_retries < RETRIES_7196)
                    {
                    // Fogbugz 7196 workaround, go try this core again, up to five times total
                    core_retries++;
                    retard++;
                    ct_state = CT_SEND_TESTCASES;
                    }
                else
                    {
                    core_retries = 0;
#endif
                    if (ct_while_hashing == true)
                        {
                        // Make the core we just tested go busy again
                        send_hash(die, core, &test_hash, false, true, 0);
                        }

                    ct_state = CT_SEND_TESTCASES;
                    if (good == 2)
                        {
                        // Mark good core
                        if (ct_characterize == true)
                            core_good_die[idx>>4] |= ((uint16_t)1 << (idx&0xf));
                        good_cores_this_die++;
                        good_cores++;
                        }
                    else
                        {
                        if (ct_characterize == false)
                            core_good[idx>>4] &= ~((uint16_t)1 << (idx&0xf));       // Clear core valid bit
                        if (good == 1)
                            half_cores_this_die++;
                        //single_clockgate(die, core, false);                       // Disable a bad core
                        }
                    idx++;
                    if (core++ >= ct_core_last)
                        {
                        core = 0;
                        uprintf(UD_STARTUP, "die %d good_cores %d\n", die, good_cores_this_die);
                        if (ct_characterize == false)
                            {
                            good_cores_per_die[die] = good_cores_this_die;
                            half_cores_per_die[die] = half_cores_this_die;
                            }
                        good_cores_this_die = 0;
                        half_cores_this_die = 0;

                        if (die++ >= ct_die_last)
                            {
                            info->hash_loops = 0;
                            send_hash(HF_BROADCAST_ADDRESS, 0, &test_hash, false, false, 0);   // Reset sequence number
                            send_hash(HF_BROADCAST_ADDRESS, 0, &test_hash, false, false, 0);   // Coz chip 1 behind
                            ct_state = CT_FINISH;
                            if (good_cores == 0)
                                good_cores = -1;
                            }
                        }
#ifdef WORKAROUND_FOGBUGZ_7196
                    }
#endif
                }
            break;

        case CT_FINISH:
            if (asic_get_transmit_count() == 0)
                {
                start_time = msec_ticker;
                if (local_receive(0, &h, 3) > 0)    // Flush the couple of send_hash's that come back at us, or anything else
                    asic_pop_receive();
                else
                    {
                    done = 1;
                    ct_state = CT_IDLE;
                    }
                }
            break;

        default:
            ct_state = CT_IDLE;
            done = 1;
            break;
        }

    if (done) {
        info->ntime_roll_total = ntime_roll_total_save;
        info->ntime_roll_per_core = ntime_roll_per_core_save;
        first = 1;
    }

    return( (done) ? good_cores : 0);
}

//
// Receive a frame of a particular type, locally. Discard all others while looking.
// An op_to_look_for of 0 matches anything
//
int8_t local_receive(uint8_t op_to_look_for, struct hf_header **r, uint16_t timeout)
    {
    struct hf_header *h;

    while (asic_get_receive_count())
        {
        h = (struct hf_header *)asic_get_receive();
        if (h->operation_code == OP_STATUS)
            {
            struct hf_g1_monitor *m = (struct hf_g1_monitor *)(h+1);

            if (1220 < le16_to_cpu(m->die_temperature) && le16_to_cpu(m->die_temperature) < 2757)
                {
                // Reasonable values, 10 - 100 deg C
                ct_die_temperature = m->die_temperature;        // Already little endian
                ct_core_voltage = m->core_voltage[0];
                }
            }
        if (op_to_look_for == 0 || h->operation_code == op_to_look_for)
            {
            *r = h;
            return(1);
            }
        else
            {
            asic_pop_receive();
            }
        }

    if ((elapsed_time = elapsed_since(start_time)) > timeout)
        {
        return(-1);
        }

    return(0);
    }


////////////////////////////////////////////////////////////////////////////////
// Send a HASH job to a specific die and core, usually as part of a test
////////////////////////////////////////////////////////////////////////////////
static uint8_t send_hash(uint8_t die, uint8_t core, const struct hf_hash_serial *hd, bool make_odd, bool make_busy, uint8_t retard)
    {
    struct hf_header *h;
    struct hf_hash_serial *hs;
    static uint8_t sequence;

    if (!(h = (struct hf_header *)asic_get_tx_buffer()))
        return(++sequence);                                 // No nonce -> test will fail
    h->preamble = HF_PREAMBLE;
    h->operation_code = OP_HASH;
    h->chip_address = die;
    h->core_address = core;

    if (die == HF_BROADCAST_ADDRESS)
        {
        h->hdata = 0;                                       // Used to reset sequence number at end of test
        sequence = 0;
        }
    else if (!make_busy)
        h->hdata = cpu_to_le16(++sequence);
    else
        h->hdata = 0;
    h->data_length = U32SIZE(struct hf_hash_serial);
    h->crc8 = hf_crc8((uint8_t *)h);

    hs = (struct hf_hash_serial *)(h+1);
    memcpy(hs, hd, sizeof(*hd));

    if (retard)
        {
        // Used in the workaround to Fogbugz 7196. Slide the timing relationship
        // of when the nonce is going to occur, with respect to the reference clock.
        // Just like adjusting the ignition timing on your car's engine. Or, a reference
        // to the ASIC developer who gave everyone Fogbugz 7196 (me).
        hs->starting_nonce = cpu_to_le32(le32_to_cpu(hs->starting_nonce) - retard);
        }
    if (make_odd)
        {
        // Change the test case so the offset to the nonce is odd - so that we
        // test the alternate hash core within the same hf_search block.
        hs->starting_nonce = cpu_to_le32(le32_to_cpu(hs->starting_nonce) + 1);
        }
    if (make_busy)
        {
        // This is just being used to make cores busy, usually by broadcasting.
        // Change to full nonce range, make search difficulty "impossible", and
        // add ntime rolling so cores will keep hashing until aborted
        hs->starting_nonce = 0;
        hs->nonce_loops = 0;                    // 2^32
        hs->ntime_loops = cpu_to_le16(0xfff);   // * 4095 ~=  4 hours of work
        hs->search_difficulty = 125;            // Nonce's ain't gonna happen!
        }
    hf_crc32((void *) hs, sizeof(*hs), 1);
    asic_queue_transmit();

    start_time = msec_ticker;
    return(sequence);
    }

//
// Send an OP_ABORT, with the given flags
//

void make_abort(struct hf_header *h, uint8_t die, uint8_t core, uint16_t abort_flags)
    {
    h->preamble = HF_PREAMBLE;
    h->operation_code = OP_ABORT;
    h->chip_address = die;
    h->core_address = core;
    h->hdata = cpu_to_le16(abort_flags);
    h->data_length = 0;
    h->crc8 = hf_crc8((uint8_t *)h);
    }

static void send_abort(uint8_t die, uint8_t core, uint16_t abort_flags)
    {
    struct hf_header *h;

    asic_enable_local();

    // Send an abort
    if (!(h = (struct hf_header *)asic_get_tx_buffer()))
        return;
    make_abort(h, die, core, abort_flags);
    asic_queue_transmit();
    start_time = msec_ticker;
    }

#ifdef FEATURE_CHARACTERIZATION
////////////////////////////////////////////////////////////////////////////////
//
// Hammer the system through as many test conditions as requested by the host.
// At the end of each pass through the test, report results, and set up the next
// set of test conditions. Keep re-launching tests until we're all done.
//
// Since this an internal Factory test, there is no protection against silly values,
// or users who try and use it without a head unit properly fitted. The thermal
// overload temperature is culled down to 95C for this possibility.
//
// This is a one-time test, once it completes the system is self_reset()'d
//
////////////////////////////////////////////////////////////////////////////////

enum char_state_t {
    CHAR_IDLE = 0,
    CHAR_SET_PLL_BYPASS,
    CHAR_SET_FREQUENCY,
    CHAR_WAIT_PLL,
    CHAR_FLOOD_HASH,
    CHAR_SET_DAC,
    CHAR_SET_VOLTAGE,
    CHAR_PRE_TEST_DELAY,
    CHAR_RUN_TEST,
    CHAR_GET_VOLTAGE,
    CHAR_WAIT_VOLTAGE,
    CHAR_REPORT_RESULTS,
    CHAR_TEST_LOOP,
    CHAR_COMPLETE,
    CHAR_TEST_FINISH
    };

static uint8_t char_state = CHAR_IDLE;

static uint8_t characterization_test(void)
    {
    struct hf_header *hcb = (struct hf_header *) ucinfo.usb_init_header;
    struct hf_characterize *hcm = (struct hf_characterize *) (hcb+1);
    struct hf_characterize_result *hcr;
    struct hf_header *h;

    static uint16_t hash_clock;
    static uint16_t dac_setting;
    static uint8_t  die;
    static uint8_t  voltage;
    static uint8_t  temperature;
    static int16_t  good_cores;
    static uint16_t loop;
    static uint8_t  load_core;
    static bool     char_finished;
    static uint8_t  bad_status_frames;
    static const pll_entry_t *pt, *pt_start, *pt_end;

    int i, j, response_size;
    bool f_continue;
    status_code_t sts;
    uint8_t result_buf[sizeof(struct hf_header) + sizeof(struct hf_characterize_result) + 96/8] __attribute__((aligned(2)));
    uint8_t cstate = chain_state;

    switch (char_state)
        {
        case CHAR_IDLE:
            // Begin a characterization run
            loop = 0;
            char_finished = false;
            ucinfo.inhibit_watchdogs = true;

            POWER_LED_OFF
            ACTIVITY_LED_OFF
            activity_led_counter = 0;
            power_led_counter = 0xffff;
            led_mode = LED_AUTO;

            uprintf(UD_CHAR, "Reference clock is %d Mhz\n", module_ref_clocks[0]);
            hcm->flags = le16_to_cpu(hcm->flags);

            hash_clock = le16_to_cpu(hcm->f_low);               // MHz
            die = hcm->die_low;
            temperature = hcm->temp_low;
            voltage = hcm->v_low;                               // mV * 10, 81 = 0.81v
            dac_setting = le16_to_cpu(hcm->dac_low);

            if (hcm->thermal_override)
                ucinfo.thermal_trip_limit = GN_THERMAL_CUTOFF(hcm->thermal_override);
            else
                ucinfo.thermal_trip_limit = GN_THERMAL_CUTOFF(95.0);// LOWER the thermal cutoff limit
            ucinfo.status_period_10ms = 0;                          // We don't want status frames
            ucinfo.status_batch_delay_10ms = 0;

#if 0
            if (hcm->flags & F_DONT_DO_PLL_TWEAKUP)
                dont_do_pll_tweakup = true;
#endif
            if (hcm->flags & F_FORCE_PLL_R)
                hcm_force_pll_r = (int8_t) hcm->other[0];               // Force PLL reference divider
            if (hcm->flags & F_FORCE_PLL_RANGE)
                hcm_force_pll_range = (int8_t) hcm->other[1];           // Force PLL loop filter setting

#if 0
            if (hcm->flags & F_PLL_TABLE_SWEEP)
                {
                uint32_t low = le16_to_cpu(hcm->f_low) * 1000000;
                uint32_t high = le16_to_cpu(hcm->f_high) * 1000000;

                // Establish begin, end pointers
                for (i = 0, pt = &pll_table[0]; i < pll_table_entries; i++, pt++)
                    if (pt->freq >= low)
                        {
                        pt_start = pt;
                        hash_clock = (pt->freq / 1000000);
                        hcm_force_pll_r = pt->R;
                        hcm_force_pll_range = pt->range;
                        break;
                        }
                for (; i < pll_table_entries; i++, pt++)
                    if (pt->freq > high)
                        {
                        pt_end = pt;
                        break;
                        }

                }
#endif

            asic_enable_local();

            h = (struct hf_header *)asic_get_tx_buffer();
            make_config_frame(h, HF_BROADCAST_ADDRESS, NULL, 0, 0, 0);     // This gets the thermal cutoff limit in place
            asic_queue_transmit();
            start_time = msec_ticker;

            char_state = CHAR_SET_PLL_BYPASS;
            break;

        case CHAR_SET_PLL_BYPASS:
            // XXX Call a function to set target die temperature to "temperature"

            if (elapsed_since(start_time) > 10)
                {
                fpga_reg_write (FA_ASIC_CONTROL, F_ASIC_UNRESET | F_ASIC_PLL_BYPASS | ucinfo.asic_baud_rate_code);
                chain_setup_pll(true, 0, ~0, true);                   // Force internal bypass
                start_time = msec_ticker;
                ct_core_first = 0;
                ct_core_last = ucinfo.core_count - 1;
                ct_die_first = die;
                ct_die_last = die;
                char_state = CHAR_SET_FREQUENCY;
                }
            break;

        case CHAR_SET_FREQUENCY:
            if (elapsed_since(start_time) > 10)
                {
                fpga_reg_write (FA_ASIC_CONTROL, F_ASIC_UNRESET | ucinfo.asic_baud_rate_code);
                chain_setup_pll(false, hash_clock, ~0, true);
                char_state = CHAR_WAIT_PLL;
                }
            break;

        case CHAR_WAIT_PLL:
            if (elapsed_since(start_time) > 10)
                char_state = CHAR_FLOOD_HASH;
            load_core = 0;
            break;

        case CHAR_FLOOD_HASH:
            // I was using a broadcast for this, but it appears that turning all 96 cores on in the space
            // of 768 nsec was causing the power supply to sag so far, most of the search cores would lose
            // their head and not be hashing. By the time the supply recovered, only some of them were hashing,
            // so the load test was only going at around 25% load. We could do something fancy with groups, and
            // that would be faster (coz OP_GROUP is small), but for now we'll just send hash loads to every
            // individual core.
            if (hcm->flags & (F_TEST_WITH_ALL_ASIC_CORES_HASHING | F_TEST_WITH_ALL_DIE_CORES_HASHING | F_HASH_STANDALONE))
                {
                ct_while_hashing = true;
                led_mode = LED_HASH_FAST;
                for (; load_core < 96 && asic_get_transmit_count() < (TX_BUFFERS*3/4); load_core++)
                    {
                    if (hcm->flags & (F_TEST_WITH_ALL_ASIC_CORES_HASHING | F_HASH_STANDALONE))
                        {
                        int limit;

                        if (hcm->flags & F_HASH_STANDALONE)
                            {
                            die = 0;
                            limit = ucinfo.die_count;
                            }
                        else
                            limit = 4;

                        for (i = 0, j = (die & ~0x3); i < limit; ++i, ++j)
                            send_hash(j, load_core, &test_hash, false, true, 0);
                        }
                    else if (hcm->flags & F_TEST_WITH_ALL_DIE_CORES_HASHING)
                        send_hash(die, load_core, &test_hash, false, true, 0);
                    }
                if (load_core >= 96 && asic_get_transmit_count() == 0)
                    {
                    if (hcm->dac_incr)
                        char_state = CHAR_SET_DAC;
                    else if (hcm->v_incr)
                        char_state = CHAR_SET_VOLTAGE;
                    else
                        {
                        start_time = msec_ticker;
                        char_state = CHAR_PRE_TEST_DELAY;
                        }
                    }
                }
            else
                {
                if (hcm->v_incr || hcm->v_low)              // Voltage setting of some kind is there
                    char_state = CHAR_SET_VOLTAGE;
                else if (hcm->dac_incr || hcm->dac_low)     // No voltage, but DAC setting of some kinds is there
                    char_state = CHAR_SET_DAC;
                else
                    {
                    start_time = msec_ticker;
                    char_state = CHAR_PRE_TEST_DELAY;       // No settings changes, just run test
                    }
                }
            break;

        case CHAR_SET_DAC:
            dac_write(true, dac_setting);
            start_time = msec_ticker;
            char_state = CHAR_PRE_TEST_DELAY;
            break;

        case CHAR_SET_VOLTAGE:
            // XXX Not supported yet
            // XXX Need to turn OP_STATUS messages on for a while to get the core voltage back
            // XXX Could trigger a single OP_STATUS with a short test job.
            if (elapsed_since(start_time) > 2)
                {
                }
            char_state = CHAR_PRE_TEST_DELAY;
            break;

        case CHAR_PRE_TEST_DELAY:
            if (hcm->flags & F_HASH_STANDALONE)
                break;                                          // We stay stuck here. Box has to be reset to finish hashing
            else                                                // which is power down, and pull out USB cable
                {
                if (ct_while_hashing == true)
                    {
                    if (elapsed_since(start_time) > 10)         // Make sure load is fully up and running
                        char_state = CHAR_RUN_TEST;
                    }
                else
                    char_state = CHAR_RUN_TEST;
                }
            break;

        case CHAR_RUN_TEST:
            ct_characterize = true;
            if ((good_cores = chain_test_cores()))
                char_state = CHAR_GET_VOLTAGE;
            break;

        case CHAR_GET_VOLTAGE:
            ucinfo.status_period_10ms = 1;
            h = (struct hf_header *)asic_get_tx_buffer();
            bad_status_frames = 0;
            make_config_frame(h, die, NULL, 0, 0, 0);
            asic_queue_transmit();
            start_time = msec_ticker;
            char_state = CHAR_WAIT_VOLTAGE;
            break;

        case CHAR_WAIT_VOLTAGE:
            if (asic_get_transmit_count() == 0)
                {
                start_time = msec_ticker;
                sts = local_receive(OP_STATUS, &h, 25);
                if (sts > 0)
                    {
                    struct hf_g1_monitor *m = (struct hf_g1_monitor *)(h+1);

                    if (1220 < le16_to_cpu(m->die_temperature) && le16_to_cpu(m->die_temperature) < 2757)
                        {
                        // Reasonable values, 10 - 100 deg C
                        ct_die_temperature = m->die_temperature;        // Already little endian
                        ct_core_voltage = m->core_voltage[0];

                        // Turn status messages back off
                        ucinfo.status_period_10ms = 0;
                        h = (struct hf_header *)asic_get_tx_buffer();
                        make_config_frame(h, die, NULL, 0, 0, 0);
                        asic_queue_transmit();
                        asic_pop_receive();
                        char_state = CHAR_TEST_FINISH;
                        }
                    else
                        ++bad_status_frames;                            // Which can happen at excessive test frequencies under load
                    // wait for another one

                    if (bad_status_frames > 2)
                        char_state = CHAR_TEST_FINISH;                  // Bail out, finish test

                    }
                else
                    char_state = CHAR_TEST_FINISH;                      // Config timeout, too bad
                }
            break;

        case CHAR_TEST_FINISH:
            ct_characterize = false;
            if (hcm->flags & (F_TEST_WITH_ALL_DIE_CORES_HASHING | F_TEST_WITH_ALL_ASIC_CORES_HASHING))
                send_abort(HF_BROADCAST_ADDRESS, HF_BROADCAST_ADDRESS, 0x3);

            char_state = CHAR_REPORT_RESULTS;
            break;

        case CHAR_REPORT_RESULTS:
            POWER_LED_ON
            ACTIVITY_LED_ON
            activity_led_counter = 0x1000;
            led_mode = LED_AUTO;

            // Send results back to host
            h = (struct hf_header *) result_buf;
            hcr = (struct hf_characterize_result *) (h+1);
            response_size = sizeof(*h) + sizeof(*hcr);

            memset(result_buf, 0, sizeof(result_buf));

            h->preamble = HF_PREAMBLE;
            h->operation_code = OP_CHAR_RESULT;
            h->chip_address = die;
            h->core_address = 0;
            h->hdata = cpu_to_le16(loop);
            h->data_length = sizeof(*hcr)/4;
            h->crc8 = hf_crc8((uint8_t *)h);
            loop++;

            hcr->dac_setting = cpu_to_le16(dac_value);
            hcr->frequency = cpu_to_le16(hash_clock);
            hcr->core_voltage = voltage;
            hcr->die_temperature = ct_die_temperature;      // Already little endian
            hcr->measured_voltage = ct_core_voltage;
            hcr->die_index = die;

            hcr->good_cores_low_speed = good_cores_per_die[die];
            hcr->good_half_cores = half_cores_per_die[die];
            hcr->good_core_count = (good_cores > 0) ? good_cores : 0;

            if (hcm->flags & F_RETURN_PLL_PARAMETERS)
                {
                hcr->other_data = cpu_to_le32(last_pll_parameters);
                hcr->flags |= F_PLL_DATA_RETURNED;
                }
            if (hcm->flags & F_RETURN_CORE_MAPS)
                {
                hcr->flags |= F_CORE_MAP_TO_FOLLOW;
                response_size += ucinfo.core_count/8;
                memcpy((uint8_t *)(hcr+1), core_good_die, ucinfo.core_count/8);
                }

            if (
                 (
                     ((hcm->flags & F_PLL_TABLE_SWEEP) && pt >= pt_end)
                  || (!(hcm->flags & F_PLL_TABLE_SWEEP) && (hcm->f_incr == 0 || (hcm->f_incr && hash_clock >= le16_to_cpu(hcm->f_high))))
                 )
             && (hcm->dac_incr == 0 || (hcm->dac_incr && dac_setting >= le16_to_cpu(hcm->dac_high)))
             && (hcm->v_incr == 0 || (hcm->v_incr && voltage >= hcm->v_high))
             && (hcm->temp_incr == 0 || (hcm->temp_incr && temperature >= hcm->temp_high))
             && (hcm->die_incr == 0 || (hcm->die_incr && die >= hcm->die_high))
               )
                {
                char_finished = true;
                hcr->flags |= (F_LAST|F_LAST_THIS_DIE);
                }
            else
                {
                // Increment variables for next loop. We do it here to figure out if F_LAST_THIS_DIE needs to be set

                f_continue = false;
                if (hcm->flags & F_PLL_TABLE_SWEEP)
                    {
                    if (pt++ < pt_end)
                        {
                        hash_clock = (pt->freq / 1000000);
                        hcm_force_pll_r = pt->R;
                        hcm_force_pll_range = pt->range;
                        f_continue = true;
                        }
                    }
                else if (hcm->f_incr && hash_clock < le16_to_cpu(hcm->f_high))
                    {
                    hash_clock += hcm->f_incr;
                    f_continue = true;
                    }

                if (f_continue == true)
                    ;
                else
                    {
                    if (hcm->flags & F_PLL_TABLE_SWEEP)
                        {
                        pt = pt_start;
                        hash_clock = (pt->freq / 1000000);
                        hcm_force_pll_r = pt->R;
                        hcm_force_pll_range = pt->range;
                        }
                    else
                        hash_clock = le16_to_cpu(hcm->f_low);

                    if (hcm->dac_incr && dac_setting < le16_to_cpu(hcm->dac_high))
                        dac_setting += hcm->dac_incr;
                    else
                        {
                        dac_setting = le16_to_cpu(hcm->dac_low);
                        if (hcm->v_incr && voltage < hcm->v_high)
                            voltage += hcm->v_incr;
                        else
                            {
                            voltage = hcm->v_low;
                            if (hcm->temp_incr && temperature < hcm->temp_high)
                                temperature += hcm->temp_incr;
                            else
                                {
                                temperature = hcm->temp_low;
                                hcr->flags |= F_LAST_THIS_DIE;
                                if (hcm->die_incr && die < hcm->die_high)
                                    {
                                    die += hcm->die_incr;
                                    }
                                }
                            }
                        }
                    }
                }

            // Send response off to the host
            sts = udi_cdc_write_buf((uint8_t *)h, response_size);

            char_state = CHAR_TEST_LOOP;
            break;

        case CHAR_TEST_LOOP:
            // There used to be more in here, now it is a useless state
            if (char_finished)
                {
                start_time = msec_ticker;
                char_state = CHAR_COMPLETE;
                }
            else
                char_state = CHAR_SET_FREQUENCY;
            break;

        case CHAR_COMPLETE:
            // We wait for quite a long time, to avoid killing the USB pipe before the host is done
            if (elapsed_since(start_time) > 250)
                {
                start_time = msec_ticker;
                char_state = CHAR_IDLE;
                //chain_init(0);
                cstate = CS_SHUTTING_DOWN;
                }
            break;

        default:
            break;
        }

    return(cstate);
    }
#endif

////////////////////////////////////////////////////////////////////////////////
//
// Handle OP_USB_INIT operation
//
// Called from the host USB interface. We have to shut down all hashing, and
// re-initialize everything.
//
////////////////////////////////////////////////////////////////////////////////

// XXX Find somewhere else for this, to get rid of this buffer (Response staged here)
static uint8_t init_buf[sizeof(struct hf_usb_init_header) + sizeof(struct hf_config_data) + 120] __attribute__((aligned(4)));

void chain_usb_init(struct hf_usb_init_header *h)
    {
    struct ucinfo_t *info = &ucinfo;

    memcpy(init_buf, h, sizeof(*h) + h->data_length*4);

    // We set this to a non NULL value, which serves to remember it
    uprintf(UD_STARTUP, "chain_usb_init: %.3f secs\n", (float)msec_ticker / 1000.0);
    info->usb_init_header = (struct hf_usb_init_header *)init_buf;

    // That's it. The next time chain_handler is called, it will have to sort out the mess
    }

void shutdown_request()
    {
    self_shutdown_request = true;
    }

// Generate the reply to the host, after everything is done
static void chain_usb_init_reply()
    {
    struct ucinfo_t *info = &ucinfo;
    struct hf_usb_init_header *hu = info->usb_init_header;
    struct hf_usb_init_base *hb;
    struct hf_header *h = (struct hf_header *) hu;
    uint16_t saved_data_length;
    uint16_t extra;
    uint8_t i;
    uint16_t *q;


    if (h->operation_code == OP_USB_SHUTDOWN)
        {
        if (le16_to_cpu(h->hdata) & 0x1)
            {
            // XXX Power everything down
            }
        if (le16_to_cpu(h->hdata) & 0x2)
            {
            if (main_b_cdc_enable == true)
                {
                // Generate a reply
                h->hdata = 0;
                h->crc8 = hf_crc8((uint8_t *)h);
                udi_cdc_write_buf(h, sizeof(*h));
                }
            }
        }
    else
        {
        // Must be OP_USB_INIT
        // Set up last minute things from the original request
        if (hu->protocol == PROTOCOL_GLOBAL_WORK_QUEUE)
            {
            info->gwq_enabled = 1;
            status_watchdog_active = 1;
            host_watchdog_active = 1;
            }

        info->shed_supported = hu->shed_supported;

        // Generate reply. Fill in header data first.
        h->chip_address = info->die_count;
        h->core_address = info->core_count;
        h->hdata = cpu_to_le16((((uint16_t)info->ref_frequency)<<8)|info->device_type);

        // Now the initialization base data
        hb = (struct hf_usb_init_base *)(h+1);
        hb->operation_status = info->fault_code;        // If this is non-zero, that's it
        hb->extra_status_1 = info->fault_extra;

        hb->firmware_rev = cpu_to_le16(FIRMWARE_VERSION);
        hb->hardware_rev = cpu_to_le16(0x0101);       // 1.1  TODO: read this from the DIP switches

        hb->serial_number = cpu_to_le32(hf_nvram_get_short_serial());
        hb->hash_clockrate = cpu_to_le16(info->core_frequency);

        if (!info->shed_supported)
            {
            uint16_t my_inflight_target = le16_to_cpu(hb->inflight_target);
            for (i = 0; i < info->die_count; i++)
                {
                uint16_t core_idx = CORE_INDEX(i, 95);
                if (core_good[core_idx>>4] & ((uint16_t)1 << (core_idx&0xf)))
                    {
                    core_good[core_idx>>4] &= ~((uint16_t)1 << (core_idx&0xf));       // Clear core valid bit
                    my_inflight_target -= 2;
                    info->total_good_cores -= 1;
                    }
                }
            hb->inflight_target = cpu_to_le16(my_inflight_target);
            }

        hu->data_length = (sizeof(struct hf_usb_init_base) + sizeof(struct hf_config_data))/4;

        // Calculate sizes to send for core bitmap(s)
        extra = info->total_cores;
        if (extra & 0x1f)
            extra = (extra & ~0x1f) + 32;               // Round up to next uint32_t
        extra /= 8;                                     // Convert to bytes, it'll be a multiple of 4

        saved_data_length = h->data_length;

        // Adjust header data length to account for core bitmap's
        h->data_length += (extra>>2);
        h->crc8 = hf_crc8((uint8_t *)h);

        saved_data_length <<= 2;
        q = (uint16_t *)((uint8_t *)h + sizeof(struct hf_header) + sizeof(struct hf_usb_init_base) + sizeof(struct hf_config_data));
        for (i = 0; i < (extra>>1); i++)
            *q++ = cpu_to_le16(core_good[i]);
        saved_data_length += extra;
        uprintf(UD_STARTUP, "chain_usb_init_reply %.3f secs\n", (float)msec_ticker / 1000.0);
        if (main_b_cdc_enable == true)
            udi_cdc_write_buf(h, sizeof(*h) + saved_data_length);
        info->hash_loops = (uint64_t)0x100000000;           // Assume G-1
        }

    // Clear this out to start up normal operations
    info->usb_init_header = (struct hf_usb_init_header *) NULL;
    }


#ifdef FEATURE_COWARDLY_WORK_RESTART

#ifdef WORK_RESTART_DEBUG

static int sts_pkts_empty[12];
static int sts_pkts_unempty[12];

static void gwq_work_restart_clear(void);

static void gwq_work_restart_clear()
    {
    int i;

    for (i = 0; i < 12; i++)
        {
        sts_pkts_empty[i] = 0;
        sts_pkts_unempty[i] = 0;
        }
    }

void gwq_work_restart_check(uint8_t die, uint16_t *p)
    {
    int i;
    int empty = 1;

    for (i = 0; i < 6; i++)
        {
        if (*p++ != 0)
            {
            empty = 0;
            break;
            }
        }

    if (empty)
        sts_pkts_empty[die]++;
    else
        sts_pkts_unempty[die]++;
    }

#else
#define gwq_work_restart_clear()
#endif

#define COWARDLY_CLIFF      48                  // Beyond this we abort everything
#define COWARDLY_BATCHSIZE  7                   // How many OP_ABORT's per batch

static uint8_t restart_core;

bool gwq_work_restart_process(struct hf_header *h)
    {
    struct ucinfo_t *info = &ucinfo;
    struct hf_header *h_first = h;
    bool tx_buffer_used = false;
    int i;

    switch (info->restart_phase)
        {
        case RESTART_PHASE_2:
            // Send aborts
            if (restart_core < COWARDLY_CLIFF)
                {
                // Send some more aborts
                for (i = 0; i < COWARDLY_BATCHSIZE && restart_core < 96; i++, h++)
                    make_abort(h, HF_BROADCAST_ADDRESS, restart_core++, 0x3);
                if (i > 1)
                    h_first->data_length = i;   // Kludge to send batches of OP_ABORT's
                start_time = msec_ticker;
                tx_buffer_used = true;
                }
            else if (restart_core < 96)
                {
                // Send a final broadcast abort, to get rid of the rest
                uprintf(UD_WORK_RESTART, "P2: %d ms: final: head %4d tail %4d\n", elapsed_since(ucinfo.work_restart_start_time), group_sequence_head, group_sequence_tail);
                //make_abort(h++, HF_BROADCAST_ADDRESS, HF_BROADCAST_ADDRESS, 0x3);
                make_abort(h, HF_BROADCAST_ADDRESS, HF_BROADCAST_ADDRESS, 0x3);
                restart_core = 96;
                //h_first->data_length = 2;                       // In case first lost due to transmission error
                start_time = msec_ticker;
                tx_buffer_used = true;
                }
            else
                {
                info->restart_phase = RESTART_PHASE_3;
                gwq_work_restart_clear();
                restart_core = 0;
                }
            break;

        case RESTART_PHASE_3:
            if (!asic_get_transmit_count())                     // Wait for all the OP_ABORT's to be sent out
                {
                if (!(group_sequence_head == group_sequence_tail))
                    {
                    if (elapsed_since(start_time) > 25)
                        {
                        // May have lost an OP_ABORT, so resend it
                        uprintf(UD_WORK_RESTART, "P3: timeout: %d ms: head %4d tail %4d\n", elapsed_since(ucinfo.work_restart_start_time), group_sequence_head, group_sequence_tail);
#ifdef WORK_RESTART_DEBUG
                        uprintf(UD_WORK_RESTART, "%d %d %d %d %d %d %d %d %d %d %d %d\n", \
                                sts_pkts_empty[0], sts_pkts_empty[1], sts_pkts_empty[2], sts_pkts_empty[3], sts_pkts_empty[4], sts_pkts_empty[5],
                                sts_pkts_empty[6], sts_pkts_empty[7], sts_pkts_empty[8], sts_pkts_empty[9], sts_pkts_empty[10], sts_pkts_empty[11]);
                        uprintf(UD_WORK_RESTART, "%d %d %d %d %d %d %d %d %d %d %d %d\n", \
                                sts_pkts_unempty[0], sts_pkts_unempty[1], sts_pkts_unempty[2], sts_pkts_unempty[3], sts_pkts_unempty[4], sts_pkts_unempty[5],
                                sts_pkts_unempty[6], sts_pkts_unempty[7], sts_pkts_unempty[8], sts_pkts_unempty[9], sts_pkts_unempty[10], sts_pkts_unempty[11]);
#endif
                        make_abort(h, HF_BROADCAST_ADDRESS, HF_BROADCAST_ADDRESS, 0x3);
                        tx_buffer_used = true;
                        start_time = msec_ticker;
                        }
                    }
                else
                    {
                    gwq_init_tables();                                  // Re-initialize all gwq related tables, but don't change pointers
                    if (info->hash_clock_change)
                        {
                        uprintf(UD_WORK_RESTART, "P3: %d ms: bypass: head %4d tail %4d\n", elapsed_since(ucinfo.work_restart_start_time), group_sequence_head, group_sequence_tail);
                        start_time = msec_ticker;
                        info->restart_phase = RESTART_PHASE_4;
                        }
                    else
                        {
                        uprintf(UD_WORK_RESTART, "P3: done: %d ms, head %4d tail %4d\n", elapsed_since(ucinfo.work_restart_start_time), group_sequence_head, group_sequence_tail);
                        info->resend_op_config = true;
                        info->restart_phase = 0;                        // We're done
                        }
                    }
                }
            break;

        case RESTART_PHASE_4:                                               // This state also used in UMS mode, launched by ums_clock_change();
            // A hash clock change was requested
            if (!asic_get_transmit_count() && elapsed_since(start_time) > 2)
                {
                hash_clocks_adjust(info->hash_clock_change);
                chain_setup_pll(false, 1, info->hash_clock_change_bitmap, true);   // Go change PLL's
                start_time = msec_ticker;
                info->hash_clock_change = 0;
                info->restart_phase = RESTART_PHASE_5;
                }
            break;

        case RESTART_PHASE_5:
            // Complete
            if (!asic_get_transmit_count() && elapsed_since(start_time) > 3)
                {
                uprintf(UD_WORK_RESTART, "P5: complete: %d ms, head %4d tail %4d\n", elapsed_since(ucinfo.work_restart_start_time), group_sequence_head, group_sequence_tail);
                info->resend_op_config = true;
                info->restart_phase = 0;
                }
            break;

        default:
            break;
        }

    return(tx_buffer_used);
    }
#endif


////////////////////////////////////////////////////////////////////////////////
//
// Initialize this module
//
////////////////////////////////////////////////////////////////////////////////

// Note, when looking at the module with the uC on the left hand side, the
// die indices below refer map to:
// 0 --> Upper Left
// 1 --> Upper Right
// 2 --> Lower Right
// 3 --> Lower Left

void asic_init()
    {
    struct ucinfo_t *info = &ucinfo;

    info->status_period_10ms = 50;                              // Default status messages to every 500 msec
    info->status_batch_delay_10ms = 0;                          // Don't use triggered status messages
    info->thermal_trip_limit = GN_THERMAL_CUTOFF(110.0);

    // If you're looking for the die_enable's, they're now set in module_handler.c at startup time.
    }

//
// Initialize the hash clocks, according to what the host asked for, and the
// presence or otherwise of die settings
//

static void set_dynamic_nonce_ranges()
    {
    uint32_t i, max, max_over_64k;

    // Find maximum hash clock rate
    for (i = 0, max = 0; i < ucinfo.die_count; i++)
        if (dynamic_hash_clock_settings[i] > max)
            max = dynamic_hash_clock_settings[i];
    
    // For each hash clock we come up with a number modulo 64k which represents the fraction
    // of the max clock rate, then shift it left 16 places. This gets us a close enough
    // nonce range. Max clock rates will be 2^32, others proportional.
  
    max_over_64k = max >> 16;
    for (i = 0; i < ucinfo.die_count; i++)
        {
        if (dynamic_hash_clock_settings[i] < max)
            {
            dynamic_nonce_range[i] = (dynamic_hash_clock_settings[i] / max_over_64k) << 16;
            //notify_host("DNR: die %2d setting 0x%08x", i, dynamic_nonce_range[i]);
            }
        else
            dynamic_nonce_range[i] = 0;             // 2^32 for maximum clock rates
        }
    }

static void hash_clocks_init(uint16_t hash_clock)
    {
    int i, die;

    if (hash_clock == 1)
        {
        // Use die settings
        if (hf_nvram_die_settings_valid())
            {
            // Use the built in characterization clock rates
            for (i = 0, die = 0; i < ucinfo.physical_die_count; i++)
                {
                if (all_die_settings[i].voltage == DS_DISABLED)
                    continue;
                initial_hash_clock_settings[die] = all_die_settings[i].frequency;
                dynamic_hash_clock_settings[die++] = (uint32_t)all_die_settings[i].frequency*1000000;
                }
            }
        else
            {
            // Gotta do something
            for (i = 0; i < MAX_DIE; i++)
                {
                initial_hash_clock_settings[i] = 550;
                dynamic_hash_clock_settings[i] = (uint32_t)550000000;
                }
            }
        }
    else if (hash_clock >= MIN_HASH_CLOCK_MHZ)
        {
        // Specific one-size-fits-all clock rate
        for (i = 0; i < MAX_DIE; i++)
            {
            initial_hash_clock_settings[i] = hash_clock;
            dynamic_hash_clock_settings[i] = hash_clock*1000000;
            }
        }
    else
        {
        // Illegal, don't know what to do. This will cause clock bypass.
        memset(initial_hash_clock_settings, 0, sizeof(initial_hash_clock_settings));
        memset(dynamic_hash_clock_settings, 0, sizeof(dynamic_hash_clock_settings));
        }

    set_dynamic_nonce_ranges();
    }

//
// Adjust dynamic hash clock rates. Happens infrequently, host may request a change during
// a work restart, or we may do such a change for internal thermal control.
//
static void hash_clocks_adjust(uint16_t hash_clock_change)
    {
    struct ucinfo_t *info = &ucinfo;
    int code = hash_clock_change >> WR_COMMAND_SHIFT;
    uint16_t val = hash_clock_change & ((1<<WR_COMMAND_SHIFT)-1);
    uint32_t mask = info->hash_clock_change_bitmap;
    uint32_t val_hz = val * 1000000;
    int i;

    //notify_host("OP_WORK_RESTART: Adjust code %d mask 0x%08x val %d val_hz %d", code, mask, val, val_hz);
    switch (code)
        {
        case WR_CLOCK_VALUE:
            if (val < MIN_HASH_CLOCK_MHZ  || val > MAX_HASH_CLOCK_MHZ)
                break;
            for (i = 0; i < ucinfo.die_count; i++, mask >>= 1)
                dynamic_hash_clock_settings[i] = (mask & 0x1) ? val_hz : dynamic_hash_clock_settings[i];
            break;

        case WR_MHZ_INCREASE:
            if (val > 100)
                break;
            for (i = 0; i < ucinfo.die_count; i++, mask >>= 1)
                if (mask & 0x1)
                    dynamic_hash_clock_settings[i] += val_hz;
            break;

        case WR_MHZ_DECREASE:
            if (val > 100)
                break;
            for (i = 0; i < ucinfo.die_count; i++, mask >>= 1)
                if (mask & 0x1)
                    dynamic_hash_clock_settings[i] -= val_hz;
            break;

        case WR_PERCENT_INCREASE:
            if (val == 0 || val > 20)
                break;
            for (i = 0; i < ucinfo.die_count; i++, mask >>= 1)
                dynamic_hash_clock_settings[i] = (mask & 0x1) ? (uint32_t)(((float)dynamic_hash_clock_settings[i]) * (1.0+(float)val/100.0)) : dynamic_hash_clock_settings[i];
            break;

        case WR_PERCENT_DECREASE:
            if (val == 0 || val > 20)
                break;
            for (i = 0; i < ucinfo.die_count; i++, mask >>= 1)
                dynamic_hash_clock_settings[i] = (mask & 0x1) ? (uint32_t)(((float)dynamic_hash_clock_settings[i]) * (1.0-(float)val/100.0)) : dynamic_hash_clock_settings[i];
            break;

        case WR_REVERT:
            for (i = 0; i < ucinfo.die_count; i++, mask >>= 1)
                if (mask & 0x1)
                    dynamic_hash_clock_settings[i] = (uint32_t)initial_hash_clock_settings[i] * 1000000;
            break;

        default:
            break;
        }

    // Range limit
    for (i = 0; i < ucinfo.die_count; i++)
        {
        if (dynamic_hash_clock_settings[i] < MIN_HASH_CLOCK_MHZ * 1000000) // OK here to have MIN be 125
            dynamic_hash_clock_settings[i] = MIN_HASH_CLOCK_MHZ * 1000000;
        if (dynamic_hash_clock_settings[i] > MAX_HASH_CLOCK_MHZ * 1000000)  // XXX Check should be against nvram operating limits
            dynamic_hash_clock_settings[i] = MAX_HASH_CLOCK_MHZ * 1000000;
        }

    set_dynamic_nonce_ranges();
    }

// Fix: Make really sure that the buffer is big enough for the data
//      we are putting in it.  See tx_tail in usb_uart.c, for example.
// Fix: Why is core_good uint16_t?  Easier to manage as unsigned bytes.
uint8_t send_core_map(struct hf_header *h)
{
    uint16_t *core_info = (uint16_t *) (h+1); /* Fix: Not the right way! */
    struct ucinfo_t *info = &ucinfo;
    uint16_t extra;
    int i;

    extra = info->total_cores;
    if (extra & 0x1f)
        extra = (extra & ~0x1f) + 32;             // Round up to next uint32_t
    extra /= 8;                                   // Convert to bytes, it'll be a multiple of 4

    h->hdata = cpu_to_le16(info->total_cores);

    /* Make sure we can't run off end of core_good, although should never happen. */
    if(extra/2 > MAX_CORES/16)
        return 0;

    h->data_length = extra/4;
    h->crc8 = hf_crc8((uint8_t *)h);

    for(i=0; i < extra/2; i++)
        *(core_info + i) = cpu_to_le16(core_good[i]);

    return sizeof(*h) + extra;
}
