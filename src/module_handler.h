
#ifndef _module_handler_h
#define _module_handler_h


#define MAX_SLAVES                      4


typedef struct {
    uint16_t inputMillivolts[4];
    uint16_t outputMillivolts[4];
} moduleStatusT;

extern moduleStatusT moduleStatus[MAX_SLAVES + 1];


#define SLAVE_POWERUP_COMMAND_DELAY_MS  1000            // Time after master power-up before slave power supplies are turned on
#define SLAVE_STARTUP_COMMAND_DELAY_MS  1300            // Time after master power-up before slaves are told to start up
#define SLAVE_POWERUP_DELAY_MS           300            // Time after slave startup before slave power status is checked
                                                        // ... which happens in the slave at 1300+300 = 1600 msec after master power-up

#define MASTER_POWERUP_DELAY_MS         1800            // Time to wait after master power-up for everything to stabilize, msec


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
void fpga_route_setup(bool);
void fpga_reg_write (uint8_t, uint8_t);

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

// Elapsed msec
#define elapsed_since(start)  (((msec_ticker) < (start)) ? \
                               (((uint16_t)65535 - (start)) + (msec_ticker) + (uint16_t)1) :    /* Wrapped case */ \
                               ((msec_ticker) - (start)))                                       /* Most of the time */

// Elapsed seconds
#define seconds_elapsed_since(start)    (((sec_ticker) < (start)) ? \
                                        (((uint16_t)65535 - (start)) + (sec_ticker) + (uint16_t)1) : \
                                        ((sec_ticker) - (start)))

// Chain configurations
#define CC_UNCONFIGURED                 0               // Not yet configured
#define CC_NONE                         1               // Standalone module
#define CC_MIDDLE                       2               // In the middle of a chain
#define CC_OPEN_UP                      3               // Modules exist "UP" of me, I'm the "DOWN"-est
#define CC_OPEN_DOWN                    4               // Modules exist "DOWN" of me, I'm the "UP"-est
#define CC_LOOPBACK                     5               // I'm a single module with a loopback cable in place (factory test)


#endif /* _module_handler_h */

