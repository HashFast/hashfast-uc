/* config.h */

#ifndef _config_h
#define _config_h


#define CONFIG_SERIAL_NUMBER_ADDRESS      AVR32_FLASHC_USER_PAGE_ADDRESS
#define CONFIG_SERIAL_NUMBER_SIZE                                     28


#define CONFIG_GPIO_FAN2                  AVR32_PIN_PA08
#define CONFIG_GPIO_RECONFIG_BUTTON       AVR32_PIN_PA13
#define CONFIG_GPIO_FAN1                  AVR32_PIN_PA19
#define CONFIG_GPIO_PWR_ON                AVR32_PIN_PA22
#define CONFIG_GPIO_SPARE_UP              AVR32_PIN_PA29
#define CONFIG_GPIO_SPARE_DOWN            AVR32_PIN_PA30
#define CONFIG_GPIO_GOT_UP                AVR32_PIN_PB04
#define CONFIG_GPIO_HAVE_USB              AVR32_PIN_PB06
#define CONFIG_GPIO_USB_DOWN              AVR32_PIN_PB07
#define CONFIG_GPIO_GOT_DOWN              AVR32_PIN_PB08

#define CONFIG_PWM_CHANNEL_PWM1                        4
#define CONFIG_PWM_CHANNEL_PWM2                        0


#if defined(__AVR32_UC3B0512__)
#define CONFIG_BOARD_OSC0_HZ                     8000000
#define CONFIG_BOARD_OSC0_IS_XTAL                      0
#define CONFIG_BOARD_OSC0_STARTUP_US                 100
#elif defined(__AVR32_UC3B0256__)
#define CONFIG_BOARD_OSC0_HZ                    12000000
#define CONFIG_BOARD_OSC0_IS_XTAL                      1
#define CONFIG_BOARD_OSC0_STARTUP_US               17000
#else
#error
#endif

#if CONFIG_BOARD_OSC0_HZ == 8000000
#  define CONFIG_PLL0_MUL                               15
#  define CONFIG_PLL0_DIV                                2
#  define CONFIG_PLL1_MUL                               12
#  define CONFIG_PLL1_DIV                                2
#elif CONFIG_BOARD_OSC0_HZ == 12000000
#  define CONFIG_PLL0_MUL                               10
#  define CONFIG_PLL0_DIV                                2
#  define CONFIG_PLL1_MUL                                8
#  define CONFIG_PLL1_DIV                                2
#else
#error
#endif


#define CONFIG_CPU_FREQ         ((CONFIG_BOARD_OSC0_HZ * CONFIG_PLL0_MUL) / \
                                 CONFIG_PLL0_DIV)
#define CONFIG_PBA_FREQ         ((CONFIG_BOARD_OSC0_HZ * CONFIG_PLL0_MUL) / \
                                 CONFIG_PLL0_DIV)
#define CONFIG_USB_FREQ         ((CONFIG_BOARD_OSC0_HZ * CONFIG_PLL1_MUL) / \
                                 CONFIG_PLL1_DIV)


#define CONFIG_MAX_SLAVES                  5


#define CONFIG_TWI_SPEED              400000


#define CONFIG_INT_LEVEL_TICK       AVR32_INTC_INT0
#define CONFIG_INT_LEVEL_TWI        AVR32_INTC_INT3


#endif /* _config_h */

