/* config.h */

#ifndef _config_h
#define _config_h



#define CONFIG_GP_FUSES                       0xfff3ffff


#define CONFIG_BOARD_OSC0_HZ                    12000000
#define CONFIG_BOARD_OSC0_IS_XTAL                      1
#define CONFIG_BOARD_OSC0_STARTUP_US               17000

#define CONFIG_PLL0_MUL                               10
#define CONFIG_PLL0_DIV                                2


#define CONFIG_CPU_FREQ         ((CONFIG_BOARD_OSC0_HZ * CONFIG_PLL0_MUL) / \
                                 CONFIG_PLL0_DIV)
#define CONFIG_PBA_FREQ         ((CONFIG_BOARD_OSC0_HZ * CONFIG_PLL0_MUL) / \
                                 CONFIG_PLL0_DIV)


#define CONFIG_INT_LEVEL_TICK       AVR32_INTC_INT0


#endif /* _config_h */

