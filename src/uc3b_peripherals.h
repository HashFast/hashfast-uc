/** @file uc3b_peripherals.h
 * @brief HashFast peripherals
 *
 * TODO rename file
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

/*
 * DMA channel assignments
 */

#define DMA_UART_TX_CHANNEL         1
#define DMA_UART_RX_CHANNEL         2

#define DMA_SPI_RX_CHANNEL          3
#define DMA_SPI_TX_CHANNEL          4


/*
 * USART that talks to the ASICs is USART1
 */

#define GN_UART ((volatile avr32_usart_t *)AVR32_USART1_ADDRESS)

#define INDUCTOR_4_TEMP     AVR32_PIN_PA03                  //!< Analog (rev 0/1 boards)
#define V12_A_DETECT        AVR32_PIN_PA03                  //!< (IR-A boards)
#define INDUCTOR_3_TEMP     AVR32_PIN_PA04                  //!< Analog (rev 0/1 boards)
#define V12_B_DETECT        AVR32_PIN_PA04                  //!< (IR-A boards)
#define INDUCTOR_2_TEMP     AVR32_PIN_PA05                  //!< Analog (rev 0/1 boards)
#define V12_C_DETECT        AVR32_PIN_PA05                  //!< (IR-A boards)
#define INDUCTOR_1_TEMP     AVR32_PIN_PA06                  //!< Analog (rev 0/1 boards)
#define V12_D_DETECT        AVR32_PIN_PA06                  //!< (IR-A boards)

#define PWM2                AVR32_PIN_PA07                  //!< PWM for fan2 if 4-wire
#define FAN2                AVR32_PIN_PA08                  //!< Active high gate drive for fan2

#define TWI_SCL             AVR32_PIN_PA09                  //!< I2C clock common between all modules in a system
#define TWI_DATA            AVR32_PIN_PA10                  //!< I2C data

#define POWER_BUTTON        AVR32_PIN_PA11                  //!< Front panel power switch
#define POWER_GOOD          AVR32_PIN_PA12                  //!< Power good from the PSU. High means supplies have stabilized.
#define RECONFIG_BUTTON     AVR32_PIN_PA13                  //!< Button on the board, active low when pressed

#define SPI_MOSI            AVR32_PIN_PA14                  //!< SPI output to the FPGA
#define SPI_SCLK            AVR32_PIN_PA15                  //!< SPI clock
#define FPGA_SPI_EN_0       AVR32_PIN_PA16                  //!< SPI enable for programming the FPGA
#define FPGA_SPI_EN_1       AVR32_PIN_PA17                  //!< SPI enable for the user interface to the FPGA
#define CDONE               AVR32_PIN_PA26                  //!< FPGA progamming interface
#define CRESET              AVR32_PIN_PA27                  //!< FPGA progamming interface

#define PWM1                AVR32_PIN_PA28                  //!< PWM for fan1 if 4-wire
#define FAN1                AVR32_PIN_PA19                  //!< Active high gate drive for fan1

#define PWR_ON              AVR32_PIN_PA22                  //!< Active low main power supply enable

/** @page led_connections LED Connections
 * LED connections on the Module 8 pin connector are:
 *      2 (+ve) to 1 = (POWER LED)
 *      4 (+ve) to 3 = (ACTIVITY LED)
 * ON BABY JET, the left LED maps to POWER_LED
 * ON BABY JET, the right LED maps to ACTIVITY_LED
 * ON Sierra LEDs TBD.
 * On Module assemblies, LEDs are active high
 * On EVK, LEDs are active low
 */

#define POWER_LED           AVR32_PIN_PA20                  //!< Module LED1 (EVK:LED0)
#define ACTIVITY_LED        AVR32_PIN_PA21                  //!< Module LED2 (EVK:LED1)

#define POWER_LED_ON        gpio_set_pin_high(POWER_LED);
#define POWER_LED_OFF       gpio_set_pin_low(POWER_LED);
#define ACTIVITY_LED_ON     gpio_set_pin_high(ACTIVITY_LED);
#define ACTIVITY_LED_OFF    gpio_set_pin_low(ACTIVITY_LED);

#define UART_TX             AVR32_PIN_PA23                  //!< UART output to ASIC's
#define UART_RX             AVR32_PIN_PA24                  //!< UART input from ASIC's

#define SPI_MISO            AVR32_PIN_PA25                  //!< SPI data from the slaves

#define CLOCK               AVR32_PIN_PA18                  //!< 8 Mhz external clock

#define SPARE_UP            AVR32_PIN_PA29                  //!< Spare pin on the Chain UP connector
#define SPARE_DOWN          AVR32_PIN_PA30                  //!< Spare pin on the chain down connector

#define V_MID               AVR32_PIN_PA31                  //!< Analog in. 25C when temp1..4 equals this.

#define A_12V_DETECT        AVR32_PIN_PB00                  //!< Active high detection of the A 12V supply. (rev 0/1 boards)
#define VR_HOT              AVR32_PIN_PB00                  //!< (IR-A boards)
#define B_12V_DETECT        AVR32_PIN_PB01                  //!< Active high detection of the B 12V supply. (rev 0/1 boards)
#define UNUSED_1            AVR32_PIN_PB01                  //!< (IR-A boards)

#define FPGA_INIT_BAR       AVR32_PIN_PB02                  //!< Active low indication that the FPGA has initialized

#define ENABLE_IO_VDD       AVR32_PIN_PB03                  //!< Driven high to enable IO_VDD to the device, otherwise floats
#define GOT_UP              AVR32_PIN_PB04                  //!< Driven low by the presence of an UP module
#define SPI_EN_2            AVR32_PIN_PB05                  //!< SPI enable for the external DAC (rev 0/1 boards)
#define UNUSED_2            AVR32_PIN_PB05                  //!< (IR-A boards)
#define HAVE_USB            AVR32_PIN_PB06                  //!< Drive this low if this module has a USB connection
#define USB_DOWN            AVR32_PIN_PB07                  //!< Driven low by the chain DOWN module if it has a USB host
#define GOT_DOWN            AVR32_PIN_PB08                  //!< Driven low by the presence of DOWN module

#define GOT_AC_POWER        AVR32_PIN_PB09                  //!< Active high indicates that AC power is present (VSTBY is there)

#define BOARD_ID_1          AVR32_PIN_PB10
#define BOARD_ID_2          AVR32_PIN_PB11
