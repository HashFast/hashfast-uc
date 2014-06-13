/* tty.c */

/*
    Copyright (c) 2014 HashFast Technologies LLC
*/

#include <stdint.h>
#include <avr32/io.h>
#include <sysclk.h>
#include <gpio.h>

#include "tty.h"


#define TTY_USART     AVR32_USART2
#define TTY_BAUD      115200


void ttyInit(void) {
    uint32_t div;

    gpio_configure_pin(AVR32_USART2_RXD_0_1_PIN,
                       GPIO_DIR_INPUT | GPIO_PULL_UP);
    gpio_configure_pin(AVR32_USART2_TXD_0_1_PIN,
                       GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
    gpio_enable_module_pin(AVR32_USART2_RXD_0_1_PIN,
                           AVR32_USART2_RXD_0_1_FUNCTION);
    gpio_enable_module_pin(AVR32_USART2_TXD_0_1_PIN,
                           AVR32_USART2_TXD_0_1_FUNCTION);

    div = (sysclk_get_pba_hz() + (TTY_BAUD * (16 / 8) / 2)) /
          (TTY_BAUD * (16 / 8));
    TTY_USART.brgr = ((div >> 3) << AVR32_USART_BRGR_CD) |
                     ((div & 7) << AVR32_USART_BRGR_FP);
    TTY_USART.mr = (AVR32_USART_MR_OVER_X16 << AVR32_USART_MR_OVER) |
                   (AVR32_USART_MR_MSBF_LSBF << AVR32_USART_MR_MSBF) |
                   (AVR32_USART_MR_CHMODE_NORMAL << AVR32_USART_MR_CHMODE) |
                   (AVR32_USART_MR_NBSTOP_1 << AVR32_USART_MR_NBSTOP) |
                   (AVR32_USART_MR_PAR_NONE << AVR32_USART_MR_PAR) |
                   (AVR32_USART_MR_CHRL_8 << AVR32_USART_MR_CHRL) |
                   (AVR32_USART_MR_USCLKS_MCK << AVR32_USART_MR_USCLKS) |
                   (AVR32_USART_MR_MODE_NORMAL << AVR32_USART_MR_MODE);
    TTY_USART.idr = ~(uint32_t) 0;
    TTY_USART.cr = AVR32_USART_CR_RXEN_MASK | AVR32_USART_CR_TXEN_MASK;
}

int ttyRead(void) {
    int c;

    if (TTY_USART.csr & AVR32_USART_CSR_RXRDY_MASK)
        c = TTY_USART.rhr & AVR32_USART_RHR_RXCHR_MASK;
    else
        c = -1;

    return c;
}

int ttyWrite(char c) {
    int r;

    if (TTY_USART.csr & AVR32_USART_CSR_TXRDY_MASK) {
        TTY_USART.thr = (unsigned char) c;
        r = (unsigned char) c;
    } else
        r = -1;

    return r;
}

