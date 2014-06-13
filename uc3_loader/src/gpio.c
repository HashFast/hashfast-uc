/* gpio.c */

/*
    Copyright (c) 2013 HashFast Technologies LLC
*/

#include <stdint.h>
#include <avr32/io.h>

#include "config.h"
#include "gpio.h"


void gpioPinFunc(int pin, int func) {
    volatile avr32_gpio_port_t *port;
    uint32_t mask;

    port = &AVR32_GPIO.port[pin >> 5];
    mask = (uint32_t) 1 << (pin & 0x1f);

    switch (func) {
    case 0:
        port->pmr0c = mask;
        port->pmr1c = mask;
        break;
    case 1:
        port->pmr0s = mask;
        port->pmr1c = mask;
        break;
    case 2:
        port->pmr0c = mask;
        port->pmr1s = mask;
        break;
    case 3:
        port->pmr0s = mask;
        port->pmr1s = mask;
        break;
    }
    port->gperc = mask;
}

void gpioPinOutputEnable(int pin, int enable) {
    volatile avr32_gpio_port_t *port;
    uint32_t mask;

    port = &AVR32_GPIO.port[pin >> 5];
    mask = (uint32_t) 1 << (pin & 0x1f);

    if (enable)
        port->oders = mask;
    else
        port->oderc = mask;
}

void gpioPinPullup(int pin, int enable) {
    volatile avr32_gpio_port_t *port;
    uint32_t mask;

    port = &AVR32_GPIO.port[pin >> 5];
    mask = (uint32_t) 1 << (pin & 0x1f);

    if (enable)
        port->puers = mask;
    else
        port->puerc = mask;
}

void gpioPinSet(int pin, int value) {
    volatile avr32_gpio_port_t *port;
    uint32_t mask;

    port = &AVR32_GPIO.port[pin >> 5];
    mask = (uint32_t) 1 << (pin & 0x1f);

    if (value)
        port->ovrs = mask;
    else
        port->ovrc = mask;
}

int gpioPinValue(int pin) {
    volatile avr32_gpio_port_t *port;
    uint32_t mask;

    port = &AVR32_GPIO.port[pin >> 5];
    mask = (uint32_t) 1 << (pin & 0x1f);

    return (port->pvr & mask) ? 1 : 0;
}

