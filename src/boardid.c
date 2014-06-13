/* boardid.c */

/*
    Copyright (c) 2014 HashFast Technologies LLC
*/

#include <stdint.h>
#include <gpio.h>
#include <avr32/io.h>

#include "uc3b_peripherals.h"
#include "boardid.h"


boardidT boardid;


void boardidInit(void) {
    uint8_t mask;

    gpio_configure_pin(BOARD_ID_1, GPIO_DIR_INPUT | GPIO_PULL_UP);
    gpio_configure_pin(BOARD_ID_2, GPIO_DIR_INPUT | GPIO_PULL_UP);
    mask = 0;
    if (gpio_pin_is_high(BOARD_ID_1))
        mask |= 1;
    if (gpio_pin_is_high(BOARD_ID_2))
        mask |= 2;
    switch (mask) {
    case 0:
        boardid = iraBID;
        break;
    case 2:
        boardid = habaneroBID;
        break;
    case 3:
        boardid = rev0_1_11_12_15BID;
        break;
    default:
        boardid = unknownBID;
        // TODO: Look up BID in user page
        break;
    }
}

