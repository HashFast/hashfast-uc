/** @file boardid.c
 * @brief Determines board type based on GPIO pins.
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

#include <stdint.h>
#include <gpio.h>
#include <avr32/io.h>

#include "uc3b_peripherals.h"
#include "boardid.h"

boardidT boardid;

/**
 * Initializes board ID
 */
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
