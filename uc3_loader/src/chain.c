/* chain.c */

/*
    Copyright (c) 2013 HashFast Technologies LLC
*/

#include <stdint.h>
#include <avr32/io.h>

#include "config.h"
#include "hf_loader.h"
#include "gpio.h"
#include "timers.h"
#include "chain.h"



int chainMaster;

int chainState;


static struct {
    int state;
} chain;


void chainInit(void) {

    chain.state = HF_LOADER_CHAIN_UNCONFIGURED;
    chainState = HF_LOADER_CHAIN_UNCONFIGURED;
}

void chainTask(void) {
    int gotDown;
    int gotUp;

    if (chainState == HF_LOADER_CHAIN_UNCONFIGURED) {
        gotDown = gpioPinValue(CONFIG_GPIO_GOT_DOWN);
        gotUp = gpioPinValue(CONFIG_GPIO_GOT_UP);
        if (gotDown && gotUp)
            chain.state = HF_LOADER_CHAIN_NONE;
        else if (!gotDown && !gotUp) {
            if (!gpioPinValue(CONFIG_GPIO_SPARE_UP) ||
                1 /* the loader has no need to differentiate between
                     middle and loopback modes. */ ) {
                gpioPinSet(CONFIG_GPIO_SPARE_DOWN, 0);
                chain.state = HF_LOADER_CHAIN_MIDDLE;
            } else
                chain.state = HF_LOADER_CHAIN_LOOPBACK;
        } else if (!gotDown && gotUp) {
            gpioPinSet(CONFIG_GPIO_SPARE_DOWN, 0);
            chain.state = HF_LOADER_CHAIN_OPEN_DOWN;
        } else
            chain.state = HF_LOADER_CHAIN_OPEN_UP;

        /* we don't care about differentiating between middle and loopback
           in the loader so we can decide this is stable and stop looking
           right away. */
        if (timersTick > 0) {
            chainState = chain.state;
            switch (chainState) {
            case HF_LOADER_CHAIN_OPEN_DOWN:
            case HF_LOADER_CHAIN_LOOPBACK:
            case HF_LOADER_CHAIN_NONE:
                chainMaster = 1;
                break;
            case HF_LOADER_CHAIN_OPEN_UP:
            case HF_LOADER_CHAIN_MIDDLE:
            default:
                chainMaster = 0;
                break;
            }
        }
    }
}

