/* gpio.h */

#ifndef _gpio_h
#define _gpio_h

#ifdef __cplusplus
extern "C" {
#endif


void gpioPinFunc(int pin, int func);

void gpioPinOutputEnable(int pin, int enable);

void gpioPinPullup(int pin, int enable);

void gpioPinSet(int pin, int value);

int gpioPinValue(int pin);


#ifdef __cplusplus
}
#endif

#endif /* _gpio_h */

