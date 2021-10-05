#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx.h"

#define TICK_MS	(uint32_t)1000
#define TICK_US	(uint32_t)(1000 * 1000)
#define PIN_ON	(uint8_t)1
#define PIN_OFF	(uint8_t)0

void system_core_cfg(void);
void gpio_init(void);
void set_pin_state(GPIO_TypeDef *GPIOx, uint8_t pin, uint8_t pin_state);
void us_delay(uint32_t delay_ticks);

extern volatile uint32_t us_ticks;

#endif /* __MAIN_H */

