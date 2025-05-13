#ifndef __STM32L476G_DISCOVERY_LED_H
#define __STM32L476G_DISCOVERY_LED_H

#include "stm32l476xx.h"
#include <stdint.h>

#define NUM_EXTERNAL_LEDS 10

// Initialize all external LEDs on specified pins as outputs and turn them off
void LEDs_Init(void);

// Control functions for individual LEDs (0 to NUM_EXTERNAL_LEDS-1)
void LED_On(uint8_t led);
void LED_Off(uint8_t led);
void LED_Toggle(uint8_t led);

#endif /* __STM32L476G_DISCOVERY_LED_H */
