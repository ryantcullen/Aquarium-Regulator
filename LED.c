// LED.c
#include "LED.h"

// All 10 external LEDs mapped to GPIOC pins PC2, PC3, PC4, PC5, PC6, PC8, PC9, PC10, PC11, PC12
static GPIO_TypeDef* LED_PORT[NUM_EXTERNAL_LEDS] = {
    GPIOC, GPIOC, GPIOC, GPIOC, GPIOC,
    GPIOC, GPIOC, GPIOC, GPIOC, GPIOC
};

static const uint8_t LED_PIN_SRC[NUM_EXTERNAL_LEDS] = {
    2, 3, 4, 5, 6,
    8, 9, 10, 11, 12
};

static const uint16_t LED_PIN_MASK[NUM_EXTERNAL_LEDS] = {
    (1U << 2),  (1U << 3),  (1U << 4),  (1U << 5),  (1U << 6),
    (1U << 8),  (1U << 9),  (1U << 10), (1U << 11), (1U << 12)
};

void LEDs_Init(void) {
    // Enable clock for GPIOC
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
		
		uint8_t i = 0;
    for (i = 0; i < NUM_EXTERNAL_LEDS; i++) {
        GPIO_TypeDef* port = LED_PORT[i];
        uint8_t src = LED_PIN_SRC[i];
        uint16_t mask = LED_PIN_MASK[i];

        // Set mode to General-purpose output (MODER = 01)
        port->MODER &= ~(0x3U << (src * 2));
        port->MODER |=  (0x1U << (src * 2));
        // Push-pull output
        port->OTYPER &= ~(1U << src);
        // Low speed
        port->OSPEEDR &= ~(0x3U << (src * 2));
        // No pull-up/pull-down
        port->PUPDR &= ~(0x3U << (src * 2));
        // Initialize LED off via BRR
        port->BRR = mask;
    }
}

void LED_On(uint8_t led) {
    if (led < NUM_EXTERNAL_LEDS) {
        LED_PORT[led]->BSRR = LED_PIN_MASK[led];
    }
}

void LED_Off(uint8_t led) {
    if (led < NUM_EXTERNAL_LEDS) {
        LED_PORT[led]->BRR = LED_PIN_MASK[led];
    }
}

void LED_Toggle(uint8_t led) {
    if (led < NUM_EXTERNAL_LEDS) {
        LED_PORT[led]->ODR ^= LED_PIN_MASK[led];
    }
}
