// main.c

#include "stm32l476xx.h"
#include "ADC.h"
#include "SysClock.h"
#include "LED.h"        
#include "ADC.h"

#define ADC_MAX_COUNT     4095.0f   // 12-bit
#define LED_PIN           5         // LD2 on PA5
#define BUTTON_PIN        13        // PC13 = User button

volatile float result;        // TDS raw ADC count
volatile float tds_ppm;       // TDS in ppm
volatile float temperature;   // temperature voltage (or converted)
volatile uint8_t sensorMode;  // 0 = TDS, 1 = Temperature
volatile float rawADC; 

// LD2 for blink feedback
void configure_LED2_pin(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    GPIOA->MODER &= ~(3UL << (2 * LED_PIN));
    GPIOA->MODER |=  (1UL << (2 * LED_PIN));
    GPIOA->OTYPER &= ~(1UL << LED_PIN);
    GPIOA->PUPDR &= ~(3UL << (2 * LED_PIN));
}

void configure_PA2() {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; // Enable GPIOA clock
	GPIOA->MODER &= ~(3 << (2 * 2));	 // Clear mode bits for PA2
	GPIOA->MODER |= (2 << (2 * 2));		 // Set alternate function mode
	GPIOA->AFR[0] &= ~(0b1111 << (2 * 4));	 // Clear AF bits for PA2 (AFR[0] = AFRL)
	GPIOA->AFR[0] |= (2 << (2 * 4));	 // AF2 (0010) = TIM5_CH3, leftshift 4 b/c its PA2
	GPIOA->OTYPER &= ~(1 << 2);			 // Push-pull
	GPIOA->PUPDR &= ~(3 << (2 * 2));	 // No pull-up/pull-down
}

// configure timer for 50hz PWM
// TIM5_CH3 on PA2
void configure_timer() {
    RCC->APB1ENR1 |= RCC_APB1ENR1_TM5EN;
    TIM5->PSC = 79
    TIM5->ARR = 999;
    TIM5->CCMR1 &= ~(0b111 << 4); // Clear OC3M bits (CH3)
    TIM5->CCMR1 |= (0b110 << 4);  // Set OC3M = 110 (PWM Mode 1)
    TIM5->CCMR1 |= TIM_CCMR1_OC3PE; // Enable CCR3 preload
    TIM5->CCER &= ~TIM_CCER_CC3P; // Active high polarity for servo
    TIM5->CCER |= TIM_CCER_CC3E;  // Enable output for TIM5_CH3
    TIM5->CR1 |= TIM_CR1_ARPE; // Enable ARR preload
    TIM5->EGR |= TIM_EGR_UG;   // Force update to load all shadow regs
    TIM5->CR1 |= TIM_CR1_CEN; // Enable the counter
}

// User button on PC13
void configure_button_pin(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
    GPIOC->MODER &= ~(3UL << (2 * BUTTON_PIN));
    GPIOC->PUPDR &= ~(3UL << (2 * BUTTON_PIN));
}

// simple toggle
static inline void blink_LED(void) {
    GPIOA->ODR ^= (1UL << LED_PIN);
}

// EXTI on PC13 -> interrupt on rising edge
void configure_EXTI(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    uint32_t idx   = BUTTON_PIN >> 2;
    uint32_t shift = (BUTTON_PIN & 3) * 4;
    SYSCFG->EXTICR[idx] &= ~(0xFUL << shift);
    SYSCFG->EXTICR[idx] |=  (0x2UL << shift); // 0x2 = port C

    EXTI->IMR1  |=  (1UL << BUTTON_PIN);
    EXTI->RTSR1 |=  (1UL << BUTTON_PIN);
    EXTI->FTSR1 &= ~(1UL << BUTTON_PIN);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR1 & (1UL << BUTTON_PIN)) {
        EXTI->PR1 |= (1UL << BUTTON_PIN);
        sensorMode ^= 1;
        blink_LED();
    }
}

// TDS sensor read from ADC1_IN6
void TDSSensor(void) {
    uint16_t raw = ADC1_Read();
		rawADC = raw;
    result = (float)raw;
    float volt = (result / ADC_MAX_COUNT) * 2.3f;
    tds_ppm = (22.06f  * volt * volt * volt)
            + (177.8f  * volt * volt)
            + (353.1f  * volt)
            + 20.67f;
}

// Temperature sensor read from ADC2_IN7
void Temperature(void) {
    uint16_t raw = ADC2_Read();
    // convert however your sensor needs�here we just store voltage
    float volt = (raw / ADC_MAX_COUNT) * 2.3f;
    rawADC = raw;
}

void Initialize(void) {
    System_Clock_Init();   // 80 MHz
    ADC_Init();            // sets up ADC1@IN6 and ADC2@IN7
    LEDs_Init();           // if you have external LEDs
    configure_LED2_pin();  // onboard LD2 for debug blink
    configure_button_pin();
    configure_EXTI();
    configure_PA2();
    configure_timer();
}

int main(void) {
    Initialize();
    sensorMode = 0;
    while (1) {
        if (sensorMode == 0) {
            TDSSensor();
        } else {
            Temperature();
        }
        // small delay if you like
        //SysTick_Delay_ms(200);
    }
}
