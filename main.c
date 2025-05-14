// main.c

#include "stm32l476xx.h"
#include "ADC.h"
#include "SysClock.h"
#include "LED.h"        

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
    // convert however your sensor needs—here we just store voltage
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
