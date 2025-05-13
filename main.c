// main.c

#include "stm32l476xx.h"
#include "ADC.h"
#include "SysClock.h"
#include "LED.h"        

#define ADC_MAX_COUNT    4095.0f
#define SENSOR_MAX_VOLT  2.3f

// On-board LED (LD2) on PA5
#define LED_PIN       5
// On-board USER button (B1/UB1) on PC13
#define BUTTON_PIN   13

// Variables
volatile float   ADC_1;
volatile float   ADC_2;
volatile float   ADC_3;
volatile float   voltage;
volatile float   tds_ppm;
volatile uint8_t sensorMode = 0;    // 0 = TDS, 1 = Temperature

// GPIO + EXTI prototypes
void configure_LED_pin(void);
void configure_button_pin(void);
void configure_EXTI(void);

// LD2 for debugging
void configure_LED2_pin(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    GPIOA->MODER   &= ~(3UL << (2 * LED_PIN));
    GPIOA->MODER   |=  (1UL << (2 * LED_PIN));
    GPIOA->OTYPER  &= ~(1UL << LED_PIN);
    GPIOA->PUPDR   &= ~(3UL << (2 * LED_PIN));
}

// UB1 to toggle sensor modes
void configure_button_pin(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
    GPIOC->MODER   &= ~(3UL << (2 * BUTTON_PIN));
    GPIOC->PUPDR   &= ~(3UL << (2 * BUTTON_PIN));
}

// LD2 toggle
static inline void blink_LED(void) {
    GPIOA->ODR ^= (1UL << LED_PIN);
}

// Configure EXTi so that UB1 triggers interrupt
void configure_EXTI(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    uint32_t idx   = BUTTON_PIN >> 2;
    uint32_t shift = (BUTTON_PIN & 3) * 4;
    SYSCFG->EXTICR[idx] &= ~(0xFUL << shift);
    SYSCFG->EXTICR[idx] |=  (0x2UL << shift);
    EXTI->IMR1  |=  (1UL << BUTTON_PIN);
    EXTI->RTSR1 |=  (1UL << BUTTON_PIN);
    EXTI->FTSR1 &= ~(1UL << BUTTON_PIN);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

// EXTI handler toggles sensorMode and blinks LD2
void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR1 & (1UL << BUTTON_PIN)) {
        EXTI->PR1 |= (1UL << BUTTON_PIN);
        sensorMode ^= 1;
        blink_LED();
    }
}
 
// Sensor routines

// TDS Sensor
void TDSSensor(void) {
    // select PA1 ? ADC12_IN6
    ADC_Select_Channel(6);
    ADC1->CR |= ADC_CR_ADSTART;
    while (!(ADC123_COMMON->CSR & ADC_CSR_EOC_MST));
    ADC_1 = ADC1->DR;

    voltage = (ADC_1 / ADC_MAX_COUNT) * SENSOR_MAX_VOLT;
    tds_ppm  = (22.06f  * voltage * voltage * voltage)
             + (177.8f * voltage * voltage)
             + (353.1f * voltage)
             + 20.67f;

    if (tds_ppm > 200.0f) {
        LED_On(3);
    }
}

//Water Level Sensor
void WaterLevel(void) {
    // always on whatever pin you have wired
    ADC_Select_Channel(8);
    ADC1->CR |= ADC_CR_ADSTART;
    while (!(ADC123_COMMON->CSR & ADC_CSR_EOC_MST));
    ADC_2 = ADC1->DR;

    if (ADC_2 < 500) {
        LED_On(0); LED_On(1); LED_On(2);
    }
    else if (ADC_2 < 3600) {
        LED_On(0); LED_On(1); LED_Off(2);
    }
    else if (ADC_2 < 4000) {
        LED_On(0); LED_Off(1); LED_Off(2);
    }
    else {
        LED_Off(0); LED_Off(1); LED_Off(2);
    }
}


// Temperature Sensor
void Temperature(void) {
    // select PA2 ? ADC12_IN7
    ADC_Select_Channel(7);
    ADC1->CR |= ADC_CR_ADSTART;
    while (!(ADC123_COMMON->CSR & ADC_CSR_EOC_MST));
    ADC_3 = ADC1->DR;

    // convert `result` ? volts ? °C here
    // e.g. voltage = (result/ADC_MAX_COUNT)*V_REF;
    //      temp_c  = (voltage - 0.5f) * 100.0f;
}

// Initialization and pin configuration
void Initialize()
{
	  System_Clock_Init();    //clock
    ADC_Init();						  //ADC
    LEDs_Init();					  // External LEDs
    configure_LED2_pin();   // LD2 (for debugging)
    configure_button_pin(); // UB1 to toggle between sensor modes
    configure_EXTI();				// EXTI interrupt
}

int main(void) {
		
		Initialize();

    while (1) {
        //WaterLevel();
        if (sensorMode == 0) {
            TDSSensor();
        } else {
            Temperature();
        }
    }
}
