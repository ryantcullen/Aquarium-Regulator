// main.c

#include "stm32l476xx.h"
#include "ADC.h"
#include "SysClock.h"
#include "LED.h"        

#define ADC_MAX_COUNT   4095.0f   // 12-bit resolution
#define LED_PIN         5         // LD2 (PA5)
#define BUTTON_PIN      13        // User button (PC13)

// ADC channels for single-ADC mode switching
#define CH_TDS          5         // PA0 ? ADC12_IN5 (TDS)
#define CH_WATER        6         // PA1 ? ADC12_IN6 (water level)
#define CH_TEMP         9         // PA4 ? ADC12_IN9 (Temperature)

volatile uint8_t sensorMode;      // 0 = Water, 1 = TDS, 2 = Temperature
volatile float   rawADC;         // latest raw ADC count
volatile float   waterLevel;     // placeholder for water level value
volatile float   tds_ppm;        // computed TDS
volatile float   temperature;    // computed temperature voltage (or converted)

//— LD2 configuration for blink feedback —
void configure_LED2_pin(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    GPIOA->MODER   &= ~(3UL << (2 * LED_PIN));
    GPIOA->MODER   |=  (1UL << (2 * LED_PIN));
    GPIOA->OTYPER  &= ~(1UL << LED_PIN);
    GPIOA->PUPDR   &= ~(3UL << (2 * LED_PIN));
}
static inline void blink_LED(void) {
    GPIOA->ODR ^= (1UL << LED_PIN);
}

//— Button on PC13 ? EXTI —
void configure_button_pin(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
    GPIOC->MODER &= ~(3UL << (2 * BUTTON_PIN));
    GPIOC->PUPDR &= ~(3UL << (2 * BUTTON_PIN));
}
void configure_EXTI(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    uint32_t idx   = BUTTON_PIN >> 2;
    uint32_t shift = (BUTTON_PIN & 3) * 4;
    SYSCFG->EXTICR[idx] &= ~(0xFUL << shift);
    SYSCFG->EXTICR[idx] |=  (0x2UL << shift);  // Port C
    EXTI->IMR1  |=  (1UL << BUTTON_PIN);
    EXTI->RTSR1 |=  (1UL << BUTTON_PIN);
    EXTI->FTSR1 &= ~(1UL << BUTTON_PIN);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}
void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR1 & (1UL << BUTTON_PIN)) {
        EXTI->PR1 |= (1UL << BUTTON_PIN);
        // cycle through 0,1,2
        sensorMode = (sensorMode + 1) % 3;
        blink_LED();
				LED_Off(0);
				LED_Off(1);
				LED_Off(3);
				LED_Off(4);
				LED_Off(5);
				LED_Off(6);
				
    }
}

//— TDS sensor routine (PA0 / IN5) —
void TDSSensor(void) {
    ADC_Select_Channel(CH_TDS);
    rawADC = ADC_Read();
    float volt = (rawADC / ADC_MAX_COUNT) * 2.3f;
    tds_ppm =  22.06f*volt*volt*volt
            +177.8f *volt*volt
            +353.1f *volt
            +20.67f;

	
    if (tds_ppm > 300.0f) {
        LED_On(0);
		}
		if (tds_ppm > 500.0f) {
        LED_On(1);
		}
    if (tds_ppm < 200.0f) {
				LED_Off(0);
				LED_Off(1);
		}

}

//— Water level sensor routine (PA1 / IN6) —
void WaterLevel(void) {
    ADC_Select_Channel(CH_WATER);
    rawADC = ADC_Read();
		if (rawADC < 500){
				LED_On(3);
				LED_On(4);
				}
				else if (rawADC < 3000){
				LED_On(3);
				LED_On(4);
				}
				else if (rawADC < 3700){
				LED_On(3);
				LED_Off(4);
				}
				else{
				LED_Off(3);
				LED_Off(4);
				}
}

//— Temperature sensor routine (PA4 / IN9) —
void Temperature(void) {
    ADC_Select_Channel(CH_TEMP);
    rawADC = ADC_Read();	
	  
	  // Convert raw counts to voltage (in volts)
	  float voltage = (rawADC / ADC_MAX_COUNT) * 2.2f; // gets 49-50F
	
	  // Calculate temperature in Celsius using the LMT86 transfer function
    float temp_c = (voltage - 1) / -0.11f;
	
	  // Convert Celsius to Fahrenheit
    float temp_f = (temp_c * 9.0f/5.0f) + 32.0f;
	
	  // Single-point calibration offset
    temperature = temp_f + 23.0f;
		
		if (temperature < 60 ){
			LED_On(5);
			LED_Off(6);
		}
		else if (temperature > 60 ){
			LED_Off(5);
			LED_On(6);
		}
		else{
			LED_Off(5);
			LED_Off(6);			
		}
}

void Initialize(void) {
    System_Clock_Init();    // 80 MHz
    ADC_Init();             // single-ADC setup for channels 5, 6, 9
    LEDs_Init();            // if you have external LEDs
    configure_LED2_pin();   // onboard LD2
    configure_button_pin();
    configure_EXTI();
    sensorMode = 0;         // start in water-level mode
}

int main(void) {
    Initialize();
    while (1) {
        switch (sensorMode) {
            case 0: TDSSensor(); break;
            case 1: WaterLevel(); break;
            case 2: Temperature(); break;
        }
        // optionally delay to slow down readings
        // SysTick_Delay_ms(200);
    }
}
