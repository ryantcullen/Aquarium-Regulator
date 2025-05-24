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
volatile float   waterLevel;     // calibrated water level
volatile float   tds_ppm;        // calibrated TDS
volatile float   temperature;    // calibrated temperature

//systick counter
volatile uint32_t msTicks = 0;     
volatile uint8_t servoToggleFlag = 0;
int currentAngle = 0;

void configure_SysTick(void) {
    // 80 MHz / 80000 = 1000 Hz = 1 ms tick
    SysTick->LOAD = 80000 - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void) {
    static uint32_t lastToggle = 0;
    msTicks++;

    static uint32_t lastServoToggle = 0;
    if ((msTicks - lastServoToggle) >= 5000) {
        servoToggleFlag = 1;
        lastServoToggle = msTicks;
    }
}

void delay(uint32_t ms) {
    uint32_t start = msTicks;
    while ((msTicks - start) < ms);
}

//� LD2 configuration for blink feedback �
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

void configure_PA6() {
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;  // Enable GPIOA clock
	GPIOA->MODER &= ~(3 << (6 * 2));	  // Clear mode bits for PA6
	GPIOA->MODER |=  (2 << (6 * 2));	  // Set alternate function mode
	GPIOA->AFR[0] &= ~(0b1111 << (6 * 4)); // Clear AF bits for PA6 (AFR[0] = AFRL)
	GPIOA->AFR[0] |=  (2 << (6 * 4));	  // AF2 (0010) = TIM3_CH1, leftshift 4 b/c its PA6
	GPIOA->OTYPER &= ~(1 << 6);			  // Push-pull
	GPIOA->PUPDR  &= ~(3 << (6 * 2));	  // No pull-up/pull-down
}


// configure timer for 50hz PWM
// TIM5_CH3 on PA2
void configure_timer() {
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;              // enable TIM3 clock
    TIM3->PSC = 1599;                                  // 80 MHz / (1599 + 1) = 50 kHz
    TIM3->ARR = 999;                                   // 50 kHz / (999 + 1) = 50 Hz (20 ms PWM period)
    TIM3->CCMR1 &= ~(0b111 << 4);                      // Clear OC1M bits (CH1)
    TIM3->CCMR1 |= (0b110 << 4);                       // Set OC1M = 110 (PWM Mode 1)
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE;                    // Enable CCR1 preload
    TIM3->CCER &= ~TIM_CCER_CC1P;                      // Active high polarity for servo
    TIM3->CCER |= TIM_CCER_CC1E;                       // Enable output for TIM3_CH1
    TIM3->CR1 |= TIM_CR1_ARPE;                         // Enable ARR preload
    TIM3->EGR |= TIM_EGR_UG;                           // Force update to load all shadow regs
    TIM3->CR1 |= TIM_CR1_CEN;                          // Enable the counter

    TIM3->CCR1 = 50;                                   // 180deg
}


//� Button on PC13 ? EXTI �
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
				LED_Off(0);
				LED_Off(1);
				LED_Off(3);
				LED_Off(4);
				LED_Off(5);
				LED_Off(6);
        blink_LED();
    }
}

//� TDS sensor routine (PA0 / IN5) �
void TDSSensor(void) {
    ADC_Select_Channel(CH_TDS);
    rawADC = ADC_Read();
    float volt = (rawADC / ADC_MAX_COUNT) * 2.3f;
		// Direct raw-count to ppm conversion
		tds_ppm = (1.28603e-8f * rawADC * rawADC * rawADC)
              + (3.17711e-5f * rawADC * rawADC)
              + (0.2155523f * rawADC)
              + 18.99510f;


	
    if (tds_ppm > 200.0f) {
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

//� Water level sensor routine (PA1 / IN6) �
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

//� Temperature sensor routine (PA4 / IN9) �
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



void move_servo(uint32_t angle) {
    if (angle > 180) angle = 180;  // Clamp input to valid range
    float ticks = (angle / 180.0f) * 50.0f + 50.0f;

    TIM3->CCR1 = (uint32_t)ticks;
}

void Initialize(void) {
    System_Clock_Init();    // 80 MHz
    ADC_Init();             // single-ADC setup for channels 5, 6, 9
    LEDs_Init();            // if you have external LEDs
    configure_LED2_pin();   // onboard LD2
    configure_button_pin();	// onboard UB1
    configure_EXTI();				// interrupt for button
    configure_SysTick();		// SysTick to control motor interval
    configure_PA6();				// configure PA6 for PWM
    configure_timer();			// configure timer
    sensorMode = 0;         // start in TDS mode
}

int main(void) {
    Initialize();
    while (1) {
        switch (sensorMode) {
            case 0: TDSSensor(); break;
            case 1: WaterLevel(); break;
            case 2: Temperature(); break;
        }

        
				if (servoToggleFlag) {
						if (currentAngle == 0) {
								move_servo(180);
						currentAngle = 180;
						} else {
								move_servo(0);
								currentAngle = 0;
						}
						servoToggleFlag = 0;

				}   

    }
}
