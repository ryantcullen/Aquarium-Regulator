// ADC.c

#include "ADC.h"
#include "stm32l476xx.h"
#include <stdint.h>

/**
 * ADC.c
 *  - ADC1 on PA1 ? IN6   (TDS sensor)
 *  - ADC2 on PA4 ? IN9   (temperature or second sensor)
 *
 * APIs:
 *   void     ADC_Init(void);
 *   uint16_t ADC1_Read(void);
 *   uint16_t ADC2_Read(void);
 */

//-------------------------------------------------------------------------------------------
//  Wake up & calibrate a single ADC (ADC1 or ADC2)
//-------------------------------------------------------------------------------------------
static void ADC_Startup(ADC_TypeDef *ADCx) {
    int delay;
    // Exit deep-power-down
    if (ADCx->CR & ADC_CR_DEEPPWD) {
        ADCx->CR &= ~ADC_CR_DEEPPWD;
    }
    // Enable internal regulator
    ADCx->CR |= ADC_CR_ADVREGEN;
    // Wait ~20µs for regulator startup (@80MHz)
    delay = 20 * (80000000/1000000);
    while (delay--) {}
    // Calibrate
    ADCx->CR |= ADC_CR_ADCAL;
    while (ADCx->CR & ADC_CR_ADCAL) {}
}

//-------------------------------------------------------------------------------------------
//  Shared ADC common settings (all ADC1–3)
//-------------------------------------------------------------------------------------------
void ADC_Common_Configuration(void) {
    SYSCFG->CFGR1  |= SYSCFG_CFGR1_BOOSTEN;   // boost analog switches
    ADC123_COMMON->CCR |= ADC_CCR_VREFEN;     // enable VREFINT
    // clock = HCLK/1
    ADC123_COMMON->CCR &= ~ADC_CCR_CKMODE;
    ADC123_COMMON->CCR |=  ADC_CCR_CKMODE_0;
    ADC123_COMMON->CCR &= ~ADC_CCR_PRESC;     // presc = /1
    // independent mode
    ADC123_COMMON->CCR &= ~ADC_CCR_DUAL;
}

//-------------------------------------------------------------------------------------------
//  Configure PA1, PA2, PA4 as analog inputs
//-------------------------------------------------------------------------------------------
void ADC_Pin_Init(void) {
    // enable GPIOA clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    // PA1, PA2, PA4 -> analog mode (11), no pull-up/down
    GPIOA->MODER |=  (3U<<(2*1)) | (3U<<(2*2)) | (3U<<(2*4));
    GPIOA->PUPDR &= ~((3U<<(2*1)) | (3U<<(2*2)) | (3U<<(2*4)));
    // connect analog switch
    GPIOA->ASCR  |= (1U<<1) | (1U<<2) | (1U<<4);
}

//-------------------------------------------------------------------------------------------
//  Initialize ADC1 (IN6) & ADC2 (IN9)
//-------------------------------------------------------------------------------------------
void ADC_Init(void) {
    // enable & reset ADC common clock
    RCC->AHB2ENR  |= RCC_AHB2ENR_ADCEN;
    RCC->AHB2RSTR |= RCC_AHB2RSTR_ADCRST;
    (void)RCC->AHB2RSTR;
    RCC->AHB2RSTR &= ~RCC_AHB2RSTR_ADCRST;

    // shared setup
    ADC_Pin_Init();
    ADC_Common_Configuration();

    // ----- ADC1 (PA1 ? IN6) -----
    ADC_Startup(ADC1);
    ADC1->CFGR &= ~(ADC_CFGR_RES | ADC_CFGR_ALIGN | ADC_CFGR_CONT);
    ADC1->SQR1 &= ~0x0000000FU;          // length = 1
    ADC1->SQR1 |=  (6U << 6);            // SQ1 = channel 6
    ADC1->DIFSEL &= ~(1U << 6);          // single-ended
    // sample time 247.5 cycles on SMP6 ([20:18])
    ADC1->SMPR1 &= ~((0x7UL<<18)|(0x7UL<<21));
    ADC1->SMPR1 |=  (6U << 18);
    ADC1->CFGR &= ~ADC_CFGR_EXTEN;       // SW start only
    ADC1->CR   |=  ADC_CR_ADEN;          // enable ADC1
    while (!(ADC1->ISR & ADC_ISR_ADRDY)) {}

    // ----- ADC2 (PA4 ? IN9) -----
    ADC_Startup(ADC2);
    ADC2->CFGR &= ~(ADC_CFGR_RES | ADC_CFGR_ALIGN | ADC_CFGR_CONT);
    ADC2->SQR1 &= ~(0x1FUL << 6);
    ADC2->SQR1 |=  (9U << 6);            // SQ1 = channel 9
    ADC2->DIFSEL &= ~(1U << 9);
    // sample time 247.5 cycles on SMP9 ([29:27])
    ADC2->SMPR1 &= ~(0x7UL << 27);
    ADC2->SMPR1 |=  (6U << 27);
    ADC2->CFGR &= ~ADC_CFGR_EXTEN;
    ADC2->CR   |=  ADC_CR_ADEN;          // enable ADC2
    while (!(ADC2->ISR & ADC_ISR_ADRDY)) {}
}

//-------------------------------------------------------------------------------------------
//  Read one-shot from ADC1 (IN6)
//-------------------------------------------------------------------------------------------
uint16_t ADC1_Read(void) {
    ADC1->ISR |= ADC_ISR_EOC;           // clear old flag
    ADC1->CR  |= ADC_CR_ADSTART;        // start conversion
    while (!(ADC1->ISR & ADC_ISR_EOC)) {}
    ADC1->ISR |= ADC_ISR_EOC;           // clear flag
    return (uint16_t)ADC1->DR;
}

//-------------------------------------------------------------------------------------------
//  Read one-shot from ADC2 (IN9)
//-------------------------------------------------------------------------------------------
uint16_t ADC2_Read(void) {
    ADC2->ISR |= ADC_ISR_EOC;
    ADC2->CR  |= ADC_CR_ADSTART;
    while (!(ADC2->ISR & ADC_ISR_EOC)) {}
    ADC2->ISR |= ADC_ISR_EOC;
    return (uint16_t)ADC2->DR;
}

//-------------------------------------------------------------------------------------------
//  Shared ADC1/2 IRQ handler (optional)
//-------------------------------------------------------------------------------------------
void ADC1_2_IRQHandler(void) {
    if (ADC1->ISR & ADC_ISR_EOC) ADC1->ISR |= ADC_ISR_EOC;
    if (ADC2->ISR & ADC_ISR_EOC) ADC2->ISR |= ADC_ISR_EOC;
}
