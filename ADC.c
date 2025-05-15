// ADC.c

#include "ADC.h"
#include "stm32l476xx.h"
#include <stdint.h>

/**
 * ADC.c
 * Single-ADC implementation that switches between:
 *   – PA0 ? ADC12_IN5 (channel 5)
 *   – PA1 ? ADC12_IN6 (channel 6)
 *   – PA4 ? ADC12_IN9 (channel 9)
 *
 * APIs:
 *   void     ADC_Init(void);
 *   void     ADC_Select_Channel(uint8_t ch);
 *   uint16_t ADC_Read(void);
 */

//-------------------------------------------------------------------------------------------
//  Wake up & calibrate ADC1
//-------------------------------------------------------------------------------------------
static void ADC1_Startup(void) {
    int delay;

    // Exit deep-power-down
    if (ADC1->CR & ADC_CR_DEEPPWD) {
        ADC1->CR &= ~ADC_CR_DEEPPWD;
    }
    // Enable internal regulator
    ADC1->CR |= ADC_CR_ADVREGEN;
    // Wait ~20 µs for regulator startup (@80 MHz)
    delay = 20 * (80000000 / 1000000);
    while (delay--) {}
    // Self-calibration
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL) {}
}

//-------------------------------------------------------------------------------------------
//  ADC common settings (all ADC1–3)
//-------------------------------------------------------------------------------------------
void ADC_Common_Configuration(void) {
    // Boost analog switches at low VDDA
    SYSCFG->CFGR1   |= SYSCFG_CFGR1_BOOSTEN;
    // Enable VREFINT
    ADC123_COMMON->CCR |= ADC_CCR_VREFEN;
    // Clock = HCLK/1
    ADC123_COMMON->CCR &= ~ADC_CCR_CKMODE;
    ADC123_COMMON->CCR |=  ADC_CCR_CKMODE_0;
    // Prescaler = /1
    ADC123_COMMON->CCR &= ~ADC_CCR_PRESC;
    // Independent mode
    ADC123_COMMON->CCR &= ~ADC_CCR_DUAL;
}

//-------------------------------------------------------------------------------------------
//  Configure PA0, PA1, PA4 as analog inputs
//-------------------------------------------------------------------------------------------
void ADC_Pin_Init(void) {
    // Enable GPIOA clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    // PA0, PA1, PA4 ? analog mode (MODER=11), no pull (PUPDR=00)
    GPIOA->MODER |=  
        (3U << (2*0)) |  // PA0
        (3U << (2*1)) |  // PA1
        (3U << (2*4));   // PA4

    GPIOA->PUPDR &= ~(
        (3U << (2*0)) |
        (3U << (2*1)) |
        (3U << (2*4)));

    // Connect analog switches
    GPIOA->ASCR  |= 
        (1U << 0) |
        (1U << 1) |
        (1U << 4);
}

//-------------------------------------------------------------------------------------------
//  Initialize ADC1 for channels 5, 6, and 9
//-------------------------------------------------------------------------------------------
void ADC_Init(void) {
    // 1) Enable & reset ADC common clock
    RCC->AHB2ENR  |= RCC_AHB2ENR_ADCEN;
    RCC->AHB2RSTR |= RCC_AHB2RSTR_ADCRST;
    (void)RCC->AHB2RSTR;
    RCC->AHB2RSTR &= ~RCC_AHB2RSTR_ADCRST;

    // 2) Shared setup
    ADC_Pin_Init();
    ADC_Common_Configuration();

    // 3) Wake up & calibrate
    ADC1_Startup();

    // 4) Configure ADC1
    ADC1->CFGR &= ~(ADC_CFGR_RES | ADC_CFGR_ALIGN | ADC_CFGR_CONT);
    ADC1->SQR1 &= ~0x0000000FU;           // regular sequence length = 1
    // default to channel 5 (PA0)
    ADC1->SQR1 |=  (5U << 6);             // SQ1[4:0] = 5
    ADC1->DIFSEL &= ~(1U << 5);           // single-ended IN5

    // 5) Sample-time: 247.5 cycles for channels 5,6,9
    ADC1->SMPR1 &= ~(
        (0x7UL << 15) |   // SMP5 bits [17:15]
        (0x7UL << 18) |   // SMP6 bits [20:18]
        (0x7UL << 27));   // SMP9 bits [29:27]
    ADC1->SMPR1 |= 
        (6U << 15) |      // IN5
        (6U << 18) |      // IN6
        (6U << 27);       // IN9

    // 6) Software trigger only
    ADC1->CFGR &= ~ADC_CFGR_EXTEN;

    // 7) Enable ADC1
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY)) {}
}

//-------------------------------------------------------------------------------------------
//  Switch ADC1 between channel 5, 6, or 9
//-------------------------------------------------------------------------------------------
void ADC_Select_Channel(uint8_t ch) {
    if (ch!=5 && ch!=6 && ch!=9) return;

    // wait for any ongoing conversion to finish
    while (ADC1->CR & ADC_CR_ADSTART) {}

    // set SQ1[4:0] = ch
    ADC1->SQR1 &= ~(0x1FUL << 6);
    ADC1->SQR1 |=  (ch << 6);

    // single-ended
    ADC1->DIFSEL &= ~(1U << ch);

    // adjust sample-time just for this channel
    ADC1->SMPR1 &= ~(
        (0x7UL << 15) |
        (0x7UL << 18) |
        (0x7UL << 27));
    if (ch == 5) {
        ADC1->SMPR1 |= (6U << 15);
    } else if (ch == 6) {
        ADC1->SMPR1 |= (6U << 18);
    } else {
        ADC1->SMPR1 |= (6U << 27);
    }
}

//-------------------------------------------------------------------------------------------
//  Start a single conversion on the selected channel, wait for EOC, return data
//-------------------------------------------------------------------------------------------
uint16_t ADC_Read(void) {
    // clear old flag
    ADC1->ISR |= ADC_ISR_EOC;
    // start conversion
    ADC1->CR  |= ADC_CR_ADSTART;
    // wait for end of conversion
    while (!(ADC1->ISR & ADC_ISR_EOC)) {}
    // clear flag
    ADC1->ISR |= ADC_ISR_ADRDY;
    // read result
    return (uint16_t)ADC1->DR;
}
