// ADC.c

#include "ADC.h"
#include "LED.h"
#include "SysTimer.h"
#include "stm32l476xx.h"
#include <stdint.h>

//— Provide missing position/mask macros if needed —
#ifndef ADC_SQR1_SQ1_Pos
  #define ADC_SQR1_SQ1_Pos  6U
#endif
#ifndef ADC_SQR1_SQ1_Msk
  #define ADC_SQR1_SQ1_Msk (0x1FUL << ADC_SQR1_SQ1_Pos)
#endif
#ifndef ADC_SMPR1_SMP6_Pos
  #define ADC_SMPR1_SMP6_Pos 18U
#endif
#ifndef ADC_SMPR1_SMP6_Msk
  #define ADC_SMPR1_SMP6_Msk (0x7UL << ADC_SMPR1_SMP6_Pos)
#endif
#ifndef ADC_SMPR1_SMP7_Pos
  #define ADC_SMPR1_SMP7_Pos 21U
#endif
#ifndef ADC_SMPR1_SMP7_Msk
  #define ADC_SMPR1_SMP7_Msk (0x7UL << ADC_SMPR1_SMP7_Pos)
#endif

//— ADC Wakeup (from your original) —
void ADC_Wakeup(void) {
    int wait_time;
    if ((ADC1->CR & ADC_CR_DEEPPWD) == ADC_CR_DEEPPWD)
        ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC1->CR |= ADC_CR_ADVREGEN;
    wait_time = 20 * (80000000 / 1000000);
    while (wait_time--) { }
}

//— ADC Common Configuration —
void ADC_Common_Configuration(void) {
    SYSCFG->CFGR1 |= SYSCFG_CFGR1_BOOSTEN;
    ADC123_COMMON->CCR |= ADC_CCR_VREFEN;
    ADC123_COMMON->CCR &= ~ADC_CCR_PRESC;
    ADC123_COMMON->CCR |=  ADC_CCR_CKMODE_0;
    ADC123_COMMON->CCR &= ~ADC_CCR_DUAL;
    ADC123_COMMON->CCR |= 6U;
}

//— ADC Pin Initialization (PA1 & PA2) —
void ADC_Pin_Init(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    GPIOA->MODER |=  (3U << (2*1)) | (3U << (2*2));  // PA1, PA2 ? Analog
    GPIOA->PUPDR &= ~((3U << (2*1)) | (3U << (2*2)));
    GPIOA->ASCR  |=  GPIO_ASCR_EN_1 | GPIO_ASCR_EN_2;
}

//— Initialize ADC & default to IN6 (PA1) —
void ADC_Init(void) {
    RCC->AHB2ENR  |= RCC_AHB2ENR_ADCEN;
    RCC->AHB2RSTR |= RCC_AHB2RSTR_ADCRST;
    (void)RCC->AHB2RSTR;
    RCC->AHB2RSTR &= ~RCC_AHB2RSTR_ADCRST;

    ADC_Pin_Init();
    ADC_Common_Configuration();
    ADC_Wakeup();

    ADC1->CFGR   &= ~ADC_CFGR_RES;
    ADC1->CFGR   &= ~ADC_CFGR_ALIGN;
    ADC1->CFGR   &= ~ADC_CFGR_CONT;
    ADC1->SQR1   &= ~ADC_SQR1_L;

    // default channel = 6 (PA1)
    ADC1->SQR1   &= ~ADC_SQR1_SQ1_Msk;
    ADC1->SQR1   |=  (6U << ADC_SQR1_SQ1_Pos);
    ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_6;

    // sample times for IN6 & IN7
    ADC1->SMPR1  &= ~ADC_SMPR1_SMP6_Msk;
    ADC1->SMPR1  |=  (0U << ADC_SMPR1_SMP6_Pos);
    ADC1->SMPR1  &= ~ADC_SMPR1_SMP7_Msk;
    ADC1->SMPR1  |=  (0U << ADC_SMPR1_SMP7_Pos);

    ADC1->CFGR   &= ~ADC_CFGR_EXTEN;
    ADC1->CR     |=  ADC_CR_ADEN;
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0);
}

//— Switch regular-channel on the fly —
void ADC_Select_Channel(uint8_t channel) {
    ADC1->SQR1 &= ~ADC_SQR1_SQ1_Msk;
    ADC1->SQR1 |= ((uint32_t)channel << ADC_SQR1_SQ1_Pos);
    ADC1->DIFSEL &= ~(1U << channel);
}

//— Optional IRQ handler —
void ADC1_2_IRQHandler(void){
    NVIC_ClearPendingIRQ(ADC1_2_IRQn);
    if (ADC1->ISR & ADC_ISR_EOC) ADC1->ISR |= ADC_ISR_EOC;
    if (ADC1->ISR & ADC_ISR_EOS) ADC1->ISR |= ADC_ISR_EOS;
}
