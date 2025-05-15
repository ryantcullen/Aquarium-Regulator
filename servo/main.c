#include "stm32l476xx.h"

void pin_init()
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; // Enable GPIOA clock
	GPIOA->MODER &= ~(3 << (2 * 2));	 // Clear mode bits for PA2
	GPIOA->MODER |= (2 << (2 * 2));		 // Set alternate function mode
	GPIOA->AFR[0] &= ~(0xF << (2 * 4));	 // Clear AF bits for PA2 (AFR[0] = AFRL)
	GPIOA->AFR[0] |= (2 << (2 * 4));	 // AF2 (0010) = TIM5_CH3, leftshift 4 b/c its PA2
	GPIOA->OTYPER &= ~(1 << 2);			 // Push-pull
	GPIOA->PUPDR &= ~(3 << (2 * 2));	 // No pull-up/pull-down
}

void timer_init()
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN; // enable TIM5 (on APB1)

	// Prescaler calculations
	// Board clock is 4MHz
	// Want a ~20us tick time so we need to get a prescaler valule
	// s.t 4Mhz/x = 20us. 4,000,000 / 20 = 50,000 = 50Khz
	// Fcnt = F_clktimer / PSC + 1
	// so 50,000 = 4,000,000 / PSC + 1
	// 50(PSC) = 4,000,000 - 50k, PSC = 79

	// Similarly, we need to set the Auto-Reload Register (ARR) which
	// Determines the period (how often the timer resets
	// ARR = (f_timerclk / f_pwm * (PSC + 1)) -1
	// we want a 50hz PWM (to get a 20ms period)
	// so (4MHz / 50 * (79+1)) - 1 = 999

	// PSC and ARR define the 20ms PWM period (50Hz)
	// CCR3 sets the pulse-width within that period

	TIM5->PSC = 79;
	TIM5->ARR = 999;

	TIM5->CCMR1 &= ~(0b111 << 4);	// Clear OC3M bits (CH3)
	TIM5->CCMR1 |= (0b110 << 4);	// Set OC3M = 110 (PWM Mode 1) (bit 24 not needed in this case)
	TIM5->CCMR1 |= TIM_CCMR1_OC3PE; // Enable CCR3 preload

	TIM5->CCER &= ~TIM_CCER_CC3P; // Active high polarity for servo
	TIM5->CCER |= TIM_CCER_CC3E;  // Enable output for TIM5_CH3

	TIM5->CR1 |= TIM_CR1_ARPE; // Enable ARR preload
	TIM5->EGR |= TIM_EGR_UG;   // Force update to load all shadow regs

	TIM5->CR1 |= TIM_CR1_CEN; // Enable the counter

	// for SG90 servo, 1.0ms pulse = -90deg, 1.5 = 0deg, 2.0 = +90deg
	// since clock ticks at 20 us
	// 1ms pulse = 1000us / 20us = 50
	// 50 = -90deg
	// 75 = 0deg
	// 100 = +90deg
	TIM5->CCR3 = 75; // set center servo to center position
}

void delay_ms(uint32_t ms)
{
	// crude delay loop assuming 4 MHz system clock
	for (uint32_t i = 0; i < ms * 500; i++)
	{
		__NOP();
	}
}

void move_servo(uint32_t angle)
{
	int mod_angle = angle % 180;
	float angle_pct = mod_angle / 180;
	// for SG90 servo, 1.0ms pulse = -90deg, 1.5 = 0deg, 2.0 = +90deg
	// since clock ticks at 20 us
	// 1ms pulse = 1000us / 20us = 50
	// range is 0-180 easier to work with
	// 50 = 0
	// 75 = 90
	// 100 = 180
	// so to map computed angle to PW we multiply by 50
	// then add 50 (b/c our range is [50, 100])
	float true_angle = angle_pct * 50 + 50;
	TIM5->CCR3 = true_angle;
}

void test_servo_angles()
{
	move_servo(0); // -90deg
	delay_ms(1000);
	move_servo(90); // center
	delay_ms(1000);
	move_servo(180); // +90deg
	delay_ms(1000);
	move_servo(90); // back to center
	delay_ms(1000);
}

int main()
{
	pin_init();
	timer_init();

	while (1)
	{
		test_servo_angles();
	}
}
