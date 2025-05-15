#include "stm32l476xx.h"

void move_servo(uint32_t angle)
{
	int mod_angle = angle % 180;
	float angle_pct = mod_angle / 180.0f;
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