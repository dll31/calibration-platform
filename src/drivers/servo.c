/*
 * servo.c
 *
 *  Created on: Jul 30, 2019
 *      Author: anton
 */

#include <stm32f10x.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_gpio.h>
#include <stdlib.h>
#include "math.h"

#include "../state.h"

float k1 = 20;
float b1 = 40;


void servo_init(void){
	GPIO_InitTypeDef port;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, 1);

	port.GPIO_Mode = GPIO_Mode_AF_PP;
	port.GPIO_Pin = GPIO_Pin_9;
	port.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &port);
}

void servo_timer_init(void){
	servo_timer.TIM_Prescaler = 1000;
	servo_timer.TIM_Period = 63400;
	servo_timer.TIM_CounterMode = TIM_CounterMode_Up;
	servo_timer.TIM_RepetitionCounter = 0;

	TIM2->CCER |= (TIM_CCER_CC1E);

	TIM_TimeBaseInit(TIM1, &servo_timer);

	TIM_Cmd(TIM1, 1);
}

void change_pulse(uint32_t pulse){
	servo_timer_PWM.TIM_Pulse = pulse;
	servo_timer_PWM.TIM_OCMode = TIM_OCMode_PWM1;
	servo_timer_PWM.TIM_OutputState = TIM_OutputState_Enable;
	servo_timer_PWM.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM1, &servo_timer_PWM);

	TIM1->CR1 |= TIM_CR1_CEN;
}

void servo_rotate(double angle){
	uint32_t pulse = (uint32_t)round(k1 * angle + b1);
	change_pulse(pulse);
}
