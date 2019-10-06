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

#include "Timer.h"
#include "../state.h"
#include "servo.h"


#define FCLK	72000000
#define PRESC	72

#define MOTOR_PIN	GPIO_Pin_5
#define MOTOR_TIME	200			// in ms


float k1 = 20;
float b1 = 40;

TIM_TimeBaseInitTypeDef timer;
TIM_OCInitTypeDef	timerPWM;
GPIO_InitTypeDef	motor;



void servo_init(void) {

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    GPIO_InitTypeDef port;
    GPIO_StructInit(&port);
    port.GPIO_Mode = GPIO_Mode_AF_PP;
    port.GPIO_Pin = GPIO_Pin_6;
    port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &port);

    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = PRESC;
    timer.TIM_Period = FCLK / (PRESC * 50);
    timer.TIM_ClockDivision = 0;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &timer);

    TIM_OCStructInit(&timerPWM);
    timerPWM.TIM_Pulse = 1000;
    timerPWM.TIM_OCMode = TIM_OCMode_PWM1;
    timerPWM.TIM_OutputState = TIM_OutputState_Enable;
    timerPWM.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM4, &timerPWM);

    TIM_Cmd(TIM4, ENABLE);
}


void servo_start_pos(void)
{
	TIM4->CCR1 = SERVO_MIN;
}


void servo_next_pos(void)
{
	static int cnt = 0;
	TIM4->CCR1 = SERVO_MIN + SERVO_STEP * cnt;
	cnt = (cnt + 1) % SERVO_STEPS;
}


void motor_init(void)
{
	GPIO_StructInit(&motor);
	motor.GPIO_Mode = GPIO_Mode_Out_PP;
	motor.GPIO_Pin = GPIO_Pin_5;
	motor.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &motor);

	GPIO_WriteBit(GPIOB, MOTOR_PIN, Bit_RESET);
}

void motor_next_pos(void)
{
	GPIO_WriteBit(GPIOB, MOTOR_PIN, Bit_SET);
	timer_sleep(MOTOR_TIME);
	GPIO_WriteBit(GPIOB, MOTOR_PIN, Bit_RESET);
}

