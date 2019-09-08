/*
 * state.h
 *
 *  Created on: Jul 30, 2019
 *      Author: anton
 */

#ifndef STATE_H_
#define STATE_H_

#include <stm32f10x.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_gpio.h>
#include <stdlib.h>

GPIO_InitTypeDef port;
TIM_TimeBaseInitTypeDef timer;
TIM_OCInitTypeDef timerPWM;

//TIM_TimeBaseInitTypeDef servo_timer;
//TIM_OCInitTypeDef servo_timer_PWM;
//GPIO_InitTypeDef port;

#endif /* STATE_H_ */
