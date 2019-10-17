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

//--------- LSM6DS3 ---------------
#define LSM6DS3_PORT		GPIOA
#define LSM6DS3_CS_PIN		GPIO_PIN_12
#define LSM6DS3_SCK_PIN		GPIO_PIN_13
#define LSM6DS3_MISO_PIN	GPIO_PIN_14
#define LSM6DS3_MOSI_PIN	GPIO_PIN_15


// if error set value and go to end
#define PROCESS_ERROR(x) if (0 != (error = (x))) { goto end; }



extern TIM_TimeBaseInitTypeDef timer;
extern TIM_OCInitTypeDef timerPWM;


#endif /* STATE_H_ */
