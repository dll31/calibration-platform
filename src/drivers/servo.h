/*
 * servo.h
 *
 *  Created on: Jul 30, 2019
 *      Author: anton
 */

#ifndef SERVO_H_
#define SERVO_H_

void servo_init(void);
void servo_timer_init(void);
void change_pulse(uint32_t pulse);
void servo_rotate(double angle);


#endif /* SERVO_H_ */
