/*
 * servo.h
 *
 *  Created on: Jul 30, 2019
 *      Author: anton
 */

#ifndef SERVO_H_
#define SERVO_H_

#define SERVO_MAX	2550
#define SERVO_MIN	450
#define SERVO_STEPS	20
#define SERVO_STEP	((SERVO_MAX - SERVO_MIN) / SERVO_STEPS)

void servo_init(void);
void servo_start_pos(void);
void servo_next_pos(void);

void motor_init(void);
void motor_next_pos(void);

#endif /* SERVO_H_ */
