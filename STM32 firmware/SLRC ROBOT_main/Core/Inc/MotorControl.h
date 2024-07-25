/*
 * MotorControl.h
 *
 *  Created on: Mar 19, 2024
 *      Author: yasir
 */

#ifndef INC_MOTORCONTROL_H_
#define INC_MOTORCONTROL_H_

#define RR_MOTOR_PWM TIM_CHANNEL_1
#define FR_MOTOR_PWM TIM_CHANNEL_2

void speed(int Left, int Right);
void motor(int motor,int direction);

#endif /* INC_MOTORCONTROL_H_ */
