/*
 * MotorControl.c
 *
 *  Created on: Mar 19, 2024
 *      Author: yasir
 */

#include "main.h"
#include "stm32f4xx_hal.h"
#include "MotorControl.h"

extern TIM_HandleTypeDef htim1;

void speed(int Left, int Right){
	  __HAL_TIM_SET_COMPARE(&htim1,FR_MOTOR_PWM,Right);
	  __HAL_TIM_SET_COMPARE(&htim1,RR_MOTOR_PWM,Left);
}

void motor(int motor,int direction){
	if (motor == 0){
		if (direction == 0){
			HAL_GPIO_WritePin(RR_INB_GPIO_Port, RR_INB_Pin, 0);
			HAL_GPIO_WritePin(RR_INA_GPIO_Port, RR_INA_Pin, 1);
		}else{
			HAL_GPIO_WritePin(RR_INB_GPIO_Port, RR_INB_Pin, 1);
			HAL_GPIO_WritePin(RR_INA_GPIO_Port, RR_INA_Pin, 0);
		}
	}else{
		if (direction == 1){
			HAL_GPIO_WritePin(FR_INB_GPIO_Port, FR_INB_Pin, 0);
			HAL_GPIO_WritePin(FR_INA_GPIO_Port, FR_INA_Pin, 1);
		}else{
			HAL_GPIO_WritePin(FR_INB_GPIO_Port, FR_INB_Pin, 1);
			HAL_GPIO_WritePin(FR_INA_GPIO_Port, FR_INA_Pin, 0);
		}
	}
}
