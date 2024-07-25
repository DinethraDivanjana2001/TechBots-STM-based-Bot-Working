/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Calibrate_btn_Pin GPIO_PIN_13
#define Calibrate_btn_GPIO_Port GPIOC
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define MotorR_B_Pin GPIO_PIN_2
#define MotorR_B_GPIO_Port GPIOC
#define MotorR_A_Pin GPIO_PIN_3
#define MotorR_A_GPIO_Port GPIOC
#define Sensor_0_Pin GPIO_PIN_0
#define Sensor_0_GPIO_Port GPIOA
#define Sensor_8_Pin GPIO_PIN_1
#define Sensor_8_GPIO_Port GPIOA
#define Sensor_1_Pin GPIO_PIN_2
#define Sensor_1_GPIO_Port GPIOA
#define Sensor_2_Pin GPIO_PIN_3
#define Sensor_2_GPIO_Port GPIOA
#define Sensor_3_Pin GPIO_PIN_4
#define Sensor_3_GPIO_Port GPIOA
#define Sensor_4_Pin GPIO_PIN_5
#define Sensor_4_GPIO_Port GPIOA
#define Sensor_5_Pin GPIO_PIN_6
#define Sensor_5_GPIO_Port GPIOA
#define Sensor_6_Pin GPIO_PIN_7
#define Sensor_6_GPIO_Port GPIOA
#define MotorL_B_Pin GPIO_PIN_0
#define MotorL_B_GPIO_Port GPIOB
#define MotorL_A_Pin GPIO_PIN_1
#define MotorL_A_GPIO_Port GPIOB
#define Sensor_7_Pin GPIO_PIN_2
#define Sensor_7_GPIO_Port GPIOB
#define MotorR_PWM_Pin GPIO_PIN_8
#define MotorR_PWM_GPIO_Port GPIOA
#define MotorL_PWM_Pin GPIO_PIN_9
#define MotorL_PWM_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
