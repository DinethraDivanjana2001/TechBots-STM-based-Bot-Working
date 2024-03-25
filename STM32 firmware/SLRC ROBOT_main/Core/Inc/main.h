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
#include "stm32f4xx_hal.h"

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
#define CLR_INT_4_Pin GPIO_PIN_2
#define CLR_INT_4_GPIO_Port GPIOE
#define CLR_LED_4_Pin GPIO_PIN_3
#define CLR_LED_4_GPIO_Port GPIOE
#define CLR_INT_3_Pin GPIO_PIN_4
#define CLR_INT_3_GPIO_Port GPIOE
#define CLR_LED_3_Pin GPIO_PIN_5
#define CLR_LED_3_GPIO_Port GPIOE
#define IMU_INT_Pin GPIO_PIN_6
#define IMU_INT_GPIO_Port GPIOE
#define FR_ARRAY_OUTA_Pin GPIO_PIN_0
#define FR_ARRAY_OUTA_GPIO_Port GPIOC
#define FR_ARRAY_OUTB_Pin GPIO_PIN_1
#define FR_ARRAY_OUTB_GPIO_Port GPIOC
#define BAT_MEASURE_Pin GPIO_PIN_2
#define BAT_MEASURE_GPIO_Port GPIOC
#define RR_ENCODER_A_Pin GPIO_PIN_0
#define RR_ENCODER_A_GPIO_Port GPIOA
#define RR_ENCODER_B_Pin GPIO_PIN_1
#define RR_ENCODER_B_GPIO_Port GPIOA
#define SHOOTER_PWM_Pin GPIO_PIN_2
#define SHOOTER_PWM_GPIO_Port GPIOA
#define PROX_IN_Pin GPIO_PIN_3
#define PROX_IN_GPIO_Port GPIOA
#define FR_ENCODER_A_Pin GPIO_PIN_6
#define FR_ENCODER_A_GPIO_Port GPIOA
#define FR_ENCODER_B_Pin GPIO_PIN_7
#define FR_ENCODER_B_GPIO_Port GPIOA
#define RR_ARRAY_ODD_Pin GPIO_PIN_4
#define RR_ARRAY_ODD_GPIO_Port GPIOC
#define RR_ARRAY_EVEN_Pin GPIO_PIN_5
#define RR_ARRAY_EVEN_GPIO_Port GPIOC
#define RR_ARR_OUT_Pin GPIO_PIN_0
#define RR_ARR_OUT_GPIO_Port GPIOB
#define RR_ARRAY_MUX_1_Pin GPIO_PIN_1
#define RR_ARRAY_MUX_1_GPIO_Port GPIOB
#define RR_ARRAY_MUX_2_Pin GPIO_PIN_2
#define RR_ARRAY_MUX_2_GPIO_Port GPIOB
#define RR_ARRAY_MUX_3_Pin GPIO_PIN_7
#define RR_ARRAY_MUX_3_GPIO_Port GPIOE
#define RR_ARRAY_MUX_4_Pin GPIO_PIN_8
#define RR_ARRAY_MUX_4_GPIO_Port GPIOE
#define FR_MOTOR_PWM_Pin GPIO_PIN_9
#define FR_MOTOR_PWM_GPIO_Port GPIOE
#define RR_MOTOR_PWM_Pin GPIO_PIN_11
#define RR_MOTOR_PWM_GPIO_Port GPIOE
#define TOF_LPIN_1_Pin GPIO_PIN_12
#define TOF_LPIN_1_GPIO_Port GPIOE
#define TOF_INT_1_Pin GPIO_PIN_13
#define TOF_INT_1_GPIO_Port GPIOE
#define TOF_LPIN_6_Pin GPIO_PIN_14
#define TOF_LPIN_6_GPIO_Port GPIOE
#define TOF_INT_6_Pin GPIO_PIN_15
#define TOF_INT_6_GPIO_Port GPIOE
#define FR_INA_Pin GPIO_PIN_12
#define FR_INA_GPIO_Port GPIOB
#define FR_INB_Pin GPIO_PIN_13
#define FR_INB_GPIO_Port GPIOB
#define RR_INA_Pin GPIO_PIN_14
#define RR_INA_GPIO_Port GPIOB
#define RR_INB_Pin GPIO_PIN_15
#define RR_INB_GPIO_Port GPIOB
#define TOF_LPIN_5_Pin GPIO_PIN_9
#define TOF_LPIN_5_GPIO_Port GPIOD
#define TOF_INT_5_Pin GPIO_PIN_10
#define TOF_INT_5_GPIO_Port GPIOD
#define TOF_LPIN_4_Pin GPIO_PIN_11
#define TOF_LPIN_4_GPIO_Port GPIOD
#define TOF_INT_4_Pin GPIO_PIN_12
#define TOF_INT_4_GPIO_Port GPIOD
#define SERVO_5_Pin GPIO_PIN_13
#define SERVO_5_GPIO_Port GPIOD
#define TOF_LPIN_3_Pin GPIO_PIN_14
#define TOF_LPIN_3_GPIO_Port GPIOD
#define TOF_INT_3_Pin GPIO_PIN_15
#define TOF_INT_3_GPIO_Port GPIOD
#define SERVO_4_Pin GPIO_PIN_6
#define SERVO_4_GPIO_Port GPIOC
#define SERVO_3_Pin GPIO_PIN_7
#define SERVO_3_GPIO_Port GPIOC
#define SERVO_2_Pin GPIO_PIN_8
#define SERVO_2_GPIO_Port GPIOC
#define SERVO_1_Pin GPIO_PIN_9
#define SERVO_1_GPIO_Port GPIOC
#define TOF_LPIN_2_Pin GPIO_PIN_8
#define TOF_LPIN_2_GPIO_Port GPIOA
#define TOF_INT_2_Pin GPIO_PIN_9
#define TOF_INT_2_GPIO_Port GPIOA
#define CLR_INT_1_Pin GPIO_PIN_11
#define CLR_INT_1_GPIO_Port GPIOA
#define CLR_LED_1_Pin GPIO_PIN_12
#define CLR_LED_1_GPIO_Port GPIOA
#define FR_ARRAY_ODD_Pin GPIO_PIN_15
#define FR_ARRAY_ODD_GPIO_Port GPIOA
#define BLUTOOTH_TX_Pin GPIO_PIN_10
#define BLUTOOTH_TX_GPIO_Port GPIOC
#define BLUTOOTH_RX_Pin GPIO_PIN_11
#define BLUTOOTH_RX_GPIO_Port GPIOC
#define FR_ARRAY_EVEN_Pin GPIO_PIN_12
#define FR_ARRAY_EVEN_GPIO_Port GPIOC
#define FR_ARRAY_SB2_Pin GPIO_PIN_0
#define FR_ARRAY_SB2_GPIO_Port GPIOD
#define FR_ARRAY_SB3_Pin GPIO_PIN_1
#define FR_ARRAY_SB3_GPIO_Port GPIOD
#define FR_ARRAY_SB0_Pin GPIO_PIN_2
#define FR_ARRAY_SB0_GPIO_Port GPIOD
#define FR_ARRAY_SB1_Pin GPIO_PIN_3
#define FR_ARRAY_SB1_GPIO_Port GPIOD
#define FR_ARRAY_SA0_Pin GPIO_PIN_4
#define FR_ARRAY_SA0_GPIO_Port GPIOD
#define FR_ARRAY_SA1_Pin GPIO_PIN_5
#define FR_ARRAY_SA1_GPIO_Port GPIOD
#define FR_ARRAY_SA2_Pin GPIO_PIN_6
#define FR_ARRAY_SA2_GPIO_Port GPIOD
#define FR_ARRAY_SA3_Pin GPIO_PIN_7
#define FR_ARRAY_SA3_GPIO_Port GPIOD
#define LCD_DS_Pin GPIO_PIN_6
#define LCD_DS_GPIO_Port GPIOB
#define LCD_RESET_Pin GPIO_PIN_7
#define LCD_RESET_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_8
#define LCD_CS_GPIO_Port GPIOB
#define CLR_INT_2_Pin GPIO_PIN_0
#define CLR_INT_2_GPIO_Port GPIOE
#define CLR_LED_2_Pin GPIO_PIN_1
#define CLR_LED_2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
