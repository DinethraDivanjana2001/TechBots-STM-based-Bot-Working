/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "TPS55288.h"
#include "stm32f4xx_hal.h"
#include <stdio.h> // Include necessary library for printf
#include "PIDController.h"
#include "mpu6050.h"
#include "MotorControl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define TCS3472_ADDRESS 0x29 // I2C address of the TCS3472 device


//Direction definitions
#define left 0
#define right 1
#define Forward 1
#define Backward -1

#define RR_MOTOR_PWM TIM_CHANNEL_1
#define FR_MOTOR_PWM TIM_CHANNEL_2

#define IR_ARRAY_LENGTH 25

//Analog IR sensor values
int IR_array[IR_ARRAY_LENGTH] = {0};

//Digitalized sensor values
int digital_IR[IR_ARRAY_LENGTH] = {0};

//IR Array calibration data
int Ir_thresholds[IR_ARRAY_LENGTH] = {2000};


// PID parameters for Line Following
double Kp = 0.02;
double Ki = 11;
double Kd = 0.1;

int Drive_constant = 500;

// Initialize PID variables
double prevError = 0;
double integral = 0;
double derivative = 0;

uint8_t FR_IR_LED_ARR[25][5] = {
	{0,0,0,0,0},//Y0 IR1
	{0,0,0,0,1},//Y1 IR2
	{0,0,0,1,0},//Y2 IR3
	{0,0,0,1,1},//Y3 IR4
	{0,0,1,0,0},//Y4 IR5
	{0,0,1,0,1},//Y5 IR6
	{0,0,1,1,0},//Y6 IR7
	{0,0,1,1,1},//Y7 IR8
	{0,1,1,1,1},//Y15 IR9
	{0,1,1,1,0},//Y14 IR10
	{0,1,1,0,1},//Y13 IR11
	{0,1,1,0,0},//Y12 IR12
	{0,1,0,1,1},//Y11 IR13
	{0,1,0,1,0},//Y10 IR14
	{0,1,0,0,1},//Y9 IR15
	{0,1,0,0,0},//Y8 IR16
	{1,0,0,0,0},//Y0 IR17
	{1,0,0,0,1},//Y1 IR18
	{1,0,0,1,0},//Y2 IR19
	{1,0,0,1,1},//Y3 IR20
	{1,0,1,0,0},//Y4 IR21
	{1,0,1,0,1},//Y5 IR22
	{1,0,1,1,0},//Y6 IR23
	{1,0,1,1,1},//Y7 IR24
	{1,1,0,0,0}};//Y8 IR25



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

// Variables to store color data
uint16_t clear, red, green, blue;

//TPS55288 BuckBoost;
volatile uint32_t millis_counter = 0;
ADC_ChannelConfTypeDef adc1ConfigPrivate = {0};
ADC_ChannelConfTypeDef adc2ConfigPrivate = {0};
HAL_StatusTypeDef result;
uint8_t TX_Buffer [] = "A" ; // DATA to send
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void updateIR();
void calibrate();
void FR_Array_Mux_In_Select(int IR_LED_Num);
int Get_abs(int);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_UART4_Init();
  MX_SPI1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

//  #define ADDRESS_TO 0x40

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  //Initialize MPU6050 module
  mpu6050_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  calibrate();
  printf("Caliberation Done!");
  while (1)
  {

	  //Start motors at full speed
	  motor(left, Forward);motor(right, Forward);

	  //update the IR array with current data
	  updateIR();

	  int Left_drive, Right_drive; // Variables to store the returned drive values

	  // Call the PID_control function and store the returned values
	  struct DriveValues driveValues = PID_control(Kp, Kd, Ki, digital_IR);

	  // Extract the returned values from the structure
	  Left_drive = driveValues.LEFT;
	  Right_drive = driveValues.RIGHT;

	  if(Right_drive < 0){
		  motor(right, Backward);
	  } else {
		  motor(right, Forward);
	  }


	  if(Left_drive < 0){
		  motor(left, Backward);
	  } else {
		  motor(left, Forward);
	  }


	  speed(Get_abs(Left_drive), Get_abs(Right_drive));

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1023;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 16800-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 59;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 55999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CLR_LED_4_Pin|CLR_INT_3_Pin|CLR_LED_3_Pin|RR_ARRAY_MUX_3_Pin
                          |RR_ARRAY_MUX_4_Pin|TOF_LPIN_1_Pin|TOF_INT_6_Pin|CLR_LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RR_ARRAY_ODD_Pin|RR_ARRAY_EVEN_Pin|FR_ARRAY_EVEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RR_ARRAY_MUX_1_Pin|RR_ARRAY_MUX_2_Pin|FR_INA_Pin|FR_INB_Pin
                          |RR_INA_Pin|RR_INB_Pin|LCD_DS_Pin|LCD_RESET_Pin
                          |LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, TOF_LPIN_5_Pin|TOF_LPIN_4_Pin|TOF_LPIN_3_Pin|FR_ARRAY_SB2_Pin
                          |FR_ARRAY_SB3_Pin|FR_ARRAY_SB0_Pin|FR_ARRAY_SB1_Pin|FR_ARRAY_SA0_Pin
                          |FR_ARRAY_SA1_Pin|FR_ARRAY_SA2_Pin|FR_ARRAY_SA3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TOF_LPIN_2_Pin|CLR_LED_1_Pin|FR_ARRAY_ODD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CLR_INT_4_Pin IMU_INT_Pin TOF_INT_1_Pin TOF_LPIN_6_Pin
                           CLR_INT_2_Pin */
  GPIO_InitStruct.Pin = CLR_INT_4_Pin|IMU_INT_Pin|TOF_INT_1_Pin|TOF_LPIN_6_Pin
                          |CLR_INT_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CLR_LED_4_Pin CLR_INT_3_Pin CLR_LED_3_Pin RR_ARRAY_MUX_3_Pin
                           RR_ARRAY_MUX_4_Pin TOF_LPIN_1_Pin TOF_INT_6_Pin CLR_LED_2_Pin */
  GPIO_InitStruct.Pin = CLR_LED_4_Pin|CLR_INT_3_Pin|CLR_LED_3_Pin|RR_ARRAY_MUX_3_Pin
                          |RR_ARRAY_MUX_4_Pin|TOF_LPIN_1_Pin|TOF_INT_6_Pin|CLR_LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PROX_IN_Pin TOF_INT_2_Pin CLR_INT_1_Pin */
  GPIO_InitStruct.Pin = PROX_IN_Pin|TOF_INT_2_Pin|CLR_INT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RR_ARRAY_ODD_Pin RR_ARRAY_EVEN_Pin FR_ARRAY_EVEN_Pin */
  GPIO_InitStruct.Pin = RR_ARRAY_ODD_Pin|RR_ARRAY_EVEN_Pin|FR_ARRAY_EVEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RR_ARRAY_MUX_1_Pin RR_ARRAY_MUX_2_Pin FR_INA_Pin FR_INB_Pin
                           RR_INA_Pin RR_INB_Pin LCD_DS_Pin LCD_RESET_Pin
                           LCD_CS_Pin */
  GPIO_InitStruct.Pin = RR_ARRAY_MUX_1_Pin|RR_ARRAY_MUX_2_Pin|FR_INA_Pin|FR_INB_Pin
                          |RR_INA_Pin|RR_INB_Pin|LCD_DS_Pin|LCD_RESET_Pin
                          |LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TOF_LPIN_5_Pin TOF_LPIN_4_Pin TOF_LPIN_3_Pin FR_ARRAY_SB2_Pin
                           FR_ARRAY_SB3_Pin FR_ARRAY_SB0_Pin FR_ARRAY_SB1_Pin FR_ARRAY_SA0_Pin
                           FR_ARRAY_SA1_Pin FR_ARRAY_SA2_Pin FR_ARRAY_SA3_Pin */
  GPIO_InitStruct.Pin = TOF_LPIN_5_Pin|TOF_LPIN_4_Pin|TOF_LPIN_3_Pin|FR_ARRAY_SB2_Pin
                          |FR_ARRAY_SB3_Pin|FR_ARRAY_SB0_Pin|FR_ARRAY_SB1_Pin|FR_ARRAY_SA0_Pin
                          |FR_ARRAY_SA1_Pin|FR_ARRAY_SA2_Pin|FR_ARRAY_SA3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : TOF_INT_5_Pin TOF_INT_4_Pin TOF_INT_3_Pin */
  GPIO_InitStruct.Pin = TOF_INT_5_Pin|TOF_INT_4_Pin|TOF_INT_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : TOF_LPIN_2_Pin CLR_LED_1_Pin FR_ARRAY_ODD_Pin */
  GPIO_InitStruct.Pin = TOF_LPIN_2_Pin|CLR_LED_1_Pin|FR_ARRAY_ODD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
    millis_counter++;
  }
}

uint32_t millis() {
  return millis_counter;
}


void updateIR(){
	  // loop through amount of leds. ARRAY STARTS FROM 1
	   HAL_GPIO_WritePin(FR_ARRAY_EVEN_GPIO_Port, FR_ARRAY_EVEN_Pin, GPIO_PIN_SET);
	 	HAL_GPIO_WritePin(FR_ARRAY_ODD_GPIO_Port, FR_ARRAY_ODD_Pin, GPIO_PIN_SET);
	  for (int i = 0; i < 25; i=i+2) {
			FR_Array_Mux_In_Select(i+1);
		  if(i <= 15){
	  		  	adc1ConfigPrivate.Channel = ADC_CHANNEL_10;
	  		  	HAL_ADC_ConfigChannel(&hadc1, &adc1ConfigPrivate);
	  		    HAL_ADC_Start(&hadc1);
	  		    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//	  		    raw = HAL_ADC_GetValue(&hadc1);
	  		    IR_array[i] = HAL_ADC_GetValue(&hadc1);
//		  			delay_us(2);

		  }else{
			adc2ConfigPrivate.Channel = ADC_CHANNEL_11;
	  		  	HAL_ADC_ConfigChannel(&hadc2, &adc2ConfigPrivate);
	  		    HAL_ADC_Start(&hadc2);
	  		    HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
//	  		    raw = HAL_ADC_GetValue(&hadc2);
	  		    IR_array[i] = HAL_ADC_GetValue(&hadc2);
//		  			delay_us(2);
		  }
		  digital_IR[i] = IR_array[i] < Ir_thresholds[i];
		  HAL_Delay(1);
	  }
	  for (int i = 1; i < 25; i=i+2) {
			FR_Array_Mux_In_Select(i+1);
		  if(i <= 15){
	  		  	adc1ConfigPrivate.Channel = ADC_CHANNEL_10;
	  		  	HAL_ADC_ConfigChannel(&hadc1, &adc1ConfigPrivate);
	  		    HAL_ADC_Start(&hadc1);
	  		    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//	  		    raw = HAL_ADC_GetValue(&hadc1);
	  		    IR_array[i] = HAL_ADC_GetValue(&hadc1);
//		  			delay_us(2);
		  }else{
			adc2ConfigPrivate.Channel = ADC_CHANNEL_11;
	  		  	HAL_ADC_ConfigChannel(&hadc2, &adc2ConfigPrivate);
	  		    HAL_ADC_Start(&hadc2);
	  		    HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
//	  		    raw = HAL_ADC_GetValue(&hadc2);
	  		    IR_array[i] = HAL_ADC_GetValue(&hadc2);
//		  			delay_us(2);

		  }
		  digital_IR[i] = IR_array[i] < Ir_thresholds[i];
		  HAL_Delay(1);
	  }
}


void FR_Array_Mux_In_Select(int IR_LED_Num){
	if(IR_LED_Num <= 16){
	    HAL_GPIO_WritePin(FR_ARRAY_SA0_GPIO_Port, FR_ARRAY_SA0_Pin, FR_IR_LED_ARR[IR_LED_Num-1][4]);
	    HAL_GPIO_WritePin(FR_ARRAY_SA1_GPIO_Port, FR_ARRAY_SA1_Pin, FR_IR_LED_ARR[IR_LED_Num-1][3]);
	    HAL_GPIO_WritePin(FR_ARRAY_SA2_GPIO_Port, FR_ARRAY_SA2_Pin, FR_IR_LED_ARR[IR_LED_Num-1][2]);
	    HAL_GPIO_WritePin(FR_ARRAY_SA3_GPIO_Port, FR_ARRAY_SA3_Pin, FR_IR_LED_ARR[IR_LED_Num-1][1]);
	}	if(IR_LED_Num > 16){
	    HAL_GPIO_WritePin(FR_ARRAY_SB0_GPIO_Port, FR_ARRAY_SB0_Pin, FR_IR_LED_ARR[IR_LED_Num-1][4]);
	    HAL_GPIO_WritePin(FR_ARRAY_SB1_GPIO_Port, FR_ARRAY_SB1_Pin, FR_IR_LED_ARR[IR_LED_Num-1][3]);
	    HAL_GPIO_WritePin(FR_ARRAY_SB2_GPIO_Port, FR_ARRAY_SB2_Pin, FR_IR_LED_ARR[IR_LED_Num-1][2]);
	    HAL_GPIO_WritePin(FR_ARRAY_SB3_GPIO_Port, FR_ARRAY_SB3_Pin, FR_IR_LED_ARR[IR_LED_Num-1][1]);
	}

}


void calibrate()
{

//  motor(right, Backward);
//  motor(left, Forward);
//
//  // Stop motors when calibrating
//  speed(255, 255);

  // make sensor_max_values array of length IR_ARRAY_LENGTH equal to sensor calibration array
  int sensor_max_values[IR_ARRAY_LENGTH] = {0};

  // sensor min values array
  int sensor_min_values[IR_ARRAY_LENGTH] = {0};

  int now_time_for_calibration = millis();

  while (millis() - now_time_for_calibration < 5)
  {

    updateIR();
    uint8_t allZeros = 1;
    for (int i = 0; i < IR_ARRAY_LENGTH; i++)
    {
      if (sensor_max_values[i] != 0)
      {
        allZeros = 0;
        break; // No need to continue checking if we find a non-zero value
      }
    }

    if (allZeros)
    {
      for (int i = 0; i < IR_ARRAY_LENGTH; i++)
      {
        sensor_max_values[i] = IR_array[i];
        sensor_min_values[i] = IR_array[i];
      }
    }

    for (int i = 0; i < IR_ARRAY_LENGTH; i++)
    {
      if (IR_array[i] > sensor_max_values[i])
      {
        sensor_max_values[i] = IR_array[i];
      }
      if (IR_array[i] < sensor_min_values[i])
      {
        sensor_min_values[i] = IR_array[i];
      }
    }
  }

  speed(0, 0);

  for (int i = 0; i < IR_ARRAY_LENGTH; i++)
  {
    Ir_thresholds[i] = (sensor_max_values[i] + sensor_min_values[i]) / 2;
  }

}

//// EXTI line 1 interrupt handler
//void EXTI0_IRQHandler(void) {
//    // Your code here
//    // This code will be executed when the interrupt occurs
//    // Handle the interrupt (e.g., clear the interrupt flag)
//    HAL_GPIO_EXTI_IRQHandler(CLR_INT_2_Pin); // Assuming GPIO_PIN_1 is the pin associated with EXTI line 1
//}

void servo_sweep(void){
	int x;
	  for(x=1400; x<7000; x=x+1)
	  {
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1, x);
		HAL_Delay(3);
	  }

	  for(x=1400; x<7000; x=x+1)
	  {
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2, x);
		HAL_Delay(3);
	  }

	  for(x=1400; x<7000; x=x+1)
	  {
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3, x);
		HAL_Delay(3);
	  }

	  for(x=1400; x<7000; x=x+1)
	  {
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4, x);
		HAL_Delay(3);
	  }
}

int Get_abs(int num)
{
	if (num>=0)
	{
		return num;
	}
	else
	{
		return -num;
	}
}

// Function to initialize TCS3472
void TCS3472_Init(I2C_HandleTypeDef *hi2c) {
    // Example initialization steps
    // Write to the command register to set up the device
    // You may need to set integration time, wait time, etc.
    // Refer to the datasheet for initialization details
}

// Function to read color data from TCS3472
void TCS3472_Read_Color_Data(I2C_HandleTypeDef *hi2c, uint16_t *clear, uint16_t *red, uint16_t *green, uint16_t *blue) {
    // Write to the command register to select the desired register for reading color data
    // For example, to read clear data, you need to write the command register with 0x14 address
    uint8_t command = 0x14;
    HAL_I2C_Master_Transmit(hi2c, TCS3472_ADDRESS << 1, &command, 1, HAL_MAX_DELAY);

    // Read color data
    uint8_t data[8]; // Assuming each color data is 2 bytes
    HAL_I2C_Master_Receive(hi2c, TCS3472_ADDRESS << 1, data, 8, HAL_MAX_DELAY);

    // Extract color data from received data
    *clear = (data[1] << 8) | data[0]; // Assuming clear data is in register 0x14
    *red = (data[3] << 8) | data[2];   // Assuming red data is in register 0x16
    *green = (data[5] << 8) | data[4]; // Assuming green data is in register 0x18
    *blue = (data[7] << 8) | data[6];  // Assuming blue data is in register 0x1A
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
