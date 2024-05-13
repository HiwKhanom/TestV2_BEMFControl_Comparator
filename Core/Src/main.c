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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
EXTI_HandleTypeDef hexti0;
EXTI_HandleTypeDef hexti1;
EXTI_HandleTypeDef hexti2;

EXTI_ConfigTypeDef sEXTIConfigTypeDef = {0};

uint16_t PWM_max = 1500;
uint16_t PWM_min = 10;
uint16_t PWM_start = 900;

uint16_t duty = 900;

uint8_t BLDCstep = 0;

uint8_t INT = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void BLDC_startBEMF();
void BLDC_state();

//BLDC_step
void AH_BL();
void AH_CL();
void BH_CL();
void BH_AL();
void CH_AL();
void CH_BL();

//BLDC state
void A_Rising();
void A_Falling();
void B_Rising();
void B_Falling();
void C_Rising();
void C_Falling();

//void setAINTRising();
//void setAINTFalling();
//void setBINTRising();
//void setBINTFalling();
//void setCINTRising();
//void setCINTFalling();

//set maxDuty
uint16_t PWM_setLimit(uint16_t duty);

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
  MX_LPUART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_DisableIRQ((EXTI0_IRQn | EXTI1_IRQn | EXTI2_IRQn));

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  //init delay system
  HAL_Delay(2000);

  duty = PWM_start;
  BLDC_startBEMF();

  HAL_NVIC_EnableIRQ((EXTI0_IRQn | EXTI1_IRQn | EXTI2_IRQn));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LA_Pin|LB_Pin|LC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : B1_Pin Comp1_Pin Comp2_Pin Comp3_Pin */
  GPIO_InitStruct.Pin = B1_Pin|Comp1_Pin|Comp2_Pin|Comp3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LA_Pin LB_Pin LC_Pin */
  GPIO_InitStruct.Pin = LA_Pin|LB_Pin|LC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void BLDC_state(){
	switch(BLDCstep){
	case 0:
		AH_BL();
		C_Rising();
		break;
	case 1:
		AH_CL();
		B_Falling();
		break;
	case 2:
		BH_CL();
		A_Rising();
		break;
	case 3:
		BH_AL();
		C_Falling();
		break;
	case 4:
		CH_AL();
		B_Rising();
		break;
	case 5:
		CH_BL();
		A_Falling();
		break;
	}
}

void AH_BL(){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

	//AL BH(PWM) CH(PWM) CL OFF
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, RESET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, RESET);

	//AH(PWM) BL ON
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_setLimit(duty));
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, SET);
}
void AH_CL(){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

	//AL BH(PWM) BL CH(PWM) OFF
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, RESET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, RESET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

	//AH(PWM) CL ON
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_setLimit(duty));
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, SET);
}
void BH_CL(){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

	//AH(PWM) AL BL CH(PWM) OFF
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, RESET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

	//BH(PWM) CL ON
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_setLimit(duty));
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, SET);
}
void BH_AL(){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

	//AH(PWM) BL CH(PWM) CL OFF
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, RESET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, RESET);

	//BH(PWM) AL ON
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_setLimit(duty));
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, SET);
}
void CH_AL(){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

	//AH(PWM) BH(PWM) BL CL OFF
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, RESET);

	//CH(PWM) AL ON
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_setLimit(duty));
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, SET);
}
void CH_BL(){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

	//AH(PWM) AL BH(PWM) CL OFF
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, RESET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, RESET);

	//CH(PWM) BL ON
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_setLimit(duty));
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, SET);
}

void A_Rising(){
	HAL_NVIC_DisableIRQ(EXTI0_IRQn | EXTI1_IRQn | EXTI2_IRQn);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	sEXTIConfigTypeDef.Trigger = EXTI_TRIGGER_RISING;
	HAL_EXTI_SetConfigLine(&hexti0, &sEXTIConfigTypeDef);

	INT = 1;
}

void A_Falling(){
	HAL_NVIC_DisableIRQ(EXTI0_IRQn | EXTI1_IRQn | EXTI2_IRQn);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	sEXTIConfigTypeDef.Trigger = EXTI_TRIGGER_FALLING;
	HAL_EXTI_SetConfigLine(&hexti0, &sEXTIConfigTypeDef);

	INT = 1;
}

void B_Rising(){
	HAL_NVIC_DisableIRQ(EXTI0_IRQn | EXTI1_IRQn | EXTI2_IRQn);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	sEXTIConfigTypeDef.Trigger = EXTI_TRIGGER_RISING;
	HAL_EXTI_SetConfigLine(&hexti0, &sEXTIConfigTypeDef);

	INT = 2;
}

void B_Falling(){
	HAL_NVIC_DisableIRQ(EXTI0_IRQn | EXTI1_IRQn | EXTI2_IRQn);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	sEXTIConfigTypeDef.Trigger = EXTI_TRIGGER_FALLING;
	HAL_EXTI_SetConfigLine(&hexti0, &sEXTIConfigTypeDef);

	INT = 2;
}

void C_Rising(){
	HAL_NVIC_DisableIRQ(EXTI0_IRQn | EXTI1_IRQn | EXTI2_IRQn);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	sEXTIConfigTypeDef.Trigger = EXTI_TRIGGER_RISING;
	HAL_EXTI_SetConfigLine(&hexti0, &sEXTIConfigTypeDef);

	INT = 3;
}

void C_Falling(){
	HAL_NVIC_DisableIRQ(EXTI0_IRQn | EXTI1_IRQn | EXTI2_IRQn);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	sEXTIConfigTypeDef.Trigger = EXTI_TRIGGER_FALLING;
	HAL_EXTI_SetConfigLine(&hexti0, &sEXTIConfigTypeDef);

	INT = 3;
}

//
//void setAINTRising(){
//	HAL_NVIC_DisableIRQ(EXTI0_IRQn | EXTI1_IRQn | EXTI2_IRQn);
//	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
//
//	sEXTIConfigTypeDef.Trigger = EXTI_TRIGGER_RISING;
//	HAL_EXTI_SetConfigLine(&hexti0, &sEXTIConfigTypeDef);
//}
//
//void setAINTFalling(){
//	HAL_NVIC_DisableIRQ(EXTI0_IRQn | EXTI1_IRQn | EXTI2_IRQn);
//	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
//
//	sEXTIConfigTypeDef.Trigger = EXTI_TRIGGER_FALLING;
//	HAL_EXTI_SetConfigLine(&hexti0, &sEXTIConfigTypeDef);
//}
//
//void setBINTRising(){
//	HAL_NVIC_DisableIRQ(EXTI0_IRQn | EXTI1_IRQn | EXTI2_IRQn);
//	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
//
//	sEXTIConfigTypeDef.Trigger = EXTI_TRIGGER_RISING;
//	HAL_EXTI_SetConfigLine(&hexti0, &sEXTIConfigTypeDef);
//}
//
//void setBINTFalling(){
//	HAL_NVIC_DisableIRQ(EXTI0_IRQn | EXTI1_IRQn | EXTI2_IRQn);
//	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
//
//	sEXTIConfigTypeDef.Trigger = EXTI_TRIGGER_FALLING;
//	HAL_EXTI_SetConfigLine(&hexti0, &sEXTIConfigTypeDef);
//}
//
//void setCINTRising(){
//	HAL_NVIC_DisableIRQ(EXTI0_IRQn | EXTI1_IRQn | EXTI2_IRQn);
//	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
//
//	sEXTIConfigTypeDef.Trigger = EXTI_TRIGGER_RISING;
//	HAL_EXTI_SetConfigLine(&hexti0, &sEXTIConfigTypeDef);
//}
//
//void setCINTFalling(){
//	HAL_NVIC_DisableIRQ(EXTI0_IRQn | EXTI1_IRQn | EXTI2_IRQn);
//	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
//
//	sEXTIConfigTypeDef.Trigger = EXTI_TRIGGER_FALLING;
//	HAL_EXTI_SetConfigLine(&hexti0, &sEXTIConfigTypeDef);
//}

uint16_t PWM_setLimit(uint16_t duty){
	if(duty < PWM_min) return duty = PWM_min;
	else if(duty > PWM_max) return duty = PWM_max;
	else return duty;
}

void BLDC_startBEMF(){
	static uint32_t i = 20000;
	while(i > 100){
		HAL_Delay((i/10000));
		BLDC_state();
		BLDCstep++;
		BLDCstep %= 6;
		i = i - 200;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(INT == 1){
		for (uint8_t i = 0; i < 150 ; i++){
			if(BLDCstep & 1){
				if(!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0))){
					i -= 1;
				}
			}
			else{
				if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)){
					i -= 1;
				}
			}
		}
	}

	else if(INT == 2){
		for (uint8_t i = 0; i < 150 ; i++){
			if(BLDCstep & 1){
				if(!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1))){
					i -= 1;
				}
			}
			else{
				if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)){
					i -= 1;
				}
			}
		}
	}

	else if(INT == 3){
		for (uint8_t i = 0; i < 150 ; i++){
			if(BLDCstep & 1){
				if(!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2))){
					i -= 1;
				}
			}
			else{
				if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)){
					i -= 1;
				}
			}
		}
	}

	BLDC_state();
	BLDCstep++;
	BLDCstep %= 6;
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
