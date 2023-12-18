/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "ssd1306.h"
#include "stdio.h"
#include "string.h"
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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  int button_1 = 0;
  int button_2 = 0;
  int button_3 = 0;
  int button_4 = 0;
  int vacancy;
  char vacancy_sit[20];
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	button_1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	button_2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
	button_3 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
	button_4 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);


  	if(button_1 == 1 && button_2 == 1 && button_3 == 1 && button_4 == 1){
  		vacancy = 0;
  	}

  	else if(button_1 == 0 && button_2 == 1 && button_3 == 1 && button_4 == 1){
  		vacancy = 1;
  	}
  	else if(button_1 == 1 && button_2 == 0 && button_3 == 1 && button_4 == 1){
  		vacancy = 1;
  	}
  	else if(button_1 == 1 && button_2 == 1 && button_3 == 0 && button_4 == 1){
  		vacancy = 1;
  	}
  	else if(button_1 == 1 && button_2 == 1 && button_3 == 1 && button_4 == 0){
  		vacancy = 1;
  	}

  	else if(button_1 == 0 && button_2 == 0 && button_3 == 1 && button_4 == 1){
  		vacancy = 2;
  	}
  	else if(button_1 == 0 && button_2 == 1 && button_3 == 0 && button_4 == 1){
  		vacancy = 2;
  	}
  	else if(button_1 == 0 && button_2 == 1 && button_3 == 1 && button_4 == 0){
  		vacancy = 2;
  	}
  	else if(button_1 == 1 && button_2 == 0 && button_3 == 0 && button_4 == 1){
  		vacancy = 2;
  	}
  	else if(button_1 == 1 && button_2 == 0 && button_3 == 1 && button_4 == 0){
  		vacancy = 2;
  	}
  	else if(button_1 == 1 && button_2 == 1 && button_3 == 0 && button_4 == 0){
  		vacancy = 2;
  	}

  	else if(button_1 == 0 && button_2 == 0 && button_3 == 0 && button_4 == 1){
  		vacancy = 3;
  	}
  	else if(button_1 == 0 && button_2 == 0 && button_3 == 1 && button_4 == 0){
  		vacancy = 3;
  	}
  	else if(button_1 == 0 && button_2 == 1 && button_3 == 0 && button_4 == 0){
  		vacancy = 3;
  	}
  	else if(button_1 == 1 && button_2 == 0 && button_3 == 0 && button_4 == 0){
  		vacancy = 3;
  	}

  	else if(button_1 == 0 && button_2 == 0 && button_3 == 0 && button_4 == 0){
  		vacancy = 4;
  	}

  	ssd1306_SetCursor(2, 0);
	sprintf(vacancy_sit, "vacancy: %d", vacancy);
	HAL_Delay(10);
	ssd1306_WriteString(vacancy_sit, Font_11x18, White);
	ssd1306_WriteString("        ", Font_11x18, White);
	ssd1306_UpdateScreen();


  	if(button_1 == 1 && button_2 == 1 && button_3 == 1 && button_4 == 1){
  		HAL_UART_Transmit(&huart1, (uint8_t *)"1 1 1 1\n", sizeof("1 1 1 1\n")-1, 10);
  		HAL_UART_Transmit(&huart2, (uint8_t *)"1\n", sizeof("1\n")-1, 10);
  		htim3.Instance->CCR1 = 1500;
  		HAL_Delay(1000);
  	}

  	else if(button_1 == 0 && button_2 == 1 && button_3 == 1 && button_4 == 1){
  		HAL_UART_Transmit(&huart1, (uint8_t *)"0 1 1 1\n", sizeof("0 1 1 1\n")-1, 10);
  		HAL_UART_Transmit(&huart2, (uint8_t *)"0\n", sizeof("0\n")-1, 10);
  		htim3.Instance->CCR1 = 2500;
  		HAL_Delay(1000);
  	}
  	else if(button_1 == 1 && button_2 == 0 && button_3 == 1 && button_4 == 1){

  		HAL_UART_Transmit(&huart1, (uint8_t *)"1 0 1 1\n", sizeof("1 0 1 1\n")-1, 10);
  		HAL_UART_Transmit(&huart2, (uint8_t *)"0\n", sizeof("0\n")-1, 10);
  		htim3.Instance->CCR1 = 2500;
  		HAL_Delay(1000);
  	}
  	else if(button_1 == 1 && button_2 == 1 && button_3 == 0 && button_4 == 1){
  		htim3.Instance->CCR1 = 2500;
  		HAL_UART_Transmit(&huart1, (uint8_t *)"1 1 0 1\n", sizeof("1 1 0 1\n")-1, 10);
  		HAL_UART_Transmit(&huart2, (uint8_t *)"0\n", sizeof("0\n")-1, 10);
  		HAL_Delay(1000);
  	}
  	else if(button_1 == 1 && button_2 == 1 && button_3 == 1 && button_4 == 0){
  		htim3.Instance->CCR1 = 2500;
  		HAL_UART_Transmit(&huart1, (uint8_t *)"1 1 1 0\n", sizeof("1 1 1 0\n")-1, 10);
  		HAL_UART_Transmit(&huart2, (uint8_t *)"0\n", sizeof("0\n")-1, 10);
  		HAL_Delay(1000);
  	}

  	else if(button_1 == 0 && button_2 == 0 && button_3 == 1 && button_4 == 1){
  		htim3.Instance->CCR1 = 2500;
  		HAL_UART_Transmit(&huart1, (uint8_t *)"0 0 1 1\n", sizeof("0 0 1 1\n")-1, 10);
  		HAL_UART_Transmit(&huart2, (uint8_t *)"0\n", sizeof("0\n")-1, 10);
  		HAL_Delay(1000);
  	}
  	else if(button_1 == 0 && button_2 == 1 && button_3 == 0 && button_4 == 1){
  		htim3.Instance->CCR1 = 2500;
  		HAL_UART_Transmit(&huart1, (uint8_t *)"0 1 0 1\n", sizeof("0 1 0 1\n")-1, 10);
  		HAL_UART_Transmit(&huart2, (uint8_t *)"0\n", sizeof("0\n")-1, 10);
  		HAL_Delay(1000);
  	}
  	else if(button_1 == 0 && button_2 == 1 && button_3 == 1 && button_4 == 0){
  		htim3.Instance->CCR1 = 2500;
  		HAL_UART_Transmit(&huart1, (uint8_t *)"0 1 1 0\n", sizeof("0 1 1 0\n")-1, 10);
  		HAL_UART_Transmit(&huart2, (uint8_t *)"0\n", sizeof("0\n")-1, 10);
  		HAL_Delay(1000);
  	}
  	else if(button_1 == 1 && button_2 == 0 && button_3 == 0 && button_4 == 1){
  		htim3.Instance->CCR1 = 2500;
  		HAL_UART_Transmit(&huart1, (uint8_t *)"1 0 0 1\n", sizeof("1 0 0 1\n")-1, 10);
  		HAL_UART_Transmit(&huart2, (uint8_t *)"0\n", sizeof("0\n")-1, 10);
  		HAL_Delay(1000);
  	}
  	else if(button_1 == 1 && button_2 == 0 && button_3 == 1 && button_4 == 0){
  		htim3.Instance->CCR1 = 2500;
  		HAL_UART_Transmit(&huart1, (uint8_t *)"1 0 1 0\n", sizeof("1 0 1 0\n")-1, 10);
  		HAL_UART_Transmit(&huart2, (uint8_t *)"0\n", sizeof("0\n")-1, 10);
  		HAL_Delay(1000);
  	}
  	else if(button_1 == 1 && button_2 == 1 && button_3 == 0 && button_4 == 0){
  		htim3.Instance->CCR1 = 2500;
  		HAL_UART_Transmit(&huart1, (uint8_t *)"1 1 0 0\n", sizeof("1 1 0 0\n")-1, 10);
  		HAL_UART_Transmit(&huart2, (uint8_t *)"0\n", sizeof("0\n")-1, 10);
  		HAL_Delay(1000);
  	}

  	else if(button_1 == 0 && button_2 == 0 && button_3 == 0 && button_4 == 1){
  		htim3.Instance->CCR1 = 2500;
  		HAL_UART_Transmit(&huart1, (uint8_t *)"0 0 0 1\n", sizeof("0 0 0 1\n")-1, 10);
  		HAL_UART_Transmit(&huart2, (uint8_t *)"0\n", sizeof("0\n")-1, 10);
  		HAL_Delay(1000);
  	}
  	else if(button_1 == 0 && button_2 == 0 && button_3 == 1 && button_4 == 0){
  		htim3.Instance->CCR1 = 2500;
  		HAL_UART_Transmit(&huart1, (uint8_t *)"0 0 1 0\n", sizeof("0 0 1 0\n")-1, 10);
  		HAL_UART_Transmit(&huart2, (uint8_t *)"0\n", sizeof("0\n")-1, 10);
  		HAL_Delay(1000);
  	}
  	else if(button_1 == 0 && button_2 == 1 && button_3 == 0 && button_4 == 0){
  		htim3.Instance->CCR1 = 2500;
  		HAL_UART_Transmit(&huart1, (uint8_t *)"0 1 0 0\n", sizeof("0 1 0 0\n")-1, 10);
  		HAL_UART_Transmit(&huart2, (uint8_t *)"0\n", sizeof("0\n")-1, 10);
  		HAL_Delay(1000);
  	}
  	else if(button_1 == 1 && button_2 == 0 && button_3 == 0 && button_4 == 0){
  		htim3.Instance->CCR1 = 2500;
  		HAL_UART_Transmit(&huart1, (uint8_t *)"1 0 0 0\n", sizeof("1 0 0 0\n")-1, 10);
  		HAL_UART_Transmit(&huart2, (uint8_t *)"0\n", sizeof("0\n")-1, 10);
  		HAL_Delay(1000);
  	}

  	else if(button_1 == 0 && button_2 == 0 && button_3 == 0 && button_4 == 0){
  		HAL_UART_Transmit(&huart1, (uint8_t *)"0 0 0 0\n", sizeof("0 0 0 0\n")-1, 10);
  		HAL_UART_Transmit(&huart2, (uint8_t *)"0\n", sizeof("0\n")-1, 10);
  		htim3.Instance->CCR1 = 2500;
  		HAL_Delay(1000);
  	}
  	HAL_Delay(10);
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
