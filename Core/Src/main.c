/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_lib.h"
#include "linear_motor_lib.h"

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM15_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t T31pulseWidth = 0;
uint32_t T31Rising = 0;
uint32_t T31Tick = 0;
uint32_t T31vectSum = 0;
uint32_t T31vect[5] = { 0 };

uint32_t T32pulseWidth = 0;
uint32_t T32Rising = 0;
uint32_t T32Tick = 0;
uint32_t T32vectSum = 0;
uint32_t T32vect[5] = { 0 };

uint32_t T33pulseWidth = 0;
uint32_t T33Rising = 0;
uint32_t T33Tick = 0;
uint32_t T33vectSum = 0;
uint32_t T33vect[5] = { 0 };

uint32_t T34pulseWidth = 0;
uint32_t T34Rising = 0;
uint32_t T34Tick = 0;
uint32_t T34vectSum = 0;
uint32_t T34vect[5] = { 0 };

// MOTORS - UART
uint8_t rx_buffer[100] = {'\0'};
uint8_t rxChar = '\0';
uint8_t rxIndex = 0;

Motor_Wheel MW[4];
Motor_Wheel* pMW[4];


Linear_Motor LM[2];
Linear_Motor* pLM[2];

uint32_t linearPulse_1 = 0;
uint32_t linearPulse_2 = 0;

uint8_t Init_Done = 0;

void set_linear_motor(uint32_t shift, uint8_t dir)
{
	if (TIM_CHANNEL_STATE_GET(&htim15, TIM_CHANNEL_2) != HAL_TIM_CHANNEL_STATE_BUSY)
	{
		linearPulse_1 = shift;
		// TODO: SET DIRECTION GPIO
		HAL_TIM_PWM_Start_IT(&htim15, TIM_CHANNEL_2);
	}
}

void memset_volatile(volatile void *s, char c, size_t n)
{
    volatile char *p = s;
    while (n-- > 0) {
        *p++ = c;
    }
}

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
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM15_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /*uint8_t my_priority = NVIC_GetPriority( USART2_IRQn );
  uint8_t timer_priority = NVIC_GetPriority( TIM15_IRQn );
  if (timer_priority < my_priority)
	  {
	  NVIC_SetPriority( TIM15_IRQn, my_priority + 1 );
	  }*/
  // --------------------------------------
  // ENCODERS READING
  // --------------------------------------
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
  uint8_t MSG[65] = {'\0'};
  //uint8_t number1[8] = {'\0'};
  //uint8_t number2[8] = {'\0'};
  //uint8_t number3[8] = {'\0'};
  //uint8_t number4[8] = {'\0'};
  float speed1 = 0;
  float speed2 = 0;
  float speed3 = 0;
  float speed4 = 0;
  uint8_t cycleCounter = 0;

  uint32_t sumVect = 0.0;
  uint32_t vect[4][5] = { 0.0 };

  // --------------------------------------
  // MOTORS CONTROL
  // --------------------------------------
  int pulse_counter = 0;
  int stage = 0;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  //HAL_TIM_Base_Start_IT(&htim15);
  //HAL_TIM_PWM_Start_IT(&htim15, TIM_CHANNEL_1);
  //HAL_TIM_PWM_Start_IT(&htim15, TIM_CHANNEL_2);
  //HAL_TIM_OnePulse_Start(&htim15, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 2);
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 2);
  //HAL_TIM_Base_Start(&htim16);
  // Serial OUT temp string
  uint8_t info[100] = {'\0'};

  // SETUP
  for (int i = 0; i < 4; i++)
  {
	  pMW[i] = &MW[i];
	  motor_wheel_init(pMW[i], i+1); // init structure
	  motorPWM_pulse(&htim1, pMW[i], 0.0); // send zero velocities
	  motor_break(pMW[i], 0);  // set brake to false
  }
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);


  // basicly - clear all uart queues
  //__HAL_UART_FLUSH_DRREGISTER(&huart2);
  HAL_UART_Receive_DMA(&huart2, &rxChar, 1);
  HAL_UART_Transmit(&huart2, info, strlen(info), 100);

  //uint8_t str[] = "Проверка передачи UART\r\n\0";

  //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 10);
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

  //uint8_t UART1_rxBuffer[12] = {0};
  //HAL_UART_Receive (&huart1, UART1_rxBuffer, 12, 5000); // read 12 bytes, 5000 ms timeout
  //HAL_UART_Receive_IT(&huart2, &rxChar, 1);

  for (int i = 0; i < 2; i++)
   {
 	  pLM[i] = &LM[i];
 	  linear_motor_init(pLM[i], i+1, i); // init structure
   }
  //linear_motor_set_target(pLM[1], 10000);
  //linear_motor_pulse(pLM[1], &htim15, &linearPulse_1);
  //set_linear_motor(10050, 0);
  Init_Done = 1;
  //HAL_UART_Receive_IT(&huart2, &rxChar, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //sprintf(MSG, "pls\n");
	  //HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 30);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	cycleCounter++;
	if (cycleCounter == 255)
	{
		cycleCounter = 1;
	}
	// --------------------------------------
	// ENCODERS READING
	// --------------------------------------
    if (cycleCounter % 1 == 0) // send odometry every second cycle
	 {
		 speed1 = 0.0;
		 speed2 = 0.0;
		 speed3 = 0.0;
		 speed4 = 0.0;

		 if (T31pulseWidth < 30000 && T31pulseWidth != 0)
		 {
			 speed1 = 13850.4 / T31pulseWidth;
		 }
		 if (T32pulseWidth < 30000 && T32pulseWidth != 0)
		 {
			 speed2 = 13850.4 / T32pulseWidth;
		 }
		 if (T33pulseWidth < 30000 && T33pulseWidth != 0)
		 {
			 speed3 = 13850.4 / T33pulseWidth;
		 }
		 if (T34pulseWidth < 30000 && T34pulseWidth != 0)
		 {
			 speed4 = 13850.4 / T34pulseWidth;
		 }
		 memset(MSG, 0, sizeof(MSG));
		 sprintf(MSG, "[enc] %.2f %.2f %.2f %.2f\n", speed1, speed2, speed3, speed4);
		 //sprintf(MSG, "[enc] %d %d %d %d\n", T31pulseWidth, T32pulseWidth, T33pulseWidth, T34pulseWidth);
		 HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 30);
	 }

	 for (int i=0; i < 4; i++)
	 {
		 sumVect = 0.0;
		 for (int j=0; j < 4; j++)
		 {
			 vect[i][j] = vect[i][j+1];
			 sumVect += vect[i][j] / 5;
		 }
		 if (i==0)
		 {
			 vect[i][4] = T31pulseWidth;
			 sumVect += vect[i][4] / 5;
			 if (abs(T31pulseWidth - sumVect) <= 5) // exact match may differ by 5 or less
				 T31pulseWidth = 0;
		 }
		 else if (i==1)
		 {
			 vect[i][4] = T32pulseWidth;
			 sumVect += vect[i][4] / 5;
			 if (abs(T32pulseWidth - sumVect) <= 5)
				 T32pulseWidth = 0;
		 }
		 else if (i==2)
		 {
			 vect[i][4] = T33pulseWidth;
			 sumVect += vect[i][4] / 5;
			 if (abs(T33pulseWidth - sumVect) <= 5)
				 T33pulseWidth = 0;
		 }
		 else if (i==3)
		 {
			 vect[i][4] = T34pulseWidth;
			 sumVect += vect[i][4] / 5;
			 if (abs(T34pulseWidth - sumVect) <= 5)
				 T34pulseWidth = 0;
		 }
	 }

	 HAL_Delay(100);
	// --------------------------------------
	// MOTORS CONTROL
	// --------------------------------------
	 HAL_UART_Receive_IT(&huart2, &rxChar, 1);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 160-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 80-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 3;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 40-1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 5-1;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 100;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DIR_LEFT_Pin|DIR_RIGHT_Pin|DIR_FRONT_Pin|DIR_REAR_Pin
                          |BREAK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIR_LEFT_Pin DIR_RIGHT_Pin DIR_FRONT_Pin DIR_REAR_Pin
                           BREAK_Pin */
  GPIO_InitStruct.Pin = DIR_LEFT_Pin|DIR_RIGHT_Pin|DIR_FRONT_Pin|DIR_REAR_Pin
                          |BREAK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// --------------------------------------
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim15)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			if (linearPulse_1 > 0)
			{
				linearPulse_1--;
			}
			else if (linearPulse_1 == 0)
			{
				HAL_TIM_PWM_Stop_IT(&htim15, TIM_CHANNEL_1);
			}
		}
		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			if (linearPulse_2 > 0)
			{
				linearPulse_2--;
			}
			else if (linearPulse_2 == 0)
			{
				HAL_TIM_PWM_Stop_IT(&htim15, TIM_CHANNEL_2);
			}
		}
	}
}
// ENCODERS READING
// --------------------------------------
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
    	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			T31Tick = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
			if (T31Tick > T31Rising)
			{
				T31vectSum = 0;
				for (int i = 0; i < 4; i++)
				{
					T31vect[i] = T31vect[i+1];
					T31vectSum += T31vect[i] / 5;
				}
				T31vect[4] = T31Tick - T31Rising;
				T31vectSum += T31vect[4] / 5;
				T31pulseWidth = T31vectSum;
			}
			T31Rising = T31Tick;
		}
    	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
    		T32Tick = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
			if (T32Tick > T32Rising)
			{
				T32vectSum = 0;
				for (int i = 0; i < 4; i++)
				{
					T32vect[i] = T32vect[i+1];
					T32vectSum += T32vect[i] / 5;
				}
				T32vect[4] = T32Tick - T32Rising;
				T32vectSum += T32vect[4] / 5;
				T32pulseWidth = T32vectSum;
			}
			T32Rising = T32Tick;
		}
    	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
        {
    		T33Tick = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);
			if (T33Tick > T33Rising)
			{
				T33vectSum = 0;
				for (int i = 0; i < 4; i++)
				{
					T33vect[i] = T33vect[i+1];
					T33vectSum += T33vect[i] / 5;
				}
				T33vect[4] = T33Tick - T33Rising;
				T33vectSum += T33vect[4] / 5;
				T33pulseWidth = T33vectSum;
			}
			T33Rising = T33Tick;
        }
    	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		{
    		T34Tick = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4);
			if (T34Tick > T34Rising)
			{
				T34vectSum = 0;
				for (int i = 0; i < 4; i++)
				{
					T34vect[i] = T34vect[i+1];
					T34vectSum += T34vect[i] / 5;
				}
				T34vect[4] = T34Tick - T34Rising;
				T34vectSum += T34vect[4] / 5;
				T34pulseWidth = T34vectSum;
			}
			T34Rising = T34Tick;
		}
    }
}

// --------------------------------------
// MOTORS CONTROL
// --------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2) { // Current UART
		rx_buffer[rxIndex++] = rxChar;    // Add data to Rx_Buffer
		HAL_UART_Receive_IT(&huart2, &rxChar, 1);

		if ((rxChar == '\n') && Init_Done == 1)
		{
			uint8_t MSG[5] = {'\0'};
			int arw1=0, arw2=0, arw3=0, arw4=0, motor_brk=0;
		    int turn=0;
			sscanf(rx_buffer, "%s %d %d %d %d %d %d", &MSG, &arw1, &arw2, &arw3, &arw4, &turn, &motor_brk);
			memset(rx_buffer, 0, sizeof(rx_buffer));
			//memset_volatile(rx_buffer, 0, sizeof(rx_buffer));
			rxIndex = 0;
			if (!strcmp(MSG, "[drv]")) // returns 0 if strings are equal
			{
				uint8_t reply[65] = {'\0'};
				sprintf(reply, "received: %d %d %d %d %d %d\n", arw1, arw2, arw3, arw4, turn, motor_brk);
				HAL_UART_Transmit_IT(&huart2, reply, sizeof(reply));
				//motorPWM_pulse(&htim1, pMW[0], ((float) arw1 / 100.0));
				//motorPWM_pulse(&htim1, pMW[1], ((float) arw2 / 100.0));
				//motorPWM_pulse(&htim1, pMW[2], ((float) arw3 / 100.0));
				//motorPWM_pulse(&htim1, pMW[3], ((float) arw4 / 100.0));
				motorPWM_pulse(&htim1, pMW[0], arw1 );
				motorPWM_pulse(&htim1, pMW[1], arw2 );
				motorPWM_pulse(&htim1, pMW[2], arw3 );
				motorPWM_pulse(&htim1, pMW[3], arw4 );
				motor_break(pMW[0], motor_brk);
				//motor_break(pMW[1], motor_brk);
				//motor_break(pMW[2], motor_brk);
				//motor_break(pMW[3], motor_brk);
				linear_motor_set_target(pLM[0], turn);
				linear_motor_set_target(pLM[1], turn);
				linear_motor_pulse(pLM[0], &htim15, &linearPulse_1);
				linear_motor_pulse(pLM[1], &htim15, &linearPulse_2);
			}
			//rxChar = '\0';
		}

	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
