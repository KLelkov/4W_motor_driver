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
//#include "stm32f0xx_hal.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_BUSY 1
#define UART_PACKET_OK 0
#define UART_PACKET_TOO_LONG 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM15_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void UART_Send (const uint8_t message[]);
void drv_messageCheck(const char message[]);
void cal_messageCheck(const char message[]);
void calculate_angles(float *frontAngle, float *rearAngle);
void calculate_pulses(int32_t frontAngle, int32_t rearAngle);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ENCODERS
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
uint8_t UART2_rxBuffer = '\000';
uint8_t UART_newMessage = 0;
char rxString[100] = { '\0' };
volatile uint8_t UART_TX_Busy = 0;

// MOTORS - Motor Wheels
Motor_Wheel MW[4];
Motor_Wheel* pMW[4];

// MOTORS - Linear Motors
Linear_Motor LM[2];
Linear_Motor* pLM[2];

uint32_t linearPulse_1 = 0;
uint32_t linearPulse_2 = 0;

// INITIALIZATION FLAG
uint8_t Init_Done = 0;

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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(1000);

  // --------------------------------------
  // ENCODERS READING
  // --------------------------------------
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
  uint8_t MSG[65] = {'\0'};
  float speed1 = 0;
  float speed2 = 0;
  float speed3 = 0;
  float speed4 = 0;
  float gamma1 = 0;
  float gamma2 = 0;
  uint8_t cycleCounter = 0;

  uint32_t sumVect = 0.0;
  uint32_t vect[4][5] = { 0.0 };

  // --------------------------------------
  // MOTORS CONTROL
  // --------------------------------------
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 2);
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 2);

  // SETUP
  for (int i = 0; i < 4; i++)
  {
	  pMW[i] = &MW[i];
	  motor_wheel_init(pMW[i], i+1); // init structure
	  motorPWM_pulse(&htim1, pMW[i], 0.0); // send zero velocities
	  motor_break(pMW[i], 0);  // set brake to false
  }


  // basicly - clear all uart queues
  HAL_UART_Receive_DMA(&huart3, &UART2_rxBuffer, 1);
  HAL_UART_Transmit(&huart3, MSG, strlen(MSG), 50);


  for (int i = 0; i < 2; i++)
  {
	  pLM[i] = &LM[i];
	  linear_motor_init(pLM[i], i+1, i); // init structure
  }





  // No incoming processing should be done before it is set
  Init_Done = 1;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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
			 speed1 = 13900.0 / T31pulseWidth;
		 }
		 if (T32pulseWidth < 30000 && T32pulseWidth != 0)
		 {
			 speed2 = 13900.0 / T32pulseWidth;
		 }
		 if (T33pulseWidth < 30000 && T33pulseWidth != 0)
		 {
			 speed3 = 13900.0 / T33pulseWidth;
		 }
		 if (T34pulseWidth < 30000 && T34pulseWidth != 0)
		 {
			 speed4 = 13900.0 / T34pulseWidth;
		 }
		 calculate_angles(&gamma1, &gamma2);
		 memset(MSG, 0, sizeof(MSG));
		 sprintf(MSG, "[enc] %.2f %.2f %.2f %.2f %.2f %.2f\n", speed1, speed2, speed3, speed4, gamma1, gamma2);
		 //sprintf(MSG, "[enc] %d %d %d %d %d %d\n", T31pulseWidth, T32pulseWidth, T33pulseWidth, T34pulseWidth, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7), HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13));
		 UART_Send(MSG);

		 //memset(MSG, 0, sizeof(MSG));
		 //sprintf(MSG, "status: %d %d\n", HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7), HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13));
		 //UART_Send(MSG);
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

	 HAL_Delay(50);
	// --------------------------------------
	// MOTORS CONTROL
	// --------------------------------------
	 if ((UART_newMessage == 1) && (cycleCounter % 2 == 0))
	 {
		 // check if received string contains [drv] message and parse it
		 drv_messageCheck(rxString);
		 cal_messageCheck(rxString);
		 // clear received b
		 memset(rxString, 0, sizeof(rxString));
		 // set newMessage flag to 0 to begin new string receive
		 UART_newMessage = 0;
	 }
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
  htim15.Init.Prescaler = 160-1;
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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DIR_LEFT_Pin|DIR_RIGHT_Pin|DIR_FRONT_Pin|DIR_REAR_Pin
                          |BREAK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_LEFT_Pin DIR_RIGHT_Pin DIR_FRONT_Pin DIR_REAR_Pin
                           BREAK_Pin */
  GPIO_InitStruct.Pin = DIR_LEFT_Pin|DIR_RIGHT_Pin|DIR_FRONT_Pin|DIR_REAR_Pin
                          |BREAK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void UART_Send(const uint8_t message[])
{
	while(UART_TX_Busy){};
	UART_TX_Busy = 1;
	HAL_UART_Transmit_DMA(&huart3, (uint8_t*)message, strlen(message));
}

void drv_messageCheck(const char message[])
{
	// create a local copy of the incoming string, just it case
	static char cmd_buf[100];
	strcpy(cmd_buf, message);

	uint8_t MSG[5] = {'\0'};
	int arw1=0, arw2=0, arw3=0, arw4=0, motor_brk=0;
	int turn = 0;
	sscanf(cmd_buf, "%s %d %d %d %d %d %d", &MSG, &arw1, &arw2, &arw3, &arw4, &turn, &motor_brk);
	if (!strcmp(MSG, "[drv]")) // returns 0 if strings are equal
	{
		uint8_t reply[40] = {'\0'};
		sprintf(reply, "received: %d %d %d %d %d %d \n", arw1, arw2, arw3, arw4, turn, motor_brk);
		UART_Send(reply);

		// Since only 1 break pin is used, it is enough to call this function for only 1 wheel
		motor_break(pMW[0], motor_brk);

		// Direction is set separately for left and right sided wheels
		if (arw1 < 0 && arw3 < 0)
		{
			motor_DIR(pMW[0], 1); // set direction to backward
		}
		else
		{
			motor_DIR(pMW[0], 0); // set direction to forward
		}
		if (arw2 < 0 && arw4 < 0)
		{
			motor_DIR(pMW[1], 1); // set direction to backward
		}
		else
		{
			motor_DIR(pMW[1], 0); // set direction to forward
		}

		motorPWM_pulse(&htim1, pMW[0], arw1 );
		motorPWM_pulse(&htim1, pMW[1], arw2 );
		motorPWM_pulse(&htim1, pMW[2], arw3 );
		motorPWM_pulse(&htim1, pMW[3], arw4 );

		// Positive turn direction is RIGHT
		if (abs(turn) > 25)
		{
			turn = turn / abs(turn) * 25;
		}
		calculate_pulses(turn, -turn);
		//linear_motor_set_target(pLM[0], turn);
		//linear_motor_set_target(pLM[1], turn);
		//linear_motor_pulse(pLM[0], &htim15, &linearPulse_1);
		//linear_motor_pulse(pLM[1], &htim15, &linearPulse_2);
	}
}

void cal_messageCheck(const char message[])
{
	// create a local copy of the incoming string, just it case
	static char cmd_buf[100];
	strcpy(cmd_buf, message);

	uint8_t MSG[5] = {'\0'};
	sscanf(cmd_buf, "%s", &MSG);
	if (!strcmp(MSG, "[cal]")) // returns 0 if strings are equal
	{
		uint8_t reply[] = "received calibration command\n";
		UART_Send(reply);
		uint32_t flag = linear_motor_calibrate(pLM[0], &htim15, &linearPulse_1);
		if (flag == 0)
		{
			UART_Send("Front motor calibration timed out! Check for mechanical problems and repeat calibration.");
		}
		flag = linear_motor_calibrate(pLM[1], &htim15, &linearPulse_2);
		if (flag == 0)
		{
			UART_Send("Rear motor calibration timed out! Check for mechanical problems and repeat calibration.");
		}
	}
}

void calculate_angles(float *frontAngle, float *rearAngle)
{
    // Positive turn direction is RIGHT
	int32_t posFront = linear_motor_get_position(pLM[0]);
	int32_t posRear = linear_motor_get_position(pLM[1]);

	*frontAngle = round((posFront - 75.5498) / (- 253.6124));
	if (posRear > 0)
	{
		*rearAngle = round((posRear + 159.8128) / (242.2376));
	}
	else if (posRear < 0)
	{
		*rearAngle = round((posRear + 332.1803) / (314.4046));
	}
	if (posRear == 0)
	{
		*rearAngle = 0;
	}

}

void calculate_pulses(int32_t frontAngle, int32_t rearAngle)
{
    // Positive turn direction is RIGHT
	int32_t pulses_front = -253.6124 * frontAngle + 75.5498;
	int32_t pulses_rear = 0;
	if (rearAngle > 0)
	{
		pulses_rear = 242.2376 * rearAngle - 159.8128;
	}
	else if (rearAngle < 0)
	{
		pulses_rear = 314.4046 * rearAngle - 332.1803;
	}

	// Positive turn direction is RIGHT
	linear_motor_set_target(pLM[0], pulses_front);
	linear_motor_set_target(pLM[1], pulses_rear);
	linear_motor_pulse(pLM[0], &htim15, &linearPulse_1);
	linear_motor_pulse(pLM[1], &htim15, &linearPulse_2);
}

// Linear Motors Timers
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

// UART Processing
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart1)
{
	UART_TX_Busy = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3 && UART_newMessage != 1 && Init_Done == 1)
	{ // Current UART
		static short int UART2_rxindex = 0;
		static uint8_t UART2_ErrorFlag = UART_PACKET_OK;

		if (UART2_rxBuffer == '\n') // If Enter
		{
			if (UART2_ErrorFlag == UART_PACKET_OK && UART2_rxindex)
			{
				rxString[UART2_rxindex] = 0;
				UART2_rxindex = 0;
				UART_newMessage = 1;
				// once newMessage flag is set, received message will be
				// processed in the next main( while{} ) cycle
			}
			else
			{
				UART2_ErrorFlag = UART_PACKET_OK; // reset error state
			}
		}
		else
		{
			if (UART2_rxBuffer != '\r' && UART2_ErrorFlag == UART_PACKET_OK) // Ignore return
			{
				rxString[UART2_rxindex] = UART2_rxBuffer; // Add that character to the string
				UART2_rxindex++;
				if (UART2_rxindex >= 100) // User typing too much, we can't have commands that big
				{
					UART2_ErrorFlag = UART_PACKET_TOO_LONG;
					UART2_rxindex = 0;
					rxString[UART2_rxindex] = '\000';
				}
			}
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
