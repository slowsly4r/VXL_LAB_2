/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void display7SEG(int num);
void update7SEG(int index);
void displayLEDMatrix(int num);
void updateLEDMatrix(uint8_t index);
void LEDMatrixAnimation();
void updateClockBuffer();
void setSegTimer(int duration);
void segTimer_run();
void setDotTimer(int duration);
void dotTimer_run();
void setClockTimer(int duration);
void clockTimer_run();
void setMatrixTimer(int duration);
void matrixTimer_run();
void ledClear();
void setAnimationTimer(int duration);
void animationTimer_run();
void matrixClear();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Define 4 seven-segment states
enum segState {seg0 = 0, seg1 = 1, seg2 = 2, seg3 = 3};
enum segState seg = seg0;
// Create an enable pins array to control all 4 seven-segment
int controll7SegPin[4] = {EN0_Pin, EN1_Pin, EN2_Pin, EN3_Pin};
// Create two arrays to control 8 columns and 8 rows of LED matrix
int colPin[8] = {ENM0_Pin, ENM1_Pin, ENM2_Pin, ENM3_Pin, ENM4_Pin, ENM5_Pin, ENM6_Pin, ENM7_Pin};
int rowPin[8] = {ROW0_Pin, ROW1_Pin, ROW2_Pin, ROW3_Pin, ROW4_Pin, ROW5_Pin, ROW6_Pin, ROW7_Pin};
const int MAX_LED = 4;
const int MAX_LED_MATRIX = 8;
int index_led = 0;
int led_buffer[4] = {1, 2, 3, 4};
int index_led_matrix = 0;
uint8_t matrix_buffer[8] = {0b11100111, 0b11000011, 0b10011001, 0b10011001,
							0b10000001, 0b10000001, 0b10011001, 0b10011001};
int hour = 15, minute = 8, second = 50;
int segTimer_counter = 0;
int segTimer_flag = 0;
int dotTimer_counter = 0;
int dotTimer_flag = 0;
int clockTimer_counter = 0;
int clockTimer_flag = 0;
int matrixTimer_counter = 0;
int matrixTimer_flag = 0;
int animationTimer_counter = 0;
int animationTimer_flag = 0;
int TIMER_CYCLE = 1;

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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  setSegTimer(100);
  setDotTimer(500);
  setClockTimer(1000);
  setMatrixTimer(2);
  setAnimationTimer(100);
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // UPDATE SEVEN-SEGMENTS
	  if (segTimer_flag == 1) {
		  updateClockBuffer();
		  update7SEG(index_led++);
		  if (index_led >= 4) index_led = 0;
		  setSegTimer(100);
	  }
	  // UPDATE LED RED AND DOT
	  if (dotTimer_flag == 1) {
		  HAL_GPIO_TogglePin(DOT_GPIO_Port, DOT_Pin);
		  HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		  setDotTimer(500);
	  }
	  // UPDATE CLOCK
	  if (clockTimer_flag == 1) {
		  second++;
		  if (second >= 60) {
			  second = 0;
			  minute++;
		  }
		  if(minute >= 60) {
			  minute = 0;
			  hour++;
		   }
		   if(hour >= 24){
			  hour = 0;
		   }
		  updateClockBuffer();
		  setClockTimer(1000);
	  }
	  // UPDATE LED MATRIX
	  if (matrixTimer_flag == 1) {
		  updateLEDMatrix(index_led_matrix++);
		  if (index_led_matrix > 7) index_led_matrix = 0;
		  setMatrixTimer(2);
	  }
	  // UPDATE LED MATRIX ANIMATION
	  if (animationTimer_flag == 1) {
		  LEDMatrixAnimation();
		  setAnimationTimer(100);
	  }
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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 799;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ENM0_Pin|ENM1_Pin|DOT_Pin|LED_RED_Pin
                          |EN0_Pin|EN1_Pin|EN2_Pin|EN3_Pin
                          |ENM2_Pin|ENM3_Pin|ENM4_Pin|ENM5_Pin
                          |ENM6_Pin|ENM7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SEG0_Pin|SEG1_Pin|SEG2_Pin|ROW2_Pin
                          |ROW3_Pin|ROW4_Pin|ROW5_Pin|ROW6_Pin
                          |ROW7_Pin|SEG3_Pin|SEG4_Pin|SEG5_Pin
                          |SEG6_Pin|ROW0_Pin|ROW1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ENM0_Pin ENM1_Pin DOT_Pin LED_RED_Pin
                           EN0_Pin EN1_Pin EN2_Pin EN3_Pin
                           ENM2_Pin ENM3_Pin ENM4_Pin ENM5_Pin
                           ENM6_Pin ENM7_Pin */
  GPIO_InitStruct.Pin = ENM0_Pin|ENM1_Pin|DOT_Pin|LED_RED_Pin
                          |EN0_Pin|EN1_Pin|EN2_Pin|EN3_Pin
                          |ENM2_Pin|ENM3_Pin|ENM4_Pin|ENM5_Pin
                          |ENM6_Pin|ENM7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG0_Pin SEG1_Pin SEG2_Pin ROW2_Pin
                           ROW3_Pin ROW4_Pin ROW5_Pin ROW6_Pin
                           ROW7_Pin SEG3_Pin SEG4_Pin SEG5_Pin
                           SEG6_Pin ROW0_Pin ROW1_Pin */
  GPIO_InitStruct.Pin = SEG0_Pin|SEG1_Pin|SEG2_Pin|ROW2_Pin
                          |ROW3_Pin|ROW4_Pin|ROW5_Pin|ROW6_Pin
                          |ROW7_Pin|SEG3_Pin|SEG4_Pin|SEG5_Pin
                          |SEG6_Pin|ROW0_Pin|ROW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	segTimer_run();
	dotTimer_run();
	clockTimer_run();
	matrixTimer_run();
	animationTimer_run();
}
void display7SEG(int num) {
		/*
		 * TRUTH TABLE
		 *      dp g f e d c b a
		 * 0 => 1 1 0 0 0 0 0 0 => 0xC0
		 * 1 => 1 1 1 1 1 0 0 1 => 0xF9
		 * 2 => 1 1 1 0 0 1 0 0 => 0xA4
		 * 3 => 1 0 1 1 0 0 0 0 => 0xB0
		 * 4 => 1 0 0 1 1 0 0 1 => 0x99
		 * 5 => 1 0 0 1 0 0 1 0 => 0x92
		 * 6 => 1 0 0 0 0 0 1 0 => 0x82
		 * 7 => 1 1 1 1 1 0 0 0 => 0xF8
		 * 8 => 1 0 0 0 0 0 0 0 => 0x80
		 * 9 => 1 0 0 1 0 0 0 0 => 0x90
		 */
	char ledNum[10] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90};
	for (int i = 0; i < 7; i++) {
		// Shift GPIO_PIN_0 left by i, right shift so the bit for segment i is in the LSB position
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 << i, (ledNum[num] >> i) & 1);
	}
}
void update7SEG(int index) {
	ledClear();
	switch (index){
	case 0:
		// Display the first 7 SEG with led_buffer [0]
		HAL_GPIO_WritePin(GPIOA, controll7SegPin[index], RESET);
		display7SEG(led_buffer[0]);
		break;
	case 1:
		// Display the second 7 SEG with led_buffer [1]
		HAL_GPIO_WritePin(GPIOA, controll7SegPin[index], RESET);
		display7SEG(led_buffer[1]);
		break;
	case 2:
		// Display the third 7 SEG with led_buffer [2]
		HAL_GPIO_WritePin(GPIOA, controll7SegPin[index], RESET);
		display7SEG(led_buffer[2]);
		break;
	case 3:
		// Display the fourth 7 SEG with led_buffer [3]
		HAL_GPIO_WritePin(GPIOA, controll7SegPin[index], RESET);
		display7SEG(led_buffer[3]);
		break;
	default:
		break;
	}
}
void displayLEDMatrix(int num) {
	for (int i = 0; i < 8; i++) {
		HAL_GPIO_WritePin(GPIOA,  colPin[i], (matrix_buffer[num] >> i) & 1);
	}
}
void updateLEDMatrix(uint8_t index) {
	matrixClear();
	switch (index) {
		case 0:
			// Display the first row
			HAL_GPIO_WritePin(GPIOB, rowPin[index], RESET);
			displayLEDMatrix(0);
			break ;
		case 1:
			// Display the second row
			HAL_GPIO_WritePin(GPIOB, rowPin[index], RESET);
			displayLEDMatrix(1);
			break ;
		case 2:
			// Display the third row
			HAL_GPIO_WritePin(GPIOB, rowPin[index], RESET);
			displayLEDMatrix(2);
			break ;
		case 3:
			// Display the fourth row
			HAL_GPIO_WritePin(GPIOB, rowPin[index], RESET);
			displayLEDMatrix(3);
			break ;
		case 4:
			// Display the fifth row
			HAL_GPIO_WritePin(GPIOB, rowPin[index], RESET);
			displayLEDMatrix(4);
		break ;
			case 5:
			// Display the sixth row
			HAL_GPIO_WritePin(GPIOB, rowPin[index], RESET);
			displayLEDMatrix(5);
			break;
		case 6:
			// Display the seventh row
			HAL_GPIO_WritePin(GPIOB, rowPin[index], RESET);
			displayLEDMatrix(6);
			break ;
		case 7:
			// Display the eighth row
			HAL_GPIO_WritePin(GPIOB, rowPin[index], RESET);
			displayLEDMatrix(7);
			break ;
		default:
			break;
	}
}
void LEDMatrixAnimation() {
	for (int i = 0; i < 8; i++) {
		uint8_t temp = (matrix_buffer[i] & 0b00000001) << 7;
		matrix_buffer[i] = (matrix_buffer[i] >> 1) | temp;
	}
}
void updateClockBuffer() {
	// Generate values for led_buffer base on the values of hour and minute
	led_buffer[0] = hour / 10;
	led_buffer[1] = hour % 10;
	led_buffer[2] = minute / 10;
	led_buffer[3] = minute % 10;
}
void setSegTimer(int duration) {
	segTimer_counter = duration / TIMER_CYCLE;
	segTimer_flag = 0;
}
void segTimer_run() {
	if (segTimer_counter > 0) {
		segTimer_counter--;
		if (segTimer_counter == 0) segTimer_flag = 1;
	}
}
void setDotTimer(int duration) {
	dotTimer_counter = duration / TIMER_CYCLE;
	dotTimer_flag = 0;
}
void dotTimer_run() {
	if (dotTimer_counter > 0) {
		dotTimer_counter--;
		if (dotTimer_counter == 0) dotTimer_flag = 1;
	}
}
void setClockTimer(int duration){
	clockTimer_counter = duration / TIMER_CYCLE;
	clockTimer_flag = 0;
}
void clockTimer_run(){
	if (clockTimer_counter > 0) {
		clockTimer_counter--;
		if (clockTimer_counter == 0) clockTimer_flag = 1;
	}
}
void setMatrixTimer(int duration){
	matrixTimer_counter = duration / TIMER_CYCLE;
	matrixTimer_flag = 0;
}
void matrixTimer_run(){
	if (matrixTimer_counter > 0) {
		matrixTimer_counter--;
		if (matrixTimer_counter == 0) matrixTimer_flag = 1;
	}
}
void setAnimationTimer(int duration){
	animationTimer_counter = duration / TIMER_CYCLE;
	animationTimer_flag = 0;
}
void animationTimer_run(){
	if (animationTimer_counter > 0) {
		animationTimer_counter--;
		if (animationTimer_counter == 0) animationTimer_flag = 1;
	}
}
void ledClear() {
	HAL_GPIO_WritePin(GPIOA, EN0_Pin | EN1_Pin | EN2_Pin | EN3_Pin, SET);
	HAL_GPIO_WritePin(GPIOB, SEG0_Pin | SEG1_Pin | SEG2_Pin |
			SEG3_Pin | SEG4_Pin | SEG5_Pin | SEG6_Pin, SET);
}
void matrixClear() {
	HAL_GPIO_WritePin(GPIOB, ROW0_Pin | ROW1_Pin | ROW2_Pin | ROW3_Pin | ROW4_Pin
			| ROW5_Pin | ROW6_Pin | ROW7_Pin, SET);
	HAL_GPIO_WritePin(GPIOA, ENM0_Pin | ENM1_Pin | ENM2_Pin | ENM3_Pin| ENM4_Pin
			| ENM5_Pin| ENM6_Pin| ENM7_Pin, SET);
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
