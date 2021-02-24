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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//save status matrix
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
//scan and update of button matrix
uint16_t ButtonMatrixState = 0;
uint32_t ButtonMatrixTimestamp = 0;
uint32_t numnow = 0;
uint8_t state = 1;
uint16_t ButtonMatrixstatepre=0;

enum {
	one=1,two,three,four,five,six,saven,egg,nine,ten,even,twelve,waitclear,clear = 10,BS = 11,ok = 12
	};
void ButtonMatrixUpdate();
void checknumpress();
void q();

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
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		ButtonMatrixUpdate();
		checknumpress();
		q();

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA7 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
GPIO_TypeDef *ButtonmatrixPort[8] = { GPIOA, GPIOB, GPIOB, GPIOB, GPIOA, GPIOC,
		GPIOB, GPIOA };
uint16_t ButtonMatrixPin[8] = { GPIO_PIN_10, GPIO_PIN_3, GPIO_PIN_5, GPIO_PIN_4,
		GPIO_PIN_9, GPIO_PIN_7, GPIO_PIN_6, GPIO_PIN_7 };
uint8_t ButtonMatrixLine = 0;
uint8_t ButtonMatrixRow = 0;
void ButtonMatrixUpdate() {
	if (HAL_GetTick() - ButtonMatrixTimestamp >= 100) {
		ButtonMatrixTimestamp = HAL_GetTick();
		for (int i = 0; i < 4; i++) {
			GPIO_PinState Pinstate;
			Pinstate = HAL_GPIO_ReadPin(ButtonmatrixPort[i],
					ButtonMatrixPin[i]);
			if (Pinstate == GPIO_PIN_RESET) {
				ButtonMatrixState |= (uint16_t) 0x1
						<< (i + ButtonMatrixRow * 4);
			} else {
				ButtonMatrixState &= ~((uint16_t) 0x1
						<< (i + ButtonMatrixRow * 4));
			}

		}
		uint8_t NowoutputPin = ButtonMatrixRow + 4;
		HAL_GPIO_WritePin(ButtonmatrixPort[NowoutputPin],
				ButtonMatrixPin[NowoutputPin], GPIO_PIN_SET);
		ButtonMatrixRow = (ButtonMatrixRow + 1) % 4;
		uint8_t NextoutputPin = ButtonMatrixRow + 4;
		HAL_GPIO_WritePin(ButtonmatrixPort[NextoutputPin],
				ButtonMatrixPin[NextoutputPin], GPIO_PIN_RESET);

	}

}
void checknumpress() {
	if (ButtonMatrixState == 0b1000000000000) {
		numnow=0;
	} else if (ButtonMatrixState == 0b100000000) {
		numnow=1;
	} else if (ButtonMatrixState == 0b1000000000) {
		numnow=2;
	} else if (ButtonMatrixState == 0b10000000000) {
		numnow= 3;
	} else if (ButtonMatrixState == 0b10000) {
		numnow= 4;
	} else if (ButtonMatrixState == 0b100000) {
		numnow= 5;
	} else if (ButtonMatrixState == 0b1000000) {
		numnow= 6;
	} else if (ButtonMatrixState == 0b1) {
		numnow=7;
	} else if (ButtonMatrixState == 0b10) {
		numnow=8;
	} else if (ButtonMatrixState == 0b100) {
		numnow=9;
	} else if (ButtonMatrixState == 0b1000) {
		numnow=10;
	} else if (ButtonMatrixState == 0b10000000) {
		numnow=11;
	} else if (ButtonMatrixState == 0b1000000000000000) {
		numnow= 12;
	} else {
		numnow=255;
	}
}
void q(){

			switch (state) {
				case waitclear:
					if ((ButtonMatrixState != 0) && (ButtonMatrixstatepre!=ButtonMatrixState)){
						if (numnow == clear) {
							state = one;
						}
					}
				break;
	 			case one:
					if ((ButtonMatrixState != 0) && (ButtonMatrixstatepre!=ButtonMatrixState)){
						if (numnow == 6) {
							state = two;
						}else if (numnow == clear){
							state = one;
						}else if (numnow == BS){
							state = one;
						}else if (numnow == ok){
							state = one;
						}else{
							state=waitclear;
						}
					}
				break;
	 			case two:
					if ((ButtonMatrixState != 0) && (ButtonMatrixstatepre!=ButtonMatrixState)){
						if (numnow == 2) {
							state = three;
						}else if (numnow == clear){
							state = one;
						}else if (numnow == BS){
							state = one;
						}else if (numnow == ok){
							state = one;
						}else{
							state=waitclear;
						}
					}
	 			break;
	 			case three:
					if ((ButtonMatrixState != 0) && (ButtonMatrixstatepre!=ButtonMatrixState)){
						if (numnow == 3) {
							state = four;
						}else if (numnow == clear){
							state = one;
						}else if (numnow == BS){
							state = two;
						}else if (numnow == ok){
							state = one;
						}else{
							state=waitclear;
						}
					}
	 			break;
	 			case four:
					if ((ButtonMatrixState != 0) && (ButtonMatrixstatepre!=ButtonMatrixState)){
						if (numnow == 4) {
							state = five;
						}else if (numnow == clear){
							state = one;
						}else if (numnow == BS){
							state = three;
						}else if (numnow == ok){
							state = one;
						}else{
							state=waitclear;
						}
					}
				break;
	 			case five:
					if ((ButtonMatrixState != 0) && (ButtonMatrixstatepre!=ButtonMatrixState)){
						if (numnow == 0) {
							state = six;
						}else if (numnow == clear){
							state = one;
						}else if (numnow == BS){
							state = four;
						}else if (numnow == ok){
							state = one;
						}else{
							state=waitclear;
						}
					}
	 			break;
	 			case six:
					if ((ButtonMatrixState != 0) && (ButtonMatrixstatepre!=ButtonMatrixState)){
						if (numnow == 5) {
							state = saven;
						}else if (numnow == clear){
							state = one;
						}else if (numnow == BS){
							state = five;
						}else if (numnow == ok){
							state = one;
						}else{
							state=waitclear;
						}
					}
	 			break;
	 			case saven:
					if ((ButtonMatrixState != 0) && (ButtonMatrixstatepre!=ButtonMatrixState)){
						if (numnow == 0) {
							state = egg;
						}else if (numnow == clear){
							state = one;
						}else if (numnow == BS){
							state = six;
						}else if (numnow == ok){
							state = one;
						}else{
							state=waitclear;
						}
					}
	 			break;
	 			case egg:
					if ((ButtonMatrixState != 0) && (ButtonMatrixstatepre!=ButtonMatrixState)){
						if (numnow == 0) {
							state = nine;
						}else if (numnow == clear){
							state = one;
						}else if (numnow == BS){
							state = saven;
						}else if (numnow == ok){
							state = one;
						}else{
							state=waitclear;
						}
					}
	 			break;
				case nine:
					if ((ButtonMatrixState != 0) && (ButtonMatrixstatepre!=ButtonMatrixState)){
						if (numnow == 0) {
							state = ten;
						}else if (numnow == clear){
							state = one;
						}else if (numnow == BS){
							state = egg;
						}else if (numnow == ok){
							state = one;
						}else{
							state=waitclear;
						}
					}
	 			break;
				case ten:
					if ((ButtonMatrixState != 0) && (ButtonMatrixstatepre!=ButtonMatrixState)){
						if (numnow == 2) {
							state = even;
						}else if (numnow == clear){
							state = one;
						}else if (numnow == BS){
							state = nine;
						}else if (numnow == ok){
							state = one;
						}else{
							state=waitclear;
						}
					}
				break;
				case even:
					if ((ButtonMatrixState != 0) && (ButtonMatrixstatepre!=ButtonMatrixState)){
						if (numnow == 6) {
							state = twelve;
						}else if (numnow == clear){
							state = one;
						}else if (numnow == BS){
							state = ten;
						}else if (numnow == ok){
							state = one;
						}else{
							state=waitclear;
						}
					}
				break;
				case twelve:
					if ((ButtonMatrixState != 0) && (ButtonMatrixstatepre!=ButtonMatrixState)){
						if (numnow == clear){
							state = one;
						}else if (numnow == BS){
							state = even;
						}else if (numnow == ok){
							HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, RESET);
							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
							state = one;
						}else{
							state=waitclear;
						}
					}
				break;
			}
			ButtonMatrixstatepre=ButtonMatrixState;
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
	while (1) {
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
