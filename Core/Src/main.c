/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX_LED 	16

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch1_trig;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

uint8_t LED_Data[MAX_LED][4];
uint8_t LED_PWM_Data[MAX_LED * 24];
uint8_t LED_sending;

uint8_t msg_idx;
uint8_t msg[32], ch;
uint8_t mode;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
	LED_sending = 0;
}

void LED_set(uint32_t LED_number, uint8_t red, uint8_t green, uint8_t blue) {
	LED_Data[LED_number][0] = LED_number;
	LED_Data[LED_number][1] = green;
	LED_Data[LED_number][2] = red;
	LED_Data[LED_number][3] = blue;
}

void LED_send(void) {
	while (LED_sending)
		;

	LED_sending = 1;
	memset(LED_PWM_Data, 0, 24 * MAX_LED + 40);

	uint32_t idx = 0;

	for (int i = 0; i < MAX_LED; ++i) {
		uint32_t color = ((LED_Data[i][1] << 16) | (LED_Data[i][2] << 8)
				| (LED_Data[i][3]));

		for (int j = 23; j >= 0; --j) {
			if (color & (1 << j)) {
				LED_PWM_Data[idx] = 0.66 * TIM3->ARR;
			} else {
				LED_PWM_Data[idx] = 0.33 * TIM3->ARR;
			}
			idx++;
		}
	}

	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t*) LED_PWM_Data,
			24 * MAX_LED + 40);
}

void LED_menu() {
	char msg[] = "=============LED MENU=============\r\n"
			"To use LED WS2812B type following commands:\r\n"
			"ON [n]    - to turn on LED with green color, \r\n"
			"            [n] is integer value for LED \r\n"
			"                              brightness\r\n"
			"OFF       - to turn off LED\r\n"
			"PARITY    - to turn on parity mode\r\n"
			"HEARTBEAT - to turn on heartbeat mode\r\n"
			"==================================\r\n\n";

	HAL_UART_Transmit(&huart1, msg, strlen(msg), -1);
}

uint8_t brightness;

void LED_whiteMode() {
	for (int i = 0; i < MAX_LED; ++i) {
		LED_set(i, 0, brightness, 0);
	}
}

void LED_shutdownMode(void) {
	for (int i = 0; i < MAX_LED; ++i) {
		LED_set(i, 0, 0, 0);
	}
}

void LED_parityMode(void) {
	for (int i = 0; i < MAX_LED; i++) {
		if (i % 2) {
			LED_set(i, 255, 40, 0);
		} else {
			LED_set(i, 255, 255, 0);
		}
	}
}

uint8_t toggle_var, toggle_pl;

void LED_toggleMode(void) {
	HAL_Delay(10);

	for (int i = 0; i < MAX_LED; ++i) {
		LED_set(i, toggle_var, 0, 0);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_UART_Transmit(&huart1, &ch, 1, -1);
	if (ch == '\r') {
		char n = '\n';
		HAL_UART_Transmit(&huart1, (uint8_t*) &n, 1, -1);

		uint8_t slice[msg_idx + 1];
		memcpy(slice, msg, msg_idx);
		slice[msg_idx] = '\0';

		if (msg_idx >= 2) {

			if (slice[0] == 'O' && slice[1] == 'N') {
				if (msg_idx == 2) {
					brightness = 255;
					const char *s = "MODE: ON\r\n";
					HAL_UART_Transmit(&huart1, (uint8_t*) s, strlen(s), -1);
					mode = 0;
				} else {
					if (slice[2] == ' ' && msg_idx > 3) {
						uint8_t is_num = 1;

						for (int i = 3; i < msg_idx; ++i) {
							if (!isdigit(slice[i])) {
								is_num = 0;
								break;
							}
						}

						if (is_num) {
							uint32_t num = 0, rank = 1;
							for (int i = msg_idx - 1; i >= 3; --i) {
								num += (slice[i] - '0') * rank;
								rank *= 10;
							}

							if (num > 255)
								num = 255;

							brightness = num;
						}
					}
				}
			} else if (!strcmp((const char*) slice, (const char*) "OFF")) {
				const char *s = "MODE: OFF\r\n";
				HAL_UART_Transmit(&huart1, (uint8_t*) s, strlen(s), -1);
				mode = 1;
			} else if (!strcmp((const char*) slice, (const char*) "PARITY")) {
				const char *s = "MODE: PARITY\r\n";
				HAL_UART_Transmit(&huart1, (uint8_t*) s, strlen(s), -1);
				mode = 2;
			} else if (!strcmp((const char*) slice,
					(const char*) "HEARTBEAT")) {
				const char *s = "MODE: HEARTBEAT\r\n";
				HAL_UART_Transmit(&huart1, (uint8_t*) s, strlen(s), -1);
				mode = 3;
			}
		}
		msg_idx = 0;
		memset(msg, 0, 32);
	} else {
		msg[msg_idx] = ch;
		msg_idx++;
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_DMA_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	LED_sending = 0;
	msg_idx = 0;
	mode = 0;
	ch = ' ';
	toggle_var = 1, toggle_pl = 1;
	brightness = 255;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	LED_menu();

	while (1) {
		HAL_UART_Receive_IT(&huart1, &ch, 1);

		if (mode == 0) {
			LED_whiteMode();
		} else if (mode == 1) {
			LED_shutdownMode();
		} else if (mode == 2) {
			LED_parityMode();
		} else if (mode == 3) {
			LED_toggleMode();
			toggle_var += toggle_pl;
			if (toggle_var == 255) {
				toggle_pl = -1;
			} else if (toggle_var == 0) {
				toggle_pl = 1;
			}
		}
		LED_send();
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 29;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
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
static void MX_USART1_UART_Init(void) {

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
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel4_5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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

