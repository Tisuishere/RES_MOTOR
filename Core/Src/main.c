/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//uint8_t *txByte= " hello\n";
#define ADC_FILTER_SIZE 5
uint32_t adcBuffer[ADC_FILTER_SIZE] = { 0 };
uint8_t adcIndex = 0;
uint32_t adcVal;
uint32_t lastAdcVal;
uint32_t threshold = 20;
uint16_t step_need = 0;
uint16_t step_run = 0;
uint16_t step_curr = 0;
uint16_t step_new = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
//  HAL_ADC_Start_IT(&hadc1);
//	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
//	HAL_ADC_Start(&hadc1);
//	if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
//		lastAdcVal = HAL_ADC_GetValue(&hadc1);
//	}
//	HAL_ADC_Stop(&hadc1);
	HAL_ADC_Start_IT(&hadc1);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
//	  HAL_ADC_Start(&hadc1);
//	  HAL_ADC_PollForConversion(&hadc1, 1000);
//	  HAL_Delay(20);
//	  adcVal = HAL_ADC_GetValue(&hadc1);
//	  char buffer_adc[50];
//	  int length = snprintf(buffer_adc, sizeof(buffer_adc), "adc_value: %lu \r\n", adcVal);
//	  HAL_UART_Transmit(&huart1, (uint8_t *)buffer_adc, length, 1000);
//	  HAL_Delay(1000);
//	  read_ADC();
//		control();
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
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 8 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1000 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 500 - 1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

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
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_RESET);

	/*Configure GPIO pins : PA1 PA2 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void printf_uart(const char *mess) {
	HAL_UART_Transmit(&huart1, (uint8_t*) mess, strlen(mess), 500);
}
void read_ADC() {
	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
		lastAdcVal = HAL_ADC_GetValue(&hadc1);
	}
//	uint32_t newVal = HAL_ADC_GetValue(&hadc1);
//	HAL_ADC_Stop(&hadc1);
//
//	adcBuffer[adcIndex] = newVal;
//	adcIndex = (adcIndex + 1) % ADC_FILTER_SIZE;
//
//	uint32_t sum = 0;
//	for (uint8_t i = 0; i < ADC_FILTER_SIZE; i++) {
//		sum += adcBuffer[i];
//	}
//	adcVal = sum / ADC_FILTER_SIZE;
}

//void control() {
//	read_ADC();
//	if (adcVal > abs(lastAdcVal - threshold)){
////			|| (adcVal < lastAdcVal - threshold)) {
//		lastAdcVal = adcVal;
//
////		char buffer_adc[50];
////		int length = snprintf(buffer_adc, sizeof(buffer_adc),
////				"ADC value: %lu\r\n", adcVal);
////		HAL_UART_Transmit(&huart1, (uint8_t*) buffer_adc, length, 1000);
////
////		if (step_curr == 0) {
////			printf_uart("ok\n");
//			step_need = adcVal * 200 / 4036;
//
//			HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
//		} else {
//			step_new = adcVal * 200 / 4036;
//			if (step_curr > step_new) {
////				printf_uart("ok1\n");
//				step_need = step_curr - step_new;
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); // ngược
//				HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
//
//			} else if (step_curr < step_new) {
////				printf_uart("ok2\n");
//				step_need = step_new - step_curr;
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);   //  thuận
//				HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
//			} else {
//				// Không thay đổi vị trí
//			}
//		}
//	}
//}

//void control(){
//	read_ADC();
//	if (step_curr == 0){
//		  		  printf_uart("ok");
//		  		  step_need = adcVal*200/4036;
//	//	  		  char ste_buff_need[50];
//	//	  		  int length1 = snprintf(ste_buff_need, sizeof(ste_buff_need), "ste_need_value: %lu \r\n", step_need);
//	//	  		  HAL_UART_Transmit(&huart1, (uint8_t *)ste_buff_need, length1, 1000);
//		  		  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
//		  	  }
//		  	  else
//		  		  step_new = adcVal*200/4036;
//
//		  	  if (step_curr > step_new){
//		  		  printf_uart("ok1");
//		  		  step_need = step_curr - step_new;
//		  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
//		  //		  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
//		  	  }
//		  	  else if (step_curr < step_new){
//		  		  printf_uart("ok2");
//		  		  step_need = step_new - step_curr;
//		  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
//		  //		  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
//		  	  }
//		  	  else {
//		  		  // vì timer dừng sẵn nên đã kh quay -> kh cần disable
//		  //		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//		  	  }
//		  	  lastAdcVal = adcVal;
//
//		  //	  HAL_Delay(1000);
//}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		// nên xử lí thêm nhiễu qua threhold ?
		adcVal = HAL_ADC_GetValue(&hadc1);
//	  if ((adcVal > lastAdcVal + threshold) || (adcVal < lastAdcVal - threshold))
//	          {
		lastAdcVal = adcVal; // cập nhật lại giá trị cũ
		char buffer_adc[50];
		int length = snprintf(buffer_adc, sizeof(buffer_adc),
				"adc_value: %lu \r\n", lastAdcVal);
		HAL_UART_Transmit(&huart1, (uint8_t*) buffer_adc, length, 1000);
//	          }
		if (step_curr == 0) {
			printf_uart("ok");
			step_need = adcVal * 200 / 4036;
			char ste_buff_need[50];
			int length1 = snprintf(ste_buff_need, sizeof(ste_buff_need),
					"step_need_value: %lu \r\n", step_need);
			HAL_UART_Transmit(&huart1, (uint8_t*) ste_buff_need, length1, 1000);
			HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
		} else {
			printf_uart("Chay vao else \n");
			step_new = adcVal * 200 / 4036;
		}

		if (step_curr > step_new) {
			printf_uart("bi dao");
			step_need = step_curr - step_new;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
		} else if (step_curr < step_new) {
			printf_uart("xoay tiep");
			step_need = step_new - step_curr;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
			HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
		} else {
			// vì timer dừng sẵn nên đã kh quay -> kh cần disable
//		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
		}
		lastAdcVal = adcVal;

//	  HAL_Delay(1000);
//	  HAL_ADC_Start_IT(&hadc1);
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) {
		// xử lí count
		step_run++;
//		printf_uart("Chay vao PWM \n");
		if (step_run >= step_need) {
			printf_uart("ok3");
			if (step_curr == 0) {
				step_curr = step_need;
				char buffer_step_curr[50];
				int length1 = snprintf(buffer_step_curr,
						sizeof(buffer_step_curr), "step_curr value: %lu\r\n",
						step_curr);
				HAL_UART_Transmit(&huart1, (uint8_t*) buffer_step_curr, length1,
						1000);
			} else{
				step_curr = step_new;}
			step_run = 0;
			HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
			printf_uart("Chay toi day \n");
			HAL_ADC_Start_IT(&hadc1);
		}
	}
}
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
#ifdef USE_FULL_ASSERT
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
