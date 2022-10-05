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
volatile char flag = 0, btn_open_pressed = 0, btn_close_pressed = 0;
volatile char opened_door_pressed = 0, closed_door_pressed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void doorTask();
void closeDoor();
void openDoor();
void stopDoor();
void avoidBounds();
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

	  doorTask();
	  avoidBounds();

	  //while(0==flag);
	  flag=0;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
  htim2.Init.Prescaler = 7200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, motor_driver_1_Pin|motor_driver_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : motor_driver_1_Pin motor_driver_2_Pin */
  GPIO_InitStruct.Pin = motor_driver_1_Pin|motor_driver_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : btn_close_Pin btn_open_Pin */
  GPIO_InitStruct.Pin = btn_close_Pin|btn_open_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : opened_door_Pin closed_door_Pin */
  GPIO_InitStruct.Pin = opened_door_Pin|closed_door_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void doorTask() {
	static int32_t tenMilis = 0;
	static char stage = 0;

	switch(stage) {
		// Door closed
		case 0:
			stopDoor();

			if (btn_open_pressed == 1) {
				stage = 1;
			}
			break;
		// Opening door
		case 1:
			openDoor();

			if (opened_door_pressed == 1) {
				stage = 2;
			}
			break;
		// Door opened
		case 2:
			if (btn_close_pressed == 1) {
				stage = 3;
			}
		// Closing door
		case 3:
			closeDoor();

			if (closed_door_pressed == 1) {
				stage = 0;
			}
		}
		tenMilis++;
}

void openDoor() {
	HAL_GPIO_WritePin(motor_driver_1_GPIO_Port, motor_driver_1_Pin, 1);
	HAL_GPIO_WritePin(motor_driver_2_GPIO_Port, motor_driver_2_Pin, 0);
}

void closeDoor() {
	HAL_GPIO_WritePin(motor_driver_1_GPIO_Port, motor_driver_1_Pin, 0);
	HAL_GPIO_WritePin(motor_driver_2_GPIO_Port, motor_driver_2_Pin, 1);
}

void stopDoor() {
	HAL_GPIO_WritePin(motor_driver_1_GPIO_Port, motor_driver_1_Pin, 0);
	HAL_GPIO_WritePin(motor_driver_2_GPIO_Port, motor_driver_2_Pin, 0);
}

uint32_t listenClick(GPIO_TypeDef *port, uint16_t pin) {
	static char ones = 0, zeros = 0;

	if(HAL_GPIO_ReadPin(port, pin) == 1) {
		zeros = 0;
		ones++;
	} else {
		zeros++;
		ones = 0;
	}

	if(ones == 3) {
		return 1;
	} else {
		return 0;
	}
}

void avoidBounds() {
	static char zeros_closed_door = 0, ones_closed_door = 0;
	static char zeros_opened_door = 0, ones_opened_door = 0;
	static char zeros_btn_open = 0, ones_btn_open = 0;
	static char zeros_btn_close = 0, ones_btn_close = 0;
	static char stage = 0;

	if (HAL_GPIO_ReadPin(closed_door_GPIO_Port, closed_door_Pin) == 1) {
		zeros_closed_door = 0;
		ones_closed_door++;

		// button pressed just when there are at least 30 ms
		if (ones_btn_close == 3) {
			closed_door_pressed = 1;
		} else if(zeros_btn_close = 3) {
			closed_door_pressed = 0;
		}
	} else {
		zeros_closed_door++;
		ones_closed_door = 0;
	}

	if (HAL_GPIO_ReadPin(opened_door_GPIO_Port, opened_door_Pin) == 1) {
		zeros_opened_door = 0;
		ones_opened_door++;

		if (ones_btn_close == 3) {
			btn_close_pressed = 1;
		} else if(zeros_btn_close = 3) {
			btn_close_pressed = 0;
		}
	} else {
		zeros_opened_door++;
		ones_opened_door = 0;
	}

	if (HAL_GPIO_ReadPin(btn_close_GPIO_Port, btn_close_Pin) == 1) {
		zeros_btn_close = 0;
		ones_btn_close++;

		if (ones_btn_close == 3) {
			btn_close_pressed = 1;
		} else if(zeros_btn_close = 3) {
			btn_close_pressed = 0;
		}
	} else {
		zeros_btn_close++;
		ones_btn_close = 0;
	}

	if (HAL_GPIO_ReadPin(btn_open_GPIO_Port, btn_open_Pin) == 1) {
		zeros_btn_open = 0;
		ones_btn_open++;

		if (ones_btn_close == 3) {
			btn_open_pressed = 1;
		} else if(zeros_btn_close = 3) {
			btn_open_pressed = 0;
		}
	} else {
		zeros_btn_open++;
		ones_btn_open = 0;
	}

	/*
	switch(stage){
		case 0://espera a detectar un nivel alto estable
		   if(ones_opened_door == 3) {
			   stage = 0;
		   }
		   break;
		case 1://espera a detectar un nivel bajo estable
		   if(3==zeros_btn_close) {
			   etapa=0;
		   }
		   break;
	} */

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
