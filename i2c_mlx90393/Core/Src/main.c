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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MLX90393_Address 12 << 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

/* USER CODE BEGIN PV */
uint8_t pTxData[1];
uint8_t pRxData[7];

HAL_StatusTypeDef state;

volatile uint8_t flag_exti1 = 0;
volatile uint8_t isDataReady = 0;

uint8_t count = 8;
int32_t xSum;
int32_t ySum;
int32_t zSum;

// Вычисленный результат
int32_t xResult;
int32_t yResult;
int32_t zResult;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
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
  __HAL_RCC_I2C1_CLK_ENABLE();
  HAL_Delay(100);
  __HAL_RCC_I2C1_FORCE_RESET();
  HAL_Delay(100);
  __HAL_RCC_I2C1_RELEASE_RESET();
  HAL_Delay(100);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  pTxData[0] = 0x80; // Выполняем команду Exit
  state = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) MLX90393_Address, pTxData, 1, HAL_MAX_DELAY);
  state = HAL_I2C_Master_Receive(&hi2c1, (uint16_t) MLX90393_Address, pRxData, 1, HAL_MAX_DELAY);

  pTxData[0] = 0xF0; // Выполняем команду Reset
  state = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) MLX90393_Address, pTxData, 1, HAL_MAX_DELAY);
  state = HAL_I2C_Master_Receive(&hi2c1, (uint16_t) MLX90393_Address, pRxData, 1, HAL_MAX_DELAY);

  // Запускаем непрерывный режим передав конфигурацию 0x1E
  pTxData[0] = 0x1E;
  state = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) MLX90393_Address, pTxData, 1, HAL_MAX_DELAY);
  state = HAL_I2C_Master_Receive(&hi2c1, (uint16_t) MLX90393_Address, pRxData, 1, HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Команда на чтение, для получения значений осей XYZ
  pTxData[0] = 0x4E;

  while (1) {
    if (flag_exti1) {
      flag_exti1 = 0;

      // Выполняем запрос на считывание данных
      state = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) MLX90393_Address, pTxData, 1, HAL_MAX_DELAY);
      if (state == HAL_OK) {
        state = HAL_I2C_Master_Receive_DMA(&hi2c1, (uint16_t) MLX90393_Address, pRxData, 7);
        if (state == HAL_OK) {
          // Ошибок нет переходим на следующую итерацию
          continue;
        }
      }
      // Если произошла ошибка, включаем ожидание новых данных
      // При помощи макроса очищаем бит EXTI_PR
      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
      // Очищаем бит NVIC_ICPRx
      NVIC_ClearPendingIRQ(EXTI1_IRQn);
      // Включаем внешнее прерывание
      HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    }

    if (isDataReady) {
      isDataReady = 0;
      // Проверяем количество пришедших данных, обрабатываем если равно 0x02 (всего байт 2 * 0x02 + 2) равно 6 байт
      if ((pRxData[0] & 0x03) == 0x02) {
        if (count != 0) {
          // Суммируем полученные значения 8 раз, чтобы позже вычислить среднее
          xSum += (int16_t) (pRxData[1] << 8 | pRxData[2]);
          ySum += (int16_t) (pRxData[3] << 8 | pRxData[4]);
          zSum += (int16_t) (pRxData[5] << 8 | pRxData[6]);
          count--;
        }

        if (!count) {
          count = 8;
          // Суммирование закончено, вычисляем среднее сдвигая вправо на 3
          xResult = xSum >> 3;
          yResult = ySum >> 3;
          zResult = zSum >> 3;
          // Обнуляем переменные для следующего суммирования
          xSum = 0;
          ySum = 0;
          zSum = 0;
        }

        // Только здесь разрешаем снова запустить внешнее прерывание иначе оно сработает раньше,
        // чем будут получены данные от предыдущего измерения
        // При помощи макроса очищаем бит EXTI_PR
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
        // Очищаем бит NVIC_ICPRx
        NVIC_ClearPendingIRQ(EXTI1_IRQn);
        // Включаем внешнее прерывание
        HAL_NVIC_EnableIRQ(EXTI1_IRQn);
      }
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
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }

  /** Enables the Clock Security System
   */
  HAL_RCC_EnableCSS();
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_PIN_1) {
    // Сразу же отключаем прерывания на данном пине
    HAL_NVIC_DisableIRQ(EXTI1_IRQn);
    flag_exti1 = 1;
  }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance == I2C1) {
    isDataReady = 1;
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
