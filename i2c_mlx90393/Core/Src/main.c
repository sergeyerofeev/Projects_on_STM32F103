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
#include "math.h"
#include "i2c_er.h"
#include "mlx90393.h"
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

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
HAL_StatusTypeDef state = HAL_OK; // Результат выполнения запроса

// Флаг готовности выполнить вычисления, 1 раз в 40 мс
volatile uint8_t isCalculation = 0;

// Данные успешно прочитаны
uint8_t isDataReady = 0;

// Команды для одиночного измерения и чтения данных
uint8_t comSM = 0x3E, comRM = 0x4E;

uint8_t pRxData[7] = { 0 };

// Вычисленный результат
int16_t resX;
int16_t resY;
int16_t resZ;

// Смещение которое будем вычитать из результата
int16_t xOffset;
int16_t yOffset;
int16_t zOffset;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
uint8_t MLX90393_Get_Data(void);
uint8_t MLX90393_Calibrate(void);
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
  __HAL_RCC_I2C1_CLK_ENABLE();
  HAL_Delay(100);
  __HAL_RCC_I2C1_FORCE_RESET();
  HAL_Delay(100);
  __HAL_RCC_I2C1_RELEASE_RESET();
  HAL_Delay(100);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  // MLX90393 включается от ножки PB5
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_Delay(500);

  // Проверяем готовность I2C и MLX90393
  state = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t) MLX90393_Address, 1, HAL_MAX_DELAY);
  if (state == HAL_BUSY) {
    // При данном флаге вызываем функцию переинициализации I2C1
    I2C_ClearBusyFlagErratum(&hi2c1, 1000);
  }

  // Производим сброс MLX90393
  state = init_MLX90393(&hi2c1, (uint16_t) MLX90393_Address);
  if (state != HAL_OK) {
    // Если при инициализации произошла ошибка, перезапускаем микроконтроллер и MLX90393
    __set_FAULTMASK(1); // Запрещаем все маскируемые прерывания
    NVIC_SystemReset(); // Программный сброс
  }

  // Калибруем MLX90393
  if (MLX90393_Calibrate()) {
    // Калибровка завершилась с ошибкой, перезапускаем микроконтроллер и MLX90393
    __set_FAULTMASK(1); // Запрещаем все маскируемые прерывания
    NVIC_SystemReset(); // Программный сброс
  }

  // Запускаем опрос MLX90393 каждые 25 мс
  __HAL_TIM_CLEAR_FLAG(&htim4, TIM_SR_UIF); // Очищаем флаг
  HAL_TIM_Base_Start_IT(&htim4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    if (isCalculation) {
      isCalculation = 0;
      // Получаем данные и отправляем запрос на получение данных каждые 40 мс
      // В случае ошибки переходим на следующую итерацию цикла
      if (MLX90393_Get_Data()) continue;
      //MLX90393_Get_Data();
    }

    if (isDataReady) {
      isDataReady = 0;
      // Проверяем по байту статуса, количество пришедших данных, обрабатываем если равно 0x02
      // (всего байт 2 * 0x02 + 2) равно 6 байт
      if ((pRxData[0] & 0x03) == 0x02) {

        resX = (((int16_t) pRxData[1] << 8 | pRxData[2]) - xOffset);
        resY = (((int16_t) pRxData[3] << 8 | pRxData[4]) - yOffset);
        resZ = (((int16_t) pRxData[5] << 8 | pRxData[6]) - zOffset);
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
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
static void MX_I2C1_Init(void)
{

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 480-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/*
  // Код для сброса линии USB
  // Инициализируем пин DP как выход
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); // Прижимаем DP к "земле"
  for (uint16_t i = 0; i < 1000; i++) {
  }; // Немного ждём

  // Переинициализируем пин для работы с USB
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  for (uint16_t i = 0; i < 1000; i++) {
  }; // Немного ждём
*/
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM4) {
    isCalculation = 1;
  }
}

/**
 * В случае ошибки возвращаем 1
 */
uint8_t MLX90393_Get_Data(void) {
  // Выполняем запрос на считывание данных
  state = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) MLX90393_Address, (uint8_t*) &comRM, 1, HAL_MAX_DELAY);
  if (state != HAL_OK) return 1;
  state = HAL_I2C_Master_Receive(&hi2c1, (uint16_t) MLX90393_Address, pRxData, 7, HAL_MAX_DELAY);
  if (state != HAL_OK) return 1;

  // Запускаем новое измерение
  state = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) MLX90393_Address, (uint8_t*) &comSM, 1, HAL_MAX_DELAY);
  if (state != HAL_OK) return 1;
  state = HAL_I2C_Master_Receive(&hi2c1, (uint16_t) MLX90393_Address, pRxData, 1, HAL_MAX_DELAY);
  if (state != HAL_OK) return 1;

  // Ошибок нет, устанавливаем флаг о готовности данных
  isDataReady = 1;

  return 0;
}

/**
 * В случае ошибки возвращаем 1
 */
uint8_t MLX90393_Calibrate(void) {
  int32_t xCal = 0;
  int32_t yCal = 0;
  int32_t zCal = 0;
  // Считаем сколько реально было выполнено итераций по получению данных
  uint16_t count = 0;

  // Запускаем новое измерение
  state = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) MLX90393_Address, (uint8_t*) &comSM, 1, HAL_MAX_DELAY);
  if (state != HAL_OK) return 1;
  state = HAL_I2C_Master_Receive(&hi2c1, (uint16_t) MLX90393_Address, pRxData, 1, HAL_MAX_DELAY);
  if (state != HAL_OK) return 1;

  // Калибровку выполняем за 2,5 секунды, 100 итераций * 25 мс
  for (uint16_t i = 0; i < 100; i++) {
    HAL_Delay(25);
    // Если произошла ошибка при получении данных, переходим на следующую итерацию
    if (MLX90393_Get_Data()) continue;

    if (isDataReady) {
      isDataReady = 0;
      count++;
      xCal += (int16_t) pRxData[1] << 8 | pRxData[2];
      yCal += (int16_t) pRxData[3] << 8 | pRxData[4];
      zCal += (int16_t) pRxData[5] << 8 | pRxData[6];
    }
  }
  xOffset = (int16_t) (xCal / count);
  yOffset = (int16_t) (yCal / count);
  zOffset = (int16_t) (zCal / count);

  // Функция завершилась без ошибок
  return 0;
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
