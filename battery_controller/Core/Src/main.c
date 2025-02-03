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
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Два канала опрашиваем по одному разу каждый
#define ADC_BUFFER_SIZE 2
// Падение напряжение на последовательно подключенном диоде
#define V_DIODE 0.6
// Минимальное напряжение на батарее, при котором запускаем зарядку
#define MIN_CHARGE_V 3.4
// Максимальное напряжение на батарее, при котором зарядку прекращаем
#define MAX_CHARGE_V 3.8
// Адрес микросхемы EEPROM на линии I2C, 7 бит сдвинутых влево на единицу
#define ADDRESS (0x50 << 1)
// Размер массива для работы с EEPROM
#define MEM_BUFFER_SIZE 11
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
// Если преобразование ADC1 завершено, adc_end получает значение true
volatile bool adc_end = false;
volatile uint16_t adcBuffer[ADC_BUFFER_SIZE];
// Значение с канала Vrefint Channel, напряжение питания микроконтроллера
double v_ref;
// Величина 4095 * 1.2 = 4914 используемая для получение v_ref
const double internal_ref = 4914;
// Значение с канала IN0, напряжение Li-Ion батареи
double v_bat;
// Флаг готовности микросхемы
bool isMemReady = false;
// Буфер для работы с EEPROM (чтение и запись)
uint8_t memBuffer[MEM_BUFFER_SIZE];
// Идёт ли зарядка
bool isСharged = false;
// Количество циклов зарядки батареи записанные в EEPROM
uint16_t chargeCycles;
// Вычисленное значение CRC32
uint32_t dataCRC;
// union для преобразования uint32_t в массив из четырёх uint8_t
union CRC_Converter crcCharging;
union CRC_Converter crcCycles;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_I2C_IsDeviceReady(&hi2c1, ADDRESS, 5, 100) == HAL_OK) {
    isMemReady = true;
    RCC->AHBENR |= RCC_AHBENR_CRCEN; // Включить тактирование CRC
  }
  if (isMemReady) {
    // Считаем из EEPROM флаг статуса зарядки и количество циклов зарядки батареи
    if (HAL_I2C_Mem_Read(&hi2c1, ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, memBuffer, MEM_BUFFER_SIZE, 100) == HAL_OK) {
      // Получаем записанные данные и вычисляем контрольную сумму
      CRC->CR |= CRC_CR_RESET; // Сброс CRC (очистка аккумулятора)
      CRC->DR = (uint32_t) memBuffer[0]; // Передаём данные для вычисления CRC32
      dataCRC = CRC->DR; // Получаем вычисленное значение CRC32
      // Получаем записанную контрольную сумму статуса зарядки
      crcCharging = (union CRC_Converter) { .bytes = {memBuffer[4], memBuffer[3], memBuffer[2], memBuffer[1]} };
      if (crcCharging.crc32 == dataCRC) {
        // Если было сохранено 1, записываем true, если записан 0 - false
        isСharged = (memBuffer[0] == 1);
      } else {
        // В случае ошибок при записи в EEPROM разядку запрещаем
        isСharged = false;
      }
      // Количество циклов заряда батареи
      CRC->CR |= CRC_CR_RESET; // Сброс CRC (очистка аккумулятора)
      CRC->DR = (uint32_t) memBuffer[5] << 8 | memBuffer[6] ; // Передаём данные для вычисления CRC32
      dataCRC = CRC->DR; // Получаем вычисленное значение CRC32
      // Получаем записанную контрольную сумму количества циклов зарядки
      crcCycles = (union CRC_Converter) { .bytes = {memBuffer[10], memBuffer[9], memBuffer[8], memBuffer[7]} };
      if (crcCycles.crc32 == dataCRC) {
        chargeCycles = (uint16_t) memBuffer[5] << 8 | memBuffer[6];
      } else {
        // В случае ошибок при записи в EEPROM количество циклов считаем равным 0
        chargeCycles = 0;
      }
    }

  }
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adcBuffer, ADC_BUFFER_SIZE);
  HAL_TIM_Base_Start(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (adc_end) {
      v_ref = internal_ref / adcBuffer[0];
      v_bat = adcBuffer[1] * v_ref / 4095 + V_DIODE;
      // Проверяем наличие внешнего питания
      if (HAL_GPIO_ReadPin(IN_GPIO_Port, IN_Pin) == GPIO_PIN_SET) {
        if (isСharged) {
          // Ранее (до перезагрузки) зарядка уже была запущена, поэтому включаем
          HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);

          if (v_bat > MAX_CHARGE_V) {
            // Напряжение на батарее превышает верхний предел, завершаем зарядку
            HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
            isСharged = false;
            chargeCycles++;
            if (isMemReady) {
              // Сохраняем статус зарядки в память и на 1 увеличиваем количество циклов зарядки
              CRC->CR |= CRC_CR_RESET; // Сброс CRC (очистка аккумулятора)
              CRC->DR = (uint32_t) isСharged; // Передаём данные для вычисления CRC32
              crcCharging.crc32 = CRC->DR; // Получаем вычисленное значение CRC32
              CRC->CR |= CRC_CR_RESET; // Сброс CRC (очистка аккумулятора)
              CRC->DR = (uint32_t) chargeCycles; // Передаём данные для вычисления CRC32
              crcCycles.crc32 = CRC->DR; // Получаем вычисленное значение CRC32
              // Заполняем буфер для передачи в EEPROM
              memBuffer[0] = (uint8_t) isСharged;
              memBuffer[1] = crcCharging.bytes[3];
              memBuffer[2] = crcCharging.bytes[2];
              memBuffer[3] = crcCharging.bytes[1];
              memBuffer[4] = crcCharging.bytes[0];
              memBuffer[5] = (uint8_t) (chargeCycles >> 8);
              memBuffer[6] = (uint8_t) chargeCycles;
              memBuffer[7] = crcCycles.bytes[3];
              memBuffer[8] = crcCycles.bytes[2];
              memBuffer[9] = crcCycles.bytes[1];
              memBuffer[10] = crcCycles.bytes[0];

              if (HAL_I2C_Mem_Write(&hi2c1, ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, memBuffer, MEM_BUFFER_SIZE, 100) != HAL_OK) {
                isMemReady = false;
              }
            }
          }
        } else {
          // Ранее (до перезагрузки) зарядка не проводилась, поэтому выключаем
          HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);

          if (v_bat < MIN_CHARGE_V) {
            // Зарядка выключена, но напряжение на батарее стало ниже нижнего предела
            // Включаем зарядку
            HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
            isСharged = true;
            // Количество циклов зарядки остаётся прежним
            if (isMemReady) {
              // Сохраняем статус зарядки в память и количество циклов зарядки
              CRC->CR |= CRC_CR_RESET; // Сброс CRC (очистка аккумулятора)
              CRC->DR = (uint32_t) isСharged; // Передаём данные для вычисления CRC32
              crcCharging.crc32 = CRC->DR; // Получаем вычисленное значение CRC32
              CRC->CR |= CRC_CR_RESET; // Сброс CRC (очистка аккумулятора)
              CRC->DR = (uint32_t) chargeCycles; // Передаём данные для вычисления CRC32
              crcCycles.crc32 = CRC->DR; // Получаем вычисленное значение CRC32
              // Заполняем буфер для передачи в EEPROM
              memBuffer[0] = (uint8_t) isСharged;
              memBuffer[1] = crcCharging.bytes[3];
              memBuffer[2] = crcCharging.bytes[2];
              memBuffer[3] = crcCharging.bytes[1];
              memBuffer[4] = crcCharging.bytes[0];
              memBuffer[5] = (uint8_t) (chargeCycles >> 8);
              memBuffer[6] = (uint8_t) chargeCycles;
              memBuffer[7] = crcCycles.bytes[3];
              memBuffer[8] = crcCycles.bytes[2];
              memBuffer[9] = crcCycles.bytes[1];
              memBuffer[10] = crcCycles.bytes[0];

              if (HAL_I2C_Mem_Write(&hi2c1, ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, memBuffer, MEM_BUFFER_SIZE, 100) != HAL_OK) {
                isMemReady = false;
              }
            }
          }
        }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 60000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IN_Pin */
  GPIO_InitStruct.Pin = IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_Pin */
  GPIO_InitStruct.Pin = EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    adc_end = true;
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
