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
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "crc32.h"
#include "my_config.h"
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

/* USER CODE BEGIN PV */
  // Буфер для измеренных значений с ADC
  uint16_t adcBuffer[ADC_BUFFER_SIZE];
  // Флаг готовности микросхемы
  bool isMemReady = false;
  // Буфер для работы с EEPROM (чтение и запись)
  uint8_t memBuffer[MEM_BUFFER_SIZE];
  // Статус зарядки, true - батарея находится на зарядке
  bool isСharged = false;
  // Количество циклов зарядки батареи записанные в EEPROM
  uint16_t chargeCycles;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
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
  // Данный код в задачу не выносим, т.к. он выполняется один раз при старте программы
  // Подаём питание на микросхему EEPROM, низкий уровень сигнала включает PNP транзистор
  HAL_GPIO_WritePin(EE_GPIO_Port, EE_Pin, GPIO_PIN_RESET);
  HAL_Delay(1000);
  // Проверям готовность, 5 попыток, таймаут 100 мс
  if (HAL_I2C_IsDeviceReady(&hi2c1, ADDRESS, 5, 100) == HAL_OK) {
    isMemReady = true;
    RCC->AHBENR |= RCC_AHBENR_CRCEN; // Включить тактирование CRC
  } else {
    // Если при инициализации произошла ошибка, перезапускаем микроконтроллер
    // Отключаем питание микросхемы EEPROM, переводим вывод в Z состояние
    HAL_GPIO_WritePin(EE_GPIO_Port, EE_Pin, GPIO_PIN_SET);
    HAL_Delay(1000);
    __set_FAULTMASK(1); // Запрещаем все маскируемые прерывания
    NVIC_SystemReset(); // Программный сброс
  }

  if (isMemReady) {
    // Считаем из EEPROM флаг статуса зарядки и количество циклов зарядки батареи
    if (HAL_I2C_Mem_Read(&hi2c1, ADDRESS, 0x00, I2C_MEMADD_SIZE_16BIT, memBuffer, MEM_BUFFER_SIZE, 100) == HAL_OK) {
      // Статус зарядки
      // Вычисляем контрольную сумму из полученных из EEPROM данных
      uint32_t dataCRC32 = computeCRC32((uint32_t) memBuffer[0]);
      // Преобразуем считанную из EEPROM массив контрольной суммы в двойное слово
      uint32_t saveCRC32 = (uint32_t) memBuffer[1]<<24 | memBuffer[2]<<16 | memBuffer[3]<<8 | memBuffer[4];
      if (dataCRC32 == saveCRC32) {
        // Если было сохранено 1, записываем true, если записан 0 - false
        isСharged = (memBuffer[0] == 1);
      } else {
        // В случае ошибок при записи в EEPROM разядку запрещаем
        isСharged = false;
      }

      // Количество циклов заряда батареи
      // Вычисляем контрольную сумму из полученных из EEPROM данных
      dataCRC32 = computeCRC32((uint32_t) memBuffer[5] << 8 | memBuffer[6]);
      // Преобразуем считанную из EEPROM массив контрольной суммы в двойное слово
      saveCRC32 = (uint32_t) memBuffer[7]<<24 | memBuffer[8]<<16 | memBuffer[9]<<8 | memBuffer[10];
      if (dataCRC32 == saveCRC32) {
        chargeCycles = (uint16_t) memBuffer[5]<<8 | memBuffer[6];
      } else {
        // В случае ошибок при записи в EEPROM количество циклов считаем равным 0
        chargeCycles = 0;
      }
    } else {
      isMemReady = false;
    }
  }

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adcBuffer, ADC_BUFFER_SIZE);
  HAL_TIM_Base_Start(&htim3);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
