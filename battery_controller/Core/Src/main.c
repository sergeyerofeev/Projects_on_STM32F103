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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include "crc32.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Два канала опрашиваем по одному разу каждый
#define ADC_BUFFER_SIZE 2
// Величина 4095 * 1.2 = 4914.0 используемая для получение v_ref
#define INTERNAL_REF 4914.0

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

/* USER CODE BEGIN PV */
// Если преобразование ADC1 завершено, isEndADC получает значение true
static volatile bool isEndADC = false;

// Ожидаем окончания процесса записи данных в EEPROM
static volatile uint32_t time_irq;
static volatile bool isTxCompleted = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  // Буфер для измеренных значений с ADC
  uint16_t adcBuffer[ADC_BUFFER_SIZE];
  // Значение с канала Vrefint Channel, напряжение питания микроконтроллера
  double v_ref;
  // Значение с канала IN0, напряжение Li-Ion батареи
  double v_bat;
  // Флаг готовности микросхемы
  bool isMemReady = false;
  // Буфер для работы с EEPROM (чтение и запись)
  uint8_t memBuffer[MEM_BUFFER_SIZE];
  // Статус зарядки, true - батарея находится на зарядке
  bool isСharged = false;
  // Количество циклов зарядки батареи записанные в EEPROM
  uint16_t chargeCycles;
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

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (isEndADC) {
      v_ref = INTERNAL_REF / adcBuffer[0];
      v_bat = adcBuffer[1] * v_ref / 4095 + V_DIODE;
      // Проверяем наличие внешнего питания
      if (HAL_GPIO_ReadPin(IN_GPIO_Port, IN_Pin) == GPIO_PIN_SET) {
        if (isСharged) {
          // Ранее (до перезагрузки) зарядка уже была запущена, поэтому включаем
          HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET);

          if (v_bat > MAX_CHARGE_V) {
            // Напряжение на батарее превышает верхний предел, завершаем зарядку
            HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);
            isСharged = false;
            chargeCycles++;
            if (isMemReady && !isTxCompleted) {
              // Сохраняем статус зарядки в память и на 1 увеличиваем количество циклов зарядки
              // Заполняем буфер для передачи в EEPROM
              memBuffer[0] = (uint8_t) isСharged;
              decompose32into8(computeCRC32((uint32_t) isСharged), memBuffer, 1);

              memBuffer[5] = (chargeCycles >> 8) & 0xFF;
              memBuffer[6] = chargeCycles & 0xFF;
              decompose32into8(computeCRC32((uint32_t) chargeCycles), memBuffer, 7);

              // Запись в микросхему EEPROM не произодится, можем отправить данные
              if (HAL_I2C_Mem_Write_IT(&hi2c1, ADDRESS, 0x00, I2C_MEMADD_SIZE_16BIT, memBuffer, MEM_BUFFER_SIZE) != HAL_OK) {
                isMemReady = false;
              }
            }
          }
        } else {
          // Ранее (до перезагрузки) зарядка не проводилась, поэтому выключаем
          HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);

          if (v_bat < MIN_CHARGE_V) {
            // Зарядка выключена, но напряжение на батарее стало ниже нижнего предела
            // Включаем зарядку
            HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET);
            isСharged = true;
            // Количество циклов зарядки остаётся прежним
            if (isMemReady && !isTxCompleted) {
              // Сохраняем статус зарядки в память, количество циклов зарядки оставляем без изменения
              // Заполняем буфер для передачи в EEPROM
              memBuffer[0] = (uint8_t) isСharged;
              decompose32into8(computeCRC32((uint32_t) isСharged), memBuffer, 1);

              if (HAL_I2C_Mem_Write_IT(&hi2c1, ADDRESS, 0x00, I2C_MEMADD_SIZE_16BIT, memBuffer, MEM_BUFFER_SIZE) != HAL_OK) {
                isMemReady = false;
              }
            }
          }
        }
      }
    }
    // Отсчитываем время необходимое микросхеме EEPROM для записи данных, 10 мс
    if(isMemReady && isTxCompleted && (HAL_GetTick() - time_irq) > 10) {
          isTxCompleted = false;
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

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    isEndADC = true;
  }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance == I2C1) {
    isTxCompleted = true;
    time_irq = HAL_GetTick();
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
