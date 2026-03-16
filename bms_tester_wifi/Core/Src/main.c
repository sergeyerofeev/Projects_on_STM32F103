/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "usart_ring.h"
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
// Переменные для работы с UART
extern UART_HandleTypeDef huart1;
uint8_t rxData[RX_BUFSIZE];

// Флаги и счётчики
volatile bool dataReceived = false;     // Флаг получения данных по UART
bool isConnected = false;               // Флаг наличия соединения
uint8_t timeoutCounter = 0;              // Счётчик таймаутов
size_t addrCount = 0;                    // Текущий индекс в массиве запросов

// Массив структур BMS
extern BmsData bmsData[];
extern const size_t bmsDataCount;

// Переменные для отображения счётчика
uint8_t countNum = 0;
char strCount[3] = { 0 };
uint8_t x = 0;
uint8_t y = 0;

// Переменные для временных интервалов
uint32_t dataTick = 0;
uint32_t countTick = 0;
const uint32_t intervalMs = 1000;  // Интервал 1 секунда
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void _processReceivedData(void);
void _displayAllData(void);
void _displayCounter(void);
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // Инициализация дисплея
  ssd1306_Init();
  // Запускаем UART в режиме приёма
  __HAL_UART_ENABLE_IT(&MYUART, UART_IT_RXNE);
  //HAL_UART_Receive_IT(&huart1, rxData, bmsData[0].rxBufSize);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    uint32_t currentTick = HAL_GetTick();

    // Проверяем, прошла ли 1 секунда
    if (currentTick - dataTick >= intervalMs) {
      dataTick = currentTick;
      // Очищаем кольцевой буфер
      clear_uart_buff();
      // Каждую секунду запускаем опрос BMS
      addrCount = 0;
      HAL_UART_Transmit(&huart1, bmsData[addrCount].dataTx, TX_BUFSIZE, 10);
      isConnected = false;
    }

    // Получаем данные от BMS
    if (uart_available()) {
      uint8_t i = 0;
      if (!isConnected) {
        isConnected = true;
      }
      // Выполним небольшую задержку, чтобы остатки данных загрузились
      HAL_Delay(5);

      while (uart_available()) {
        // uart_read() функция чтения байта из usart_ring.c
        rxData[i++] = uart_read();
      }
      // Обрабатываем полученные данные
      _processReceivedData();
      addrCount++;
      // Если получили не все данные, запрашиваем следующие
      if (addrCount < bmsDataCount) {
        HAL_UART_Transmit(&huart1, bmsData[addrCount].dataTx, TX_BUFSIZE, 10);
      } else {
        // Все данные получены
        _displayAllData();
        // Сбрасываем счётчик для следующего цикла
        addrCount = 0;
      }
    }

    // Если нет соединения, показываем счётчик 1 раз в секунду
    if (currentTick - countTick >= intervalMs + 300) {
      countTick = currentTick - 300;
      if (!isConnected) {
        _displayCounter();
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }

  /** Enables the Clock Security System
   */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
// Функция отображения счётчика
void _displayCounter(void) {
  strCount[0] = "0123456789ABCDEF"[countNum >> 4 & 0x0F];
  strCount[1] = "0123456789ABCDEF"[countNum & 0x0F];

  x = (SSD1306_WIDTH - strlen(strCount) * Font_16x24.width) / 2;
  y = SSD1306_HEIGHT / 2 - Font_16x24.height / 2;

  ssd1306_SetCursor(x, y);
  ssd1306_Fill(Black);
  ssd1306_WriteString(strCount, Font_16x24, White);
  ssd1306_UpdateScreen();

  countNum++;
}

// Функция отображения всех данных с проверкой изменений
void _displayAllData(void) {
  bool needUpdate = false;

  // Проверяем, были ли изменения
  for (uint8_t i = 0; i < bmsDataCount; i++) {
    if (strcmp(bmsData[i].strPrev, bmsData[i].strResult) != 0) {
      needUpdate = true;
      break;
    }
  }

  if (!needUpdate) {
    return;
  }

  // Очищаем экран
  ssd1306_Fill(Black);

  // Выводим все строки
  for (uint8_t i = 0, y = 2; i < 5; i++, y += 13) {
    x = (SSD1306_WIDTH - strlen(bmsData[i].strResult) * Font_7x10.width) / 2;
    ssd1306_SetCursor(x, y);
    ssd1306_WriteString(bmsData[i].strResult, Font_7x10, White);

    // Сохраняем новое значение
    if (strcmp(bmsData[i].strPrev, bmsData[i].strResult) != 0) {
      strcpy(bmsData[i].strPrev, bmsData[i].strResult);
    }
  }

  ssd1306_UpdateScreen();
}

// Функция обработки полученных данных
void _processReceivedData(void) {
  switch (bmsData[addrCount].addr) {
    case 0x17:
      // Версия прошивки BMS
      uint16_t version = (rxData[8] << 8) | rxData[7];
      if (version == 0) {
        strcpy(bmsData[addrCount].strResult, "v 0.0.0.0");
      } else {
        bmsData[addrCount].strResult[0] = 'v';
        bmsData[addrCount].strResult[1] = ' ';

        uint8_t j = 2;
        bool first = true;

        for (int8_t i = 12; i >= 0; i -= 4) {
          if ((version >> i == 0) && first) {
            continue;
          } else {
            first = false;
          }
          bmsData[addrCount].strResult[j++] = "0123456789ABCDEF"[(version) >> i & 0x0F];
          bmsData[addrCount].strResult[j++] = '.';
        }
        bmsData[addrCount].strResult[--j] = '\0';
      }
      break;

    case 0x32:
      // Текущее значение остаточной мощности
      uint8_t power = rxData[7];
      snprintf(bmsData[addrCount].strResult, STR_SIZE, "%u %%", power);
      break;

    case 0x31:
      // Текущая остаточная емкость
      uint16_t capacity = (rxData[8] << 8) | rxData[7];
      snprintf(bmsData[addrCount].strResult, STR_SIZE, "%u mAh", capacity);
      break;

    case 0x34:
      // Текущее напряжение
      int16_t voltage = (rxData[8] << 8) | rxData[7];
      snprintf(bmsData[addrCount].strResult, STR_SIZE, "%d,%d V", voltage / 100, voltage % 100);
      break;

    case 0x60:
      // Вендор
      uint32_t code = (rxData[7] << 24) | (rxData[8] << 16) | (rxData[9] << 8) | rxData[10];
      strcpy(bmsData[addrCount].strResult, code_to_vendor(code));
      break;
  }
}

// Обработчик прерывания по UART
/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 if (huart->Instance == USART1) {
 dataReceived = true;
 }
 }*/

// Обработчик ошибок UART
/*void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
 if (huart->Instance == USART1) {
 // При ошибке перезапускаем приём
 HAL_UART_Receive_IT(&huart1, rxData, bmsData[addrCount].rxBufSize);
 }
 }*/
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
