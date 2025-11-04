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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "addr_data.h"
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
uint8_t addrCount = 0;    // Позиция в структуре адресов

bool isTransmit = false;
bool isReceive = false;

// Все данные с BMS получены
bool isComplite = false;

uint8_t countFlag = 0;
char strCount[3] = { 0, };
bool flag21 = false;

bool isTimPeriod = false;
// Размер буфера для приёма данных указываем максимального размера
uint8_t rxData[13] = { 0, };

// Первая позиция вывода символа на экран SSD1306
uint8_t x = 0;
uint8_t y = 0;
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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  ssd1306_Fill(Black);

  __HAL_TIM_CLEAR_FLAG(&htim4, TIM_SR_UIF); // очищаем флаг
  HAL_TIM_Base_Start_IT(&htim4);

  //HAL_UART_Receive_IT(&huart1, rxData, RX_BUFSIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    if (isTimPeriod) {
      isTimPeriod = false;
      // Если данные от BMS не поступили выводим в центре экрана значение счётчика
      strCount[0] = "0123456789ABCDEF"[countFlag >> 4 & 0x0F];
      strCount[1] = "0123456789ABCDEF"[countFlag & 0x0F];
      // Размещаем строку по центру экрана
      x = (SSD1306_WIDTH - strlen(strCount) * Font_16x26.width) / 2;
      y = SSD1306_HEIGHT / 2 - Font_16x26.height / 2;
      ssd1306_SetCursor(x, y);
      ssd1306_Fill(Black);
      ssd1306_WriteString(strCount, Font_16x26, White);
      ssd1306_UpdateScreen();
    }

    if (isReceive) {
      isReceive = false;
      // Получили данные от BMS
      switch (bmsData[addrCount].addr) {
        case 0x17:
          // Версия прошивки BMS
          uint16_t version = (rxData[8] << 8) | rxData[7];
          if (version == 0) {
            // Если число равно 0, выводим нулевую версию
            strcpy(bmsData[addrCount].strRes, "v 0.0.0.0");
          } else {
            bmsData[addrCount].strRes[0] = 'v';
            bmsData[addrCount].strRes[1] = ' ';

            uint8_t j = 2; // Начальный индекс, с которого записываем символы в строковый массив
            bool first = true;

            for (int8_t i = 12; i >= 0; i -= 4) {
              if ((version >> i == 0) && first) {
                // Последовательно отбрасываем первые нули
                continue;
              } else {
                first = false;
              }
              // Преобразуем шестнадцатиричное значение в символ
              bmsData[addrCount].strRes[j++] = "0123456789ABCDEF"[(version) >> i & 0x0F];
              bmsData[addrCount].strRes[j++] = '.';
            }
            // Вместо последней точки ставим символ конца строки
            bmsData[addrCount].strRes[--j] = '\0';
          }
          break;

        case 0x32:
          // Текущее значение остаточной мощности, 0-100%
          uint8_t power = rxData[7];
          snprintf(bmsData[addrCount].strRes, STR_SIZE, "%u %%", power);
          break;
        case 0x31:
          // Текущая остаточная емкость, в mAh
          uint16_t capacity = (rxData[8] << 8) | rxData[7];
          snprintf(bmsData[addrCount].strRes, STR_SIZE, "%u mAh", capacity);
          break;
        case 0x34:
          // Текущее напряжение,  в V
          int16_t voltage = (rxData[8] << 8) | rxData[7];
          snprintf(bmsData[addrCount].strRes, STR_SIZE, "%d,%d V", voltage / 100, voltage % 100);
          break;
        case 0x60:
          // Вендор зашитый в BMS
          uint32_t vendor = (rxData[7] << 24) | (rxData[8] << 16) | (rxData[9] << 8) | rxData[10];
          snprintf(bmsData[addrCount].strRes, STR_SIZE, "%lX", vendor);
          break;
      }
      addrCount++;
      if (addrCount < 5) {
        // Снова запускаем чтение данных
        HAL_UART_Receive_IT(&huart1, rxData, bmsData[addrCount].rxBufSize);
        HAL_UART_Transmit_IT(&huart1, bmsData[addrCount].dataTx, TX_BUFSIZE);
      } else {
        HAL_TIM_Base_Stop_IT(&htim4); // Останавливаем таймер
        isComplite = true;
      }
    }

    if (isComplite) {
      isComplite = false;
      ssd1306_Fill(Black);
      // Выводим данные на экран
      for (uint8_t i = 0, y = 2; i < 5; i++, y += 13) {
        x = (SSD1306_WIDTH - strlen(bmsData[i].strRes) * Font_7x10.width) / 2;
        ssd1306_SetCursor(x, y);
        ssd1306_WriteString(bmsData[i].strRes, Font_7x10, White);
      }
      ssd1306_UpdateScreen();
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
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM4) {
    // Прошёл интервал в 1 секунду, можем сделать запрос по UART
    HAL_UART_Receive_IT(&huart1, rxData, bmsData[addrCount].rxBufSize);
    HAL_UART_Transmit_IT(&huart1, bmsData[addrCount].dataTx, TX_BUFSIZE);
    countFlag++;
    isTimPeriod = true;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    isReceive = true;
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
