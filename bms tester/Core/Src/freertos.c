/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "semphr.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
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
/* USER CODE BEGIN Variables */
// Дескрипторы задач
TaskHandle_t countShowHandle;
TaskHandle_t dataShowHandle;
// Все ошибки складываем в одну переменную
bool isError = false;
// Позиция в структуре адресов запросов
size_t addrCount = 0;

extern UART_HandleTypeDef huart1;
// Массив структур
extern BmsData bmsData[];
// Количество элементов массива bmsData[]
extern const size_t bmsDataCount;
// Размер буфера для приёма данных указываем максимального размера
extern uint8_t rxData[RX_BUFSIZE];

// Счётчик для фонового вывода чисел на экран
uint8_t countFlag = 0;
char strCount[3] = { 0, };
// Первая позиция вывода символа на экран SSD1306
uint8_t x = 0;
uint8_t y = 0;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void vCountShow(void *argument);
void vDataShow(void *argument);
void vTaskSystemReset(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN PREPOSTSLEEP */
__weak void PreSleepProcessing(uint32_t *ulExpectedIdleTime) {
  /* place for user code */
}

__weak void PostSleepProcessing(uint32_t *ulExpectedIdleTime) {
  /* place for user code */
}
/* USER CODE END PREPOSTSLEEP */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if (xTaskCreate(vCountShow, NULL, 128, NULL, osPriorityNormal, &countShowHandle) == pdFAIL) {
    isError = true;
  }
  if (xTaskCreate(vDataShow, NULL, 128, NULL, osPriorityNormal, &dataShowHandle) == pdFAIL) {
    isError = true;
  }
  if (xTaskCreate(vTaskSystemReset, "taskSystemReset", 128, NULL, osPriorityHigh, NULL) == pdFAIL) {
    isError = true;
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;) {
    vTaskDelete(NULL);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// С периодичностью в 1 секунду показываем на экране значение счётчика
void vCountShow(void *argument) {
  for (;;) {
    // Если данные от BMS не поступили выводим в центре экрана значение счётчика
    strCount[0] = "0123456789ABCDEF"[countFlag >> 4 & 0x0F];
    strCount[1] = "0123456789ABCDEF"[countFlag & 0x0F];
    // Размещаем строку по центру экрана
    x = (SSD1306_WIDTH - strlen(strCount) * Font_16x24.width) / 2;
    y = SSD1306_HEIGHT / 2 - Font_16x24.height / 2;
    ssd1306_SetCursor(x, y);
    ssd1306_Fill(Black);
    ssd1306_WriteString(strCount, Font_16x24, White);
    ssd1306_UpdateScreen();
    countFlag++;

    // Прошёл интервал в 1 секунду, можем сделать запрос по UART
    HAL_UART_Transmit_IT(&huart1, bmsData[addrCount].dataTx, TX_BUFSIZE);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Как только данные получены, выводим их на экран
void vDataShow(void *argument) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
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
        uint32_t code = (rxData[7] << 24) | (rxData[8] << 16) | (rxData[9] << 8) | rxData[10];
        strcpy(bmsData[addrCount].strRes, code_to_vendor(code));
        break;
    }
    addrCount++;
    if (addrCount < bmsDataCount) {
      // Снова запускаем чтение данных
      HAL_UART_Receive_IT(&huart1, rxData, bmsData[addrCount].rxBufSize);
      HAL_UART_Transmit_IT(&huart1, bmsData[addrCount].dataTx, TX_BUFSIZE);
    } else {
      // Удаляем задачу вывода значения счётчика
      vTaskDelete(countShowHandle);
      // Выводим на экран все данные полученные от bms
      ssd1306_Fill(Black);

      for (uint8_t i = 0, y = 2; i < 5; i++, y += 13) {
        x = (SSD1306_WIDTH - strlen(bmsData[i].strRes) * Font_7x10.width) / 2;
        ssd1306_SetCursor(x, y);
        ssd1306_WriteString(bmsData[i].strRes, Font_7x10, White);
      }
      ssd1306_UpdateScreen();

      // Отключаем ВСЕ прерывания UART
      __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);  // Приемный буфер не пуст
      __HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);   // Буфер передачи пуст
      __HAL_UART_DISABLE_IT(&huart1, UART_IT_TC);    // Передача завершена
      __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);  // Линия в состоянии IDLE
      __HAL_UART_DISABLE_IT(&huart1, UART_IT_PE);    // Ошибка четности
      __HAL_UART_DISABLE_IT(&huart1, UART_IT_ERR);   // Ошибка (FE, OE, NE)
      __HAL_UART_DISABLE_IT(&huart1, UART_IT_CTS);   // CTS изменение
      __HAL_UART_DISABLE_IT(&huart1, UART_IT_LBD);   // Break обнаружение
    }
  }
}

// Программный сброс, в случае ошибок при инициализации задач
void vTaskSystemReset(void *argument) {
  for (;;) {
    if (isError) {
      // Запрещаем все маскируемые прерывания
      __disable_irq();
      __set_FAULTMASK(1);
      // Короткая задержка для стабилизации
      for (volatile int i = 0; i < 1000; i++)
        __NOP();
      // Выполняем сброс
      NVIC_SystemReset();
    } else {
      // Если ошибок нет, удаляем текущую задачу
      vTaskDelete(NULL);
    }
  }
}
/* USER CODE END Application */

