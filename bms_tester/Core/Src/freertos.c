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
//TaskHandle_t dataShowHandle;
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

// Флаг наличия соединения
bool isConnected = false;
// Счётчик времени ожидания ответа
uint8_t timeoutCounter = 0;
// Счётчик для фонового вывода чисел на экран
uint8_t countNum = 0;
bool countFlag = false;
char strCount[3] = { 0, };
// Первая позиция вывода символа на экран SSD1306
uint8_t x = 0;
uint8_t y = 0;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void vCountShow(void *argument);
//void vDataShow(void *argument);
void vTaskSystemReset(void *argument);
// Прототип функции обработки полученных данных
void _processReceivedData(void);
// Прототип функции отображения всех данных
void _displayAllData(void);
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
  /*  if (xTaskCreate(vDataShow, NULL, 128, NULL, osPriorityNormal, &dataShowHandle) == pdFAIL) {
   isError = true;
   }*/
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
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;) {
    vTaskDelete(NULL);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
// С периодичностью в 1 секунду показываем на экране значение счётчика или данные BMS
void vCountShow(void *argument) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    // Всегда сначала показываем счётчик (по умолчанию)
    strCount[0] = "0123456789ABCDEF"[countNum >> 4 & 0x0F];
    strCount[1] = "0123456789ABCDEF"[countNum & 0x0F];

    // Если нет соединения, показываем счётчик
    if (!isConnected) {
      x = (SSD1306_WIDTH - strlen(strCount) * Font_16x24.width) / 2;
      y = SSD1306_HEIGHT / 2 - Font_16x24.height / 2;
      ssd1306_SetCursor(x, y);
      ssd1306_Fill(Black);
      ssd1306_WriteString(strCount, Font_16x24, White);
      ssd1306_UpdateScreen();
      countNum++;
      if (!countFlag) {
        countFlag = true; // Началась итерация цифр
      }
    }

    addrCount = 0;
    // Начинаем цикл опроса всех данных
    HAL_UART_Receive_IT(&huart1, rxData, bmsData[addrCount].rxBufSize);
    HAL_UART_Transmit(&huart1, bmsData[addrCount].dataTx, TX_BUFSIZE, 10);

    // Ждём ответа с таймаутом
    timeoutCounter = 0;
    bool dataReceived = false;

    while (addrCount < bmsDataCount && timeoutCounter < 5) {
      // Ждём уведомление от прерывания с таймаутом
      if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200)) == pdTRUE) {
        // Данные получены успешно - соединение есть!
        dataReceived = true;
        isConnected = true;

        _processReceivedData();
        addrCount++;
        timeoutCounter = 0;

        if (addrCount < bmsDataCount) {
          // Отправляем следующий запрос
          HAL_UART_Receive_IT(&huart1, rxData, bmsData[addrCount].rxBufSize);
          HAL_UART_Transmit(&huart1, bmsData[addrCount].dataTx, TX_BUFSIZE, 10);
        }
      } else {
        // Таймаут - данных нет
        timeoutCounter++;
      }
    }

    // Анализируем результат опроса
    if (dataReceived) {
      if (addrCount == bmsDataCount) {
        // Все данные успешно получены - показываем их на экране
        _displayAllData();

        // ВАЖНО: Сбрасываем счётчик для следующего цикла
        addrCount = 0;

        // Соединение есть и данные получены
        isConnected = true;
      } else {
        // Получены не все данные - соединение нестабильное
        isConnected = false;
      }
    } else {
      // Нет ответа от BMS - соединения нет
      isConnected = false;
    }

    // Ждём ровно 1 секунду с учётом времени обработки
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1000));
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

// Функция отображения всех данных
void _displayAllData(void) {
  bool needUpdate = false;
  // Если перед этим выводилась итерация, пропускаем проверку уникальности данных
  if (!countFlag) {
    // Сначала проверяем, были ли изменения
    for (uint8_t i = 0; i < bmsDataCount; i++) {
      if (strcmp(bmsData[i].strPrev, bmsData[i].strResult) != 0) {
        needUpdate = true;
        break;
      }
    }
    // Если изменений нет, выходим
    if (!needUpdate) {
      return;
    }
    countFlag = false;
  }
  // Есть изменения в данных, очищаем экран
  ssd1306_Fill(Black);

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
/* USER CODE END Application */

