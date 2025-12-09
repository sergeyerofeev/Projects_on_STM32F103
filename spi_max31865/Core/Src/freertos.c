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
#include <string.h>
#include "queue.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "max31865.h"
#include "my_functions.h"
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
QueueHandle_t xQueueStr;
extern MAX31865_t pt100;
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
// Обработчик ошибок
void vErrorHandler(void);
// Прототип callback фукнции задачи получения значения температуры с датчика MAX31865
void vTaskGetTemp(void *argument);
// Прототип callback функции задачи вывода на экран данных, из очереди задач
void vTaskShow(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN PREPOSTSLEEP */
__weak void PreSleepProcessing(uint32_t *ulExpectedIdleTime)
{
/* place for user code */
}

__weak void PostSleepProcessing(uint32_t *ulExpectedIdleTime)
{
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
  xQueueStr = xQueueCreate(10, sizeof(char*));
  if (xQueueStr == NULL) {
    vErrorHandler();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  // Задача получения температуры с датчика MAX31865
  if (xTaskCreate(vTaskGetTemp, NULL, 128, NULL, osPriorityNormal, NULL) == pdFAIL) {
    vErrorHandler();
  }

  if (xTaskCreate(vTaskShow, NULL, 128, NULL, osPriorityNormal, NULL) == pdFAIL) {
    vErrorHandler();
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
void vErrorHandler() {
  __set_FAULTMASK(1); // Запрещаем все маскируемые прерывания
  NVIC_SystemReset(); // Программный сброс
}

// Callback фукнция задачи получения значения температуры с датчика MAX31865
void vTaskGetTemp(void *argument) {
  float res = 0, pt100Temp = 0;
  DataNum_t num = { 0, 0 };
  char *buffer = NULL;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    pt100Temp = Max31865_ReadTempC(&pt100, &res);
    num = transformFloat(pt100Temp);
    // Вычисляем сколько места необходимо динамически выделить для форматированной строки
    int bufferSize = snprintf(NULL, 0, " %d.%d ", num.integerPart, num.fractional) + sizeof((char) '\0');
    buffer = pvPortMalloc(bufferSize);
    if (buffer != NULL) {
      numToStr(buffer, bufferSize, num);
      // Помещаем указатель на строку в очередь задач
      if (xQueueSend(xQueueStr, &buffer, portMAX_DELAY) == pdPASS) {
        // Ссылка на указатель успешно отправлена в очередь, указатель больше не нужен
        buffer = NULL;

        // Память будет освобождена в задаче вывода на дисплей
      } else {
        // Если не удалось отправить в очередь, освобождаем память
        vPortFree(buffer);
        buffer = NULL;
      }
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
  }
}

// Callback функция задачи вывода на экран данных, из очереди задач
void vTaskShow(void *argument) {
  uint8_t x = 0, y = 0;
  char *buffer = NULL;
  for (;;) {
    // Ожидание данных из очереди
    if (xQueueReceive(xQueueStr, &buffer, portMAX_DELAY) == pdPASS) {
      if (buffer != NULL) {
        int length = strlen(buffer);
        // Выводим значение на дисплей
        // Размещаем строку по центру экрана
        x = (SSD1306_WIDTH - length * Font_7x10.width) / 2;
        y = SSD1306_HEIGHT / 2 - Font_7x10.height / 2;
        ssd1306_SetCursor(x, y);
        ssd1306_WriteString(buffer, Font_7x10, White);

        ssd1306_UpdateScreen();

        // Данные успешно переданы, освобождаем память и обнуляем указатель
        vPortFree(buffer);
        buffer = NULL;
      }
    }
  }
}
/* USER CODE END Application */

