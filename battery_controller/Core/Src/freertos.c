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
#include "event_groups.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "crc32.h"
#include "iwdg.h"
#include "i2c_er.h"
#include "my_config.h"
#include "func_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern ADC_HandleTypeDef hadc1;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
EventGroupHandle_t errorEvent;
SemaphoreHandle_t mutexMem;

TaskHandle_t batStatusHandle;
TaskHandle_t checkMemHandle;
TaskHandle_t taskBatMonitorHandle;

TimerHandle_t iwdtTimer;
TimerHandle_t adcTimer;

extern ADC_HandleTypeDef hadc1;

// Буфер для измеренных значений с ADC
extern uint16_t adcBuffer[];
/* USER CODE END Variables */
/* Definitions for logTask */
osThreadId_t logTaskHandle;
const osThreadAttr_t logTask_attributes = { .name = "logTask", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityLow, };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void taskBatMonitorFunc(void*);
void vBatStatusFunc(void*);

void vAdcTimerCallback(TimerHandle_t);
void vIwdtTimerCallback(TimerHandle_t);
/* USER CODE END FunctionPrototypes */

void logTaskFunc(void *argument);

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
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  mutexMem = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  // Таймер для IWDT
  iwdtTimer = xTimerCreate("iwdtTimer", pdMS_TO_TICKS(IWDT_PERIOD), pdTRUE, (void*) 0, vIwdtTimerCallback);
  if (iwdtTimer == NULL || (xTimerStart(iwdtTimer, 100) == pdFAIL)) {
    // Таймер не создан или ошибка при запуске, перезапускаем микроконтроллер
    __set_FAULTMASK(1); // Запрещаем все маскируемые прерывания
    NVIC_SystemReset(); // Программный сброс
  }
  // Таймер для периодического запуска измерения напряжения батареи
  adcTimer = xTimerCreate("adcTimer", pdMS_TO_TICKS(ADC_PERIOD), pdTRUE, (void*) 0, vAdcTimerCallback);
  if (adcTimer == NULL || (xTimerStart(adcTimer, 100) == pdFAIL)) {
    // Таймер не создан или ошибка при запуске, устанавливаем третий бит
    xEventGroupSetBits(errorEvent, 0x04);
  }
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of logTask */
  logTaskHandle = osThreadNew(logTaskFunc, NULL, &logTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  // Задача для однократного запуска и получения данных из EEPROM о статусе батареи
  if (xTaskCreate(vBatStatusFunc, "batStatus", 128, NULL, osPriorityNormal1, &batStatusHandle) == pdFAIL) {
    xEventGroupSetBits(errorEvent, 0x01);
  }
  // Задача проверки готовности EEPROM и перезапуска линиии I2C в случае сбоя
  if (xTaskCreate(vCheckMemFunc, "checkMem", 128, NULL, osPriorityNormal1, &checkMemHandle) == pdFAIL) {
    xEventGroupSetBits(errorEvent, 0x01);
  }

  if (xTaskCreate(taskBatMonitorFunc, "batMonitor", 128, NULL, osPriorityNormal, &taskBatMonitorHandle) == pdFAIL) {
    // Если ошибка при создании задач устанавливаем первый бит
    xEventGroupSetBits(errorEvent, 0x01);
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  // Создаём группу для регистрации ошибок
  errorEvent = xEventGroupCreate();
  if (errorEvent == NULL) {
    // Группа не создана, перезапускаем микроконтроллер
    __set_FAULTMASK(1); // Запрещаем все маскируемые прерывания
    NVIC_SystemReset(); // Программный сброс
  }
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_logTaskFunc */
/**
 * @brief  Function implementing the logTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_logTaskFunc */
void logTaskFunc(void *argument)
{
  /* USER CODE BEGIN logTaskFunc */
  /* Infinite loop */
  for (;;) {
    vTaskDelete(NULL);
  }
  /* USER CODE END logTaskFunc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void vIwdtTimerCallback(TimerHandle_t timerHandle)
{
  // Проверяем установленные флаги ошибок
  if (xEventGroupGetBits(errorEvent) == 0x00) {
    // Ошибок нет, сбрасываем IWDG
    HAL_IWDG_Refresh(&hiwdg);
  }
}

void vAdcTimerCallback(TimerHandle_t timerHandle)
{
  // Запускаем измерение напряжения, после измерения DMA автоматически остановиться
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adcBuffer, ADC_BUFFER_SIZE);
}
/* USER CODE END Application */

