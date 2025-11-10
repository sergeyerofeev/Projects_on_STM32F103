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
#include <stdbool.h>
#include <string.h>
#include "semphr.h"
#include "i2c.h"
#include "i2c_er.h"
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
SemaphoreHandle_t xBinarySemaphore;

// Дескрипторы задач
TaskHandle_t countShowHandle;
TaskHandle_t checkBusyHandle;
// Все ошибки складываем в одну переменную
bool isError = false;
// Ошибка на линии I2C
extern bool isBusy;
// Счётчик для фонового вывода чисел на экран
uint8_t countFlag = 0;
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
void vCheckBusy(void*);
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
  xBinarySemaphore = xSemaphoreCreateBinary();
  if (xBinarySemaphore == NULL) {
    // Ошибка создания семафора
    isError = true;
  }
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
  if (xTaskCreate(vCheckBusy, "checkBusy", 128, NULL, osPriorityNormal1, &checkBusyHandle) == pdFAIL) {
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

// С периодичностью в 1 секунду показываем на экране значение счётчика
void vCountShow(void *argument) {
  for (;;) {
    // Если данные от BMS не поступили выводим в центре экрана значение счётчика
    strCount[0] = "0123456789ABCDEF"[countFlag >> 4 & 0x0F];
    strCount[1] = "0123456789ABCDEF"[countFlag & 0x0F];
    // Размещаем строку по центру экрана
    x = (SSD1306_WIDTH - strlen(strCount) * Font_16x24.width) / 2;
    y = SSD1306_HEIGHT / 2 - Font_16x24.height / 2;
    for (;;) {
      ssd1306_SetCursor(x, y);
      ssd1306_Fill(Black);
      ssd1306_WriteString(strCount, Font_16x24, White);
      ssd1306_UpdateScreen();
      if (isBusy) {
        isBusy = false;
        // Ошибка при доступе к модулю I2C
        // Повышаем приоритет задачи, чтобы снова первой запуститься после решения проблемы
        vTaskPrioritySet(NULL, osPriorityNormal1);
        // Отправляем уведомление
        xTaskNotify(vCheckBusy, (uint32_t ) countShowHandle, eSetValueWithOverwrite);
        // Переводим задачу в режим ожидания
        vTaskSuspend(NULL);

        // После возврата снова попытаемся получить значение по сохранённому адресу
        // Понижаем приоритет текущей задачи до обычного
        vTaskPrioritySet(NULL, osPriorityNormal);
      } else {
        break;
      }
    }
    countFlag++;

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void vCheckBusy(void *argument) {
  for (;;) {
    uint32_t handleValue = 0;
    xTaskNotifyWait(0, 0, &handleValue, portMAX_DELAY);
    // Отключаем питание модуля I2C, высокий уровень сигнала отключает PNP транзистор
    HAL_GPIO_WritePin(EE_GPIO_Port, EE_Pin, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(1000));
    for (;;) {
      // HAL_IWDG_Refresh(&hiwdg); // Сбрасываем IWDG
      // Подаём питание на модуль I2C, низкий уровень сигнала включает PNP транзистор
      HAL_GPIO_WritePin(EE_GPIO_Port, EE_Pin, GPIO_PIN_RESET);
      vTaskDelay(pdMS_TO_TICKS(1000));
      // Проверям готовность, 5 попыток, таймаут 100 мс
      if (true/*HAL_I2C_IsDeviceReady(&hi2c1, ADDRESS, 5, 100) == HAL_OK*/) {
        // Модуль I2C успешно перезапустился, выходим из внутреннего цикла
        break;
      } else {
        // Если возвращаемое значение не равно HAL_OK
        // Отключаем питание модуля I2C
        HAL_GPIO_WritePin(EE_GPIO_Port, EE_Pin, GPIO_PIN_SET);
        // Запускаем процедуру переиницализации I2C1
        I2C_ClearBusyFlagErratum(&hi2c1, 100);
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
    }
    // HAL_IWDG_Refresh(&hiwdg); // Сбрасываем IWDG
    // Модуль I2C успешно перезапустился
    // Возвращаем вызвавшую задачу из режима ожидания
    vTaskResume((TaskHandle_t) handleValue);
  }
}

// Программный сброс, в случае ошибок при инициализации задач
void vTaskSystemReset(void *argument) {
  for (;;) {
    if (isError) {
      __set_FAULTMASK(1); // Запрещаем все маскируемые прерывания
      NVIC_SystemReset(); // Программный сброс
    } else {
      // Если ошибок нет, удаляем текущую задачу
      vTaskDelete(NULL);
    }
  }
}
/* USER CODE END Application */

