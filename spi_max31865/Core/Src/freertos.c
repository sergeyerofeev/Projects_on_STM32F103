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
#include "tim.h"
#include "queue.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "max31865.h"
#include "my_functions.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  float temp;     // Вычисленная температура
  float res;      // Текущее сопротивление датчика PT100
} varData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Тайминги (в мс)
#define LONG_PRESS_TIME_MS      1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
QueueHandle_t xQueueStr;
TaskHandle_t encoderHandle;
extern MAX31865_t pt100;
// Значения энкодера
uint16_t newCount = 800;
uint16_t prevCount = 800;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// Обработчик ошибок
void vErrorHandler(void);
// Прототип callback фукнции задачи вывода значения энкодера на экран
void vTaskEncoder(void *argument);
// Прототип callback фукнции задачи получения значения температуры с датчика MAX31865
void vTaskGetTemp(void *argument);
// Прототип callback функции задачи вывода на экран данных, из очереди задач
void vTaskShow(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN PREPOSTSLEEP */
__weak void PreSleepProcessing(uint32_t ulExpectedIdleTime) {
  /* place for user code */
}

__weak void PostSleepProcessing(uint32_t ulExpectedIdleTime) {
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
  xQueueStr = xQueueCreate(10, sizeof(varData_t));
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
  // Задача вывода значения температуры на экран
  if (xTaskCreate(vTaskShow, NULL, 128, NULL, osPriorityNormal, NULL) == pdFAIL) {
    vErrorHandler();
  }
  // Задача вывода значения энкодера на экран
  if (xTaskCreate(vTaskEncoder, NULL, 128, NULL, osPriorityNormal, &encoderHandle) == pdFAIL) {
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
void vErrorHandler() {
  __set_FAULTMASK(1); // Запрещаем все маскируемые прерывания
  NVIC_SystemReset(); // Программный сброс
}

// Callback обработчик прерывания энкодера TIM3
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) {
    newCount = (uint16_t) __HAL_TIM_GET_COUNTER(htim);

    if (newCount - prevCount >= 4 || prevCount - newCount >= 4) {
      prevCount = newCount;
      // Возобновляем задачу вывода значения энкодера на экран
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      if (xTaskResumeFromISR(encoderHandle) == pdTRUE) {
        xHigherPriorityTaskWoken = pdTRUE;
      }
      // Если требуется переключение контекста
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    }
  }
}

// Callback обработчик внешних прерываний
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  static GPIO_PinState selectPreviousState = GPIO_PIN_RESET;
  static GPIO_PinState editPreviousState = GPIO_PIN_RESET;
  static TickType_t selectRisingTime;
  static TickType_t editRisingTime;

  if (GPIO_Pin == SELECT_Pin) {
    GPIO_PinState selectCurrentState = HAL_GPIO_ReadPin(SELECT_GPIO_Port, SELECT_Pin);
    if (selectPreviousState == GPIO_PIN_RESET && selectCurrentState == GPIO_PIN_SET) {
      // Обнаружен RISING фронт
      selectRisingTime = xTaskGetTickCount();
    } else if (selectPreviousState == GPIO_PIN_SET && selectCurrentState == GPIO_PIN_RESET) {
      // Обнаружен FALLING фронт
      if (xTaskGetTickCount() - selectRisingTime < pdMS_TO_TICKS(LONG_PRESS_TIME_MS)) {
        // Короткое нажатие кнопки
      } else {
        // Длинное нажатие кнопки
      }
    }
    selectPreviousState = selectCurrentState;
  }
  if (GPIO_Pin == EDIT_Pin) {
    GPIO_PinState editCurrentState = HAL_GPIO_ReadPin(EDIT_GPIO_Port, EDIT_Pin);
    if (editPreviousState == GPIO_PIN_RESET && editCurrentState == GPIO_PIN_SET) {
      // Обнаружен RISING фронт
      editRisingTime = xTaskGetTickCount();
    } else if (editPreviousState == GPIO_PIN_SET && editCurrentState == GPIO_PIN_RESET) {
      // Обнаружен FALLING фронт
      if (xTaskGetTickCount() - editRisingTime < pdMS_TO_TICKS(LONG_PRESS_TIME_MS)) {
        // Короткое нажатие кнопки
      } else {
        // Длинное нажатие кнопки
      }
    }
    editPreviousState = editCurrentState;
  }
}

// Callback фукнция задачи вывода значения энкодера на экран
void vTaskEncoder(void *argument) {
  uint8_t x = 0, y = 0;
  char str[18] = { 0 };
  int length = 0;
  for (;;) {
    length = snprintf(str, 18, "%d - %d", newCount, prevCount >> 2);
    // Очищаем поле для вывода
    ssd1306_ClearArea(0, y, SSD1306_WIDTH, 10, Black);
    // Выводим значение энкодера на дисплей
    x = (SSD1306_WIDTH - length * Font_7x10.width) / 2;
    ssd1306_SetCursor(x, y);
    ssd1306_WriteString(str, Font_7x10, White);

    ssd1306_UpdateScreen();
    vTaskSuspend(NULL);
  }
}

// Callback фукнция задачи получения значения температуры с датчика MAX31865
void vTaskGetTemp(void *argument) {
  varData_t varData = { 0, 0 };
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    varData.temp = Max31865_ReadTempC(&pt100, &varData.res);
    // Отправляем значения температуры и сопротивления в очередь
    xQueueSend(xQueueStr, &varData, portMAX_DELAY);

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
  }
}

// Callback функция задачи вывода на экран данных, из очереди задач
void vTaskShow(void *argument) {
  varData_t varData = { 0, 0 };
  char tempStr[10] = { 0 };
  char resStr[10] = { 0 };
  uint8_t x = 0, y = 0, yMiddle = SSD1306_HEIGHT / 2 - Font_7x10.height / 2;
  int length = 0;

  for (;;) {
    // Ожидание данных из очереди
    if (xQueueReceive(xQueueStr, &varData, portMAX_DELAY) == pdPASS) {

      length = floatToString(tempStr, 10, varData.temp);
      // Выводим значение температуры на дисплей
      x = (SSD1306_WIDTH - length * Font_7x10.width) / 2;
      y = yMiddle - 10;
      ssd1306_SetCursor(x, y);
      ssd1306_WriteString(tempStr, Font_7x10, White);

      length = floatToString(resStr, 10, varData.res);
      // Выводим значение сопротивления датчика PT100 на дисплей
      x = (SSD1306_WIDTH - length * Font_7x10.width) / 2;
      y = yMiddle + 10;
      ssd1306_SetCursor(x, y);
      ssd1306_WriteString(resStr, Font_7x10, White);

      ssd1306_UpdateScreen();

    }
  }
}
/* USER CODE END Application */

