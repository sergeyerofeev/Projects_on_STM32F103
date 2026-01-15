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
#include "queue.h"
#include "usbd_customhid.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "max31865.h"
#include "my_functions.h"
#include "definition.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
QueueHandle_t xQueueSSD1306;
QueueHandle_t xQueueReceivingUSB;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern MAX31865_t pt100;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };

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

extern void MX_USB_DEVICE_Init(void);
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
  // Очередь для передачи данных на дисплей SSD1306
  xQueueSSD1306 = xQueueCreate(10, sizeof(varMAX31865_t));
  if (xQueueSSD1306 == NULL) {
    vErrorHandler();
  }
  // Очередь для передачи принятых данных по USB
  xQueueReceivingUSB = xQueueCreate(10, sizeof(varReceivingUSB_t));
  if (xQueueReceivingUSB == NULL) {
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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  varReceivingUSB_t varData;
  // Сбрасываем линию USB_DP при перезагрузке микроконтроллера
  HAL_GPIO_WritePin(DP_RESET_GPIO_Port, DP_RESET_Pin, GPIO_PIN_RESET);
  vTaskDelay(20);
  HAL_GPIO_WritePin(DP_RESET_GPIO_Port, DP_RESET_Pin, GPIO_PIN_SET);
  /* Infinite loop */
  for (;;) {
    // Ожидание данных из очереди
    if (xQueueReceive(xQueueReceivingUSB, &varData, portMAX_DELAY) == pdPASS) {
      //uint8_t sendReport[4] = { varData.reportID, varData.arrayKx[0], varData.arrayKx[1], varData.arrayKx[2] };
      // Что то делаем с данными.
      //USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, sendReport, 4);
    }
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
  varMAX31865_t varData = { 0, 0 };
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    varData.temp = Max31865_ReadTempC(&pt100, &varData.res);
    // Отправляем значения температуры и сопротивления в очередь
    xQueueSend(xQueueSSD1306, &varData, portMAX_DELAY);

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
  }
}

// Callback функция задачи вывода на экран данных, из очереди задач
void vTaskShow(void *argument) {
  varMAX31865_t varData = { 0, 0 };
  char tempStr[10] = { 0 };
  char resStr[10] = { 0 };
  uint8_t x = 0, y = 0, yMiddle = SSD1306_HEIGHT / 2 - Font_7x10.height / 2;
  int length = 0;

  for (;;) {
    // Ожидание данных из очереди
    if (xQueueReceive(xQueueSSD1306, &varData, portMAX_DELAY) == pdPASS) {

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

      // Отправляем измененную температуру по USB
      uint16_t temp = (uint16_t) varData.temp;
      USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t[] ) { temp >> 8, temp | 0xFF, 0, 0 }, 4);
    }
  }
}
/* USER CODE END Application */

