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
TaskHandle_t taskShowHandle;

/*QueueHandle_t xQueueSSD1306;*/
extern USBD_HandleTypeDef hUsbDeviceFS;
extern TIM_HandleTypeDef htim4;

// Данные полученные по USB
extern varReceivingUSB_t receivingData;
extern MAX31865_t hMAX31865;
varMAX31865_t varData;

uint16_t ccr3 = 0;

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
// Ограничительная функция
static float _constrain(float data, float minOut, float maxOut);
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
  /* add queues, ... */
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
  if (xTaskCreate(vTaskShow, NULL, 128, NULL, osPriorityNormal, &taskShowHandle) == pdFAIL) {
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
  TickType_t xLastWakeTime = xTaskGetTickCount();
  float setPoint = 100.0;
  float minOut = 0;
  float maxOut = (float) htim4.Init.Period + 1;
  float errorCurrent = 0;
  float errorPrevious = 0;
  float errorIntegral = 0;
  float errorDifferential = 0;
  // Сбрасываем линию USB_DP при перезагрузке микроконтроллера
  HAL_GPIO_WritePin(DP_RESET_GPIO_Port, DP_RESET_Pin, GPIO_PIN_RESET);
  vTaskDelay(20);
  HAL_GPIO_WritePin(DP_RESET_GPIO_Port, DP_RESET_Pin, GPIO_PIN_SET);
  /* Infinite loop */
  for (;;) {
    // Ожидание данных из очереди
    if (receivingData.reportID == 1) {
      // Проверяем, активен ли выход канала 3, если нет, то запускаем ШИМ
      if ((TIM4->CCER & TIM_CCER_CC3E) == 0) {
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
      }
      errorCurrent = setPoint - varData.temp;
      errorIntegral = errorIntegral + errorCurrent * (receivingData.ki / 5);
      errorDifferential = errorCurrent - errorPrevious;
      errorPrevious = errorCurrent;
      ccr3 = (uint16_t) _constrain(errorCurrent * receivingData.kp + errorIntegral + errorDifferential * receivingData.kd, minOut, maxOut);
      htim4.Instance->CCR3 = ccr3;
    } else if (receivingData.reportID == 2) {
      // Проверяем, активен ли выход канала 3, если да, то останавливаем ШИМ
      if (TIM4->CCER & TIM_CCER_CC3E) {
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
        ccr3 = 0;
        htim4.Instance->CCR3 = ccr3;
        errorCurrent = 0;
        errorPrevious = 0;
        errorIntegral = 0;
        errorDifferential = 0;
      }
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
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
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    varData.temp = Max31865_ReadTempC(&hMAX31865, &varData.res);
    // Отправляем уведомление задаче вывода информации на дисплей
    xTaskNotifyGive(taskShowHandle);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
  }
}

// Callback функция задачи вывода на экран данных, из очереди задач
void vTaskShow(void *argument) {
  char tempStr[10] = { 0 };
  char resStr[10] = { 0 };
  uint8_t x = 0, y = 0, yMiddle = SSD1306_HEIGHT / 2 - Font_7x10.height / 2;
  int length = 0;
  DataNum_t tempArr;
  uint8_t sendArray[5];
  uint16_t timeCount = 0;

  for (;;) {
    // Ожидаем уведомление
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    length = floatToString(tempStr, 10, varData.temp);
    // Выводим значение температуры на дисплей
    x = (SSD1306_WIDTH - length * Font_7x10.width) / 2;
    y = yMiddle - 10;
    ssd1306_SetCursor(x, y);
    ssd1306_WriteString(tempStr, Font_7x10, White);

    length = floatToString(resStr, 10, ccr3);
    // Выводим значение сопротивления датчика PT100 на дисплей
    x = (SSD1306_WIDTH - length * Font_7x10.width) / 2;
    y = yMiddle + 10;
    ssd1306_SetCursor(x, y);
    ssd1306_WriteString(resStr, Font_7x10, White);

    ssd1306_UpdateScreen();

    if (receivingData.reportID == 1) {
      // Отправляем измененную температуру и время в секундах, по линии USB
      tempArr = transformFloat(varData.temp);
      memcpy(sendArray, tempArr.array, 3);
      sendArray[3] = timeCount >> 8;
      sendArray[4] = timeCount & 0xFF;
      USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, sendArray, 5);
      timeCount++;
    } else if (receivingData.reportID == 2 && timeCount > 0) {
      timeCount = 0;
    }
  }
}

// Ограничительная функция
static float _constrain(float data, float minOut, float maxOut) {
  if (data > maxOut) {
    return maxOut;
  } else if (data < minOut) {
    return minOut;
  } else {
    return data;
  }
}
/* USER CODE END Application */

