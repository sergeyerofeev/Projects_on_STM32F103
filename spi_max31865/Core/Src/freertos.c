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
#define DT 500
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
  float dt = DT / 1000.0f;
  const float setPoint = 220.0f;        // Целевая температура

  float deltaPerCycle = 4 * dt;         // Скорость нагрева 2°C/цикл
  float deltaTemp = 0.0f;               // Разность между уставкой setPoint и текущй температурой
  float boostKp = setPoint / 5.0f;      // На данном участке Kp увеличивается
  float limitedSetpoint = 0.0f;         // Ограниченная уставка
  float deltaSetpoint = 0.0f;

  const float minOut = 0.0f;
  //float maxOut = (float) htim4.Init.Period + 1;
  const float maxOut = 4800.0f;
  float errorCurrent = 0.0f;
  float errorIntegral = 0.0f;

  float result = 0.0f;

  // Сбрасываем линию USB_DP при перезагрузке микроконтроллера
  HAL_GPIO_WritePin(DP_RESET_GPIO_Port, DP_RESET_Pin, GPIO_PIN_RESET);
  vTaskDelay(20);
  HAL_GPIO_WritePin(DP_RESET_GPIO_Port, DP_RESET_Pin, GPIO_PIN_SET);

  for (;;) {
    // Ожидание данных из очереди
    if (receivingData.reportID == 1) {
      // Проверяем, активен ли выход канала 3, если нет, то запускаем ШИМ
      if ((TIM4->CCER & TIM_CCER_CC3E) == 0) {
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
        limitedSetpoint = varData.temp;  // Начинаем с текущей температуры
      }

      // Вычисляем разность между целевой и ограниченной уставками
      deltaSetpoint = setPoint - limitedSetpoint;

      // Применение ограничения скорости
      if (deltaSetpoint > deltaPerCycle) {
        // Нагревание, с каждым циклом увеличиваем limitedSetpoint на 2°C/цикл
        limitedSetpoint += deltaPerCycle;
      } else if (deltaSetpoint < -deltaPerCycle) {
        // Остывание, с каждым циклом уменьшаем limited_setpoint на 2°C/цикл
        limitedSetpoint -= deltaPerCycle;
      } else {
        // Температура достигла уставки
        limitedSetpoint = setPoint;
      }

      // Вычисление ошибки, разница между уставкой и измеренной температурой
      errorCurrent = limitedSetpoint - varData.temp;

      errorIntegral += errorCurrent * dt * receivingData.ki / 5.0f;

      deltaTemp = setPoint - varData.temp;
      if (deltaTemp < boostKp && deltaTemp > 0) {
        // Увеличение коэффициента kp включаем только при приближении к setPoint, снизу
        result = errorCurrent * receivingData.kp * 2 + errorIntegral;
      } else {
        result = errorCurrent * receivingData.kp + errorIntegral;
      }

      // Проверяем насыщение выхода
      if (result >= maxOut) {
        result = maxOut;
      } else if (result <= minOut) {
        result = minOut;
      }

      htim4.Instance->CCR3 = (uint16_t) result;
    } else if (receivingData.reportID == 2) {
      // Проверяем, активен ли выход канала 3, если да, то останавливаем ШИМ
      if (TIM4->CCER & TIM_CCER_CC3E) {
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
        htim4.Instance->CCR3 = 0;
        errorIntegral = 400.0f;
      }
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(DT));
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void vErrorHandler() {
  __set_FAULTMASK(1); // Запрещаем все маскируемые прерывания
  NVIC_SystemReset(); // Программный сброс
  __NOP();
}

// Callback фукнция задачи получения значения температуры с датчика MAX31865
void vTaskGetTemp(void *argument) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  bool flagShow = true;

  for (;;) {
    varData.temp = Max31865_ReadTempC(&hMAX31865, &varData.res);
    //if (show1s >= 4) {
    if (flagShow) {
      // Отправляем уведомление задаче вывода информации на дисплей
      xTaskNotifyGive(taskShowHandle);
    }
    flagShow ^= true;
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(DT));
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

    length = floatToString(resStr, 10, varData.res);
    // Выводим значение сопротивления датчика PT100 на дисплей
    x = (SSD1306_WIDTH - length * Font_7x10.width) / 2;
    y = yMiddle + 10;
    ssd1306_SetCursor(x, y);
    ssd1306_WriteString(resStr, Font_7x10, White);

    ssd1306_UpdateScreen();

    // Отправляем измененную температуру и время в секундах, по линии USB
    tempArr = transformFloat(varData.temp);
    memcpy(sendArray, tempArr.array, 3);
    if (receivingData.reportID == 1) {
      sendArray[3] = timeCount >> 8;
      sendArray[4] = timeCount & 0xFF;
      timeCount++;
    } else if (receivingData.reportID == 2 && timeCount != 0) {
      sendArray[3] = 0;
      sendArray[4] = 0;
      timeCount = 0;
    }
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, sendArray, 5);
  }
}
/* USER CODE END Application */

