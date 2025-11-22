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
TaskHandle_t uart1RxHandle;
TaskHandle_t uart3RxHandle;

SemaphoreHandle_t txCompleted;

// Все ошибки складываем в одну переменную
bool isError = false;

char strRx[SIZE_BF] = { 'R', 'x', ':', ' ' };
char strTx[SIZE_BF] = { 'T', 'x', ':', ' ' };
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

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
  /* add mutex, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */

  txCompleted = xSemaphoreCreateBinary();
  if (txCompleted == NULL) {
    isError = true;
  }
  // Сразу освобождаем семафор
  xSemaphoreGive(txCompleted);
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
  xTaskCreate(vTaskSystemReset, "taskSystemReset", 128, NULL, osPriorityHigh, NULL);

  if (xTaskCreate(vTaskUart1Rx, "taskUart1Rx", 128, NULL, osPriorityNormal, &uart1RxHandle) == pdFAIL) {
    isError = true;
  }

  if (xTaskCreate(vTaskUart3Rx, "taskUart3Rx", 128, NULL, osPriorityNormal, &uart3RxHandle) == pdFAIL) {
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
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if(huart->Instance == USART2)
  {
      xSemaphoreGiveFromISR(txCompleted, &xHigherPriorityTaskWoken);
  }

  // Единая точка yield для всех условий
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

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

void vTaskUart1Rx(void *argement) {
  size_t i = 4;
  uint8_t dataUart1 = 0;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(8));
    if (uart1_available()) {
      while (uart1_available()) {
        dataUart1 = uart1_read(); // Читаем пришедший байт
        // Преобразуем шестнадцатиричное число в два символа
        strRx[i++] = "0123456789ABCDEF"[dataUart1 >> 4 & 0x0F];
        strRx[i++] = "0123456789ABCDEF"[dataUart1 & 0x0F];
        strRx[i++] = ' ';

        if (i >= SIZE_BF - 4) {
          // Проверяем возможность вставить следующие четыре символа
          strRx[i] = '\0';
          // Приёмный буфер переполнен, приводим счётчик в исходное состояние
          i = 4;
          break;
        }
      }
      continue;
    } else {
      if (i > 4) {
        strRx[i++] = '\n';
        strRx[i] = '\0';
        // Данные полностью скопированы, приводим счётчик в исходное состояние
        i = 4;
        // Передаём полученные данные на Terminal
        HAL_UART_Transmit(&huart2, (uint8_t*) strRx, strlen(strRx), 100);
      }
    }
  }
}
/* USER CODE END Application */

