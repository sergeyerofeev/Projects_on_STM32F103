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
#include "semphr.h"
#include "usart1_ring.h"
#include "usart3_ring.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  size_t xDataLength; // Количество записанных данных
  size_t xBlockSize;  // Полный размер массива
  uint8_t uData[];   // Массив для данных
} MemBlock_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Первоначальный размер массива для передачи по TX и количество добавляемых байт
#define INITIAL_BLOCK_SIZE 64
// Длина начальное строки для идентификации линии
#define INITIAL_LENGTH 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// Дескрипторы задач
TaskHandle_t uart1RxHandle;
TaskHandle_t uart3RxHandle;
TaskHandle_t uart2TxHandle;
// Очередь для передачи данных между задачами
QueueHandle_t xUartQueue;
// Семафор для синхронизации передачи DMA Uart2 TX
SemaphoreHandle_t txCompleted;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void vTaskUart1Rx(void *argument);
void vTaskUart3Rx(void *argument);
void vTaskUart2Tx(void *argument);
// Обработчик ошибок
void vErrorHandler(void);
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
    vErrorHandler();
  }
  // Сразу освобождаем семафор
  xSemaphoreGive(txCompleted);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  // Создание очереди (хранит указатели на MemBlock_t)
  xUartQueue = xQueueCreate(10, sizeof(MemBlock_t*));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if (xTaskCreate(vTaskUart1Rx, "taskUart1Rx", 128, NULL, osPriorityNormal, &uart1RxHandle) == pdFAIL) {
    vErrorHandler();
  }

  if (xTaskCreate(vTaskUart3Rx, "taskUart3Rx", 128, NULL, osPriorityNormal, &uart3RxHandle) == pdFAIL) {
    vErrorHandler();
  }

  if (xTaskCreate(vTaskUart2Tx, "taskUart2Tx", 128, NULL, osPriorityNormal, &uart2TxHandle) == pdFAIL) {
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
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (huart->Instance == USART2) {
    xSemaphoreGiveFromISR(txCompleted, &xHigherPriorityTaskWoken);
  }

  // Единая точка yield для всех условий
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void vErrorHandler() {
  __set_FAULTMASK(1); // Запрещаем все маскируемые прерывания
  NVIC_SystemReset(); // Программный сброс
}

void vTaskUart1Rx(void *argement) {
  uint8_t dataUart1 = 0;
  MemBlock_t *pxMemBlock = NULL;

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(8));
    if (pxMemBlock == NULL) {
      // Инициализация начального блока памяти
      pxMemBlock = (MemBlock_t*) pvPortMalloc(sizeof(MemBlock_t) + INITIAL_BLOCK_SIZE);
      if (pxMemBlock == NULL) {
        vErrorHandler();
      }
      pxMemBlock->xBlockSize = INITIAL_BLOCK_SIZE;
      // Записываем начальные символы инициализирующие линию
      memcpy(pxMemBlock->uData, (uint8_t[] ) { 'R', 'x', ':' }, INITIAL_LENGTH);
      pxMemBlock->xDataLength = INITIAL_LENGTH;
    }
    if (uart1_available()) {
      while (uart1_available()) {
        dataUart1 = uart1_read(); // Читаем пришедший байт

        // Проверяем необходимость расширения памяти
        if (pxMemBlock->xDataLength + INITIAL_LENGTH >= pxMemBlock->xBlockSize) {
          // Увеличиваем размер массива в новом блоке на INITIAL_BLOCK_SIZE байт
          MemBlock_t *pxNewBlock = (MemBlock_t*) pvPortMalloc(sizeof(MemBlock_t) + INITIAL_BLOCK_SIZE);
          if (pxNewBlock == NULL) {
            vErrorHandler();
          }
          // Копирование данных в новый блок
          memcpy(pxNewBlock->uData, pxMemBlock->uData, pxMemBlock->xDataLength);
          pxNewBlock->xDataLength = pxMemBlock->xDataLength;
          pxNewBlock->xBlockSize = pxMemBlock->xBlockSize + INITIAL_BLOCK_SIZE;

          // Освобождение старого блока и замена на новый
          vPortFree(pxMemBlock);

          pxMemBlock = pxNewBlock;
        }
        // Преобразуем шестнадцатиричное число в два символа
        pxMemBlock->uData[pxMemBlock->xDataLength++] = ' ';
        pxMemBlock->uData[pxMemBlock->xDataLength++] = "0123456789ABCDEF"[dataUart1 >> 4 & 0x0F];
        pxMemBlock->uData[pxMemBlock->xDataLength++] = "0123456789ABCDEF"[dataUart1 & 0x0F];
      }
    } else {
      if (pxMemBlock->xDataLength > INITIAL_LENGTH) {
        // Данные полностью скопированы, отправляем блок в очередь задач
        if (xQueueSend(xUartQueue, &pxMemBlock, portMAX_DELAY) != pdPASS) {
          // Если не удалось отправить в очередь, освобождаем память
          vPortFree(pxMemBlock);
          pxMemBlock = NULL;
        }
      }
    }
  }
}

void vTaskUart3Rx(void *argement) {
  for (;;) {
    vTaskSuspend(NULL);
  }
}

void vTaskUart2Tx(void *argement) {
  MemBlock_t *pxMemBlock = NULL;

  for (;;) {
// Ожидание данных из очереди
    if (xQueueReceive(xUartQueue, &pxMemBlock, portMAX_DELAY) == pdPASS) {
      if (pxMemBlock != NULL && pxMemBlock->xDataLength > 0) {
        // Ожидание семафора от предыдущей передачи
        if (xSemaphoreTake(txCompleted, portMAX_DELAY) == pdTRUE) {
          // Запуск передачи через DMA
          if (HAL_UART_Transmit_DMA(&huart2, pxMemBlock->uData, pxMemBlock->xDataLength) == HAL_OK) {
            // Ожидание семафора от текущей передачи
            if (xSemaphoreTake(txCompleted, portMAX_DELAY) == pdTRUE) {
              // Данные успешно переданы, даём семафор
              xSemaphoreGive(txCompleted);
            }
          }
        }
        // Освобождаем память
        vPortFree(pxMemBlock);
        pxMemBlock = NULL;
      }
    }
  }
}
/* USER CODE END Application */

