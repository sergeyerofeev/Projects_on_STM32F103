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
#include "timers.h"
#include "crc32.h"
#include <stdbool.h>
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
TaskHandle_t taskBatMonitorHandle;
TimerHandle_t xTimer;
extern I2C_HandleTypeDef hi2c1;

// Буфер для измеренных значений с ADC
extern uint16_t adcBuffer[];
// Флаг готовности микросхемы
extern bool isMemReady;
// Буфер для работы с EEPROM (чтение и запись)
extern uint8_t memBuffer[];
// Статус зарядки, true - батарея находится на зарядке
extern bool isСharged;
// Количество циклов зарядки батареи записанные в EEPROM
extern uint16_t chargeCycles;
// Запись в EEPROM закончена
volatile bool isTxCompleted = false;

/* USER CODE END Variables */
/* Definitions for logTask */
osThreadId_t logTaskHandle;
const osThreadAttr_t logTask_attributes = { .name = "logTask", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityLow, };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void taskBatMonitorFunc(void*);
void vTimerCallback(TimerHandle_t);
/* USER CODE END FunctionPrototypes */

void logTaskFunc(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  xTimer = xTimerCreate("xTimer", pdMS_TO_TICKS(1000), pdFALSE, (void*) 0, vTimerCallback);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of logTask */
  logTaskHandle = osThreadNew(logTaskFunc, NULL, &logTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  xTaskCreate(taskBatMonitorFunc, "taskBatMonitor", 128, NULL, osPriorityNormal, taskBatMonitorHandle);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
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
    osDelay(1);
  }
  /* USER CODE END logTaskFunc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void taskBatMonitorFunc(void *argument)
{
  for (;;) {
    // Ожидаем готовности преобразования напряжения на батарее
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // v_ref - это значение с канала Vrefint Channel, напряжение питания микроконтроллера
    double v_ref = INTERNAL_REF / adcBuffer[0];
    // v_bat - это значение с канала IN0, напряжение Li-Ion батареи
    double v_bat = adcBuffer[1] * v_ref / 4095 + V_DIODE;
    // Проверяем наличие внешнего питания
    if (HAL_GPIO_ReadPin(IN_GPIO_Port, IN_Pin) == GPIO_PIN_SET) {
      if (isСharged) {
        // Ранее (до перезагрузки) зарядка уже была запущена, поэтому включаем
        HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET);

        if (v_bat > MAX_CHARGE_V) {
          // Напряжение на батарее превышает верхний предел, завершаем зарядку
          HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);
          isСharged = false;
          chargeCycles++;
          if (isMemReady && isTxCompleted) {
            isTxCompleted = false;
            // Сохраняем статус зарядки в память и на 1 увеличиваем количество циклов зарядки
            // Заполняем буфер для передачи в EEPROM
            memBuffer[0] = (uint8_t) isСharged;
            decompose32into8(computeCRC32((uint32_t) isСharged), memBuffer, 1);

            memBuffer[5] = (chargeCycles >> 8) & 0xFF;
            memBuffer[6] = chargeCycles & 0xFF;
            decompose32into8(computeCRC32((uint32_t) chargeCycles), memBuffer, 7);

            // Запись в микросхему EEPROM не произодится, можем отправить данные
            if (HAL_I2C_Mem_Write_IT(&hi2c1, ADDRESS, 0x00, I2C_MEMADD_SIZE_16BIT, memBuffer, MEM_BUFFER_SIZE) != HAL_OK) {
              isMemReady = false;
            }
          }
        }
      } else {
        // Ранее (до перезагрузки) зарядка не проводилась, поэтому выключаем
        HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);

        if (v_bat < MIN_CHARGE_V) {
          // Зарядка выключена, но напряжение на батарее стало ниже нижнего предела
          // Включаем зарядку
          HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET);
          isСharged = true;
          // Количество циклов зарядки остаётся прежним
          if (isMemReady && isTxCompleted) {
            isTxCompleted = false;
            // Сохраняем статус зарядки в память, количество циклов зарядки оставляем без изменения
            // Заполняем буфер для передачи в EEPROM
            memBuffer[0] = (uint8_t) isСharged;
            decompose32into8(computeCRC32((uint32_t) isСharged), memBuffer, 1);

            if (HAL_I2C_Mem_Write_IT(&hi2c1, ADDRESS, 0x00, I2C_MEMADD_SIZE_16BIT, memBuffer, MEM_BUFFER_SIZE) != HAL_OK) {
              isMemReady = false;
            }
          }
        }
      }
    }
  }
}

void vTimerCallback(TimerHandle_t xTimer)
{
  // Запись в EEPROM закончена, устанавливаем флаг
  isTxCompleted = true;
}
/* USER CODE END Application */

