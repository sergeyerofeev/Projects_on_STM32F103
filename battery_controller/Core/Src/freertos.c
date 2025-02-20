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
/* Definitions for taskBatMonitor */
osThreadId_t taskBatMonitorHandle;
const osThreadAttr_t taskBatMonitor_attributes = {
  .name = "taskBatMonitor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for taskBatCharging */
osThreadId_t taskBatChargingHandle;
const osThreadAttr_t taskBatCharging_attributes = {
  .name = "taskBatCharging",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for timerEEPROM */
osTimerId_t timerEEPROMHandle;
const osTimerAttr_t timerEEPROM_attributes = {
  .name = "timerEEPROM"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void batMonitor(void *argument);
void batCharging(void *argument);
void vTimerCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* Create the timer(s) */
  /* creation of timerEEPROM */
  timerEEPROMHandle = osTimerNew(vTimerCallback, osTimerOnce, NULL, &timerEEPROM_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of taskBatMonitor */
  taskBatMonitorHandle = osThreadNew(batMonitor, NULL, &taskBatMonitor_attributes);

  /* creation of taskBatCharging */
  taskBatChargingHandle = osThreadNew(batCharging, NULL, &taskBatCharging_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_batMonitor */
/**
  * @brief  Function implementing the taskBatMonitor thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_batMonitor */
void batMonitor(void *argument)
{
  /* USER CODE BEGIN batMonitor */
  /* Infinite loop */
  for(;;)
  {
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
  /* USER CODE END batMonitor */
}

/* USER CODE BEGIN Header_batCharging */
/**
* @brief Function implementing the taskBatCharging thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_batCharging */
void batCharging(void *argument)
{
  /* USER CODE BEGIN batCharging */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END batCharging */
}

/* vTimerCallback function */
void vTimerCallback(void *argument)
{
  /* USER CODE BEGIN vTimerCallback */
  isTxCompleted = true;
  /* USER CODE END vTimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

