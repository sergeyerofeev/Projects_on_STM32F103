#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "main.h"
#include "cmsis_os.h"

#include <stdbool.h>
#include "i2c_er.h"
#include "crc32.h"
#include "iwdg.h"
#include "my_config.h"

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern IWDG_HandleTypeDef hiwdg;

extern SemaphoreHandle_t mutexMem;

extern TaskHandle_t batStatusHandle;
extern TaskHandle_t checkMemHandle;

// Буфер для измеренных значений с ADC
uint16_t adcBuffer[ADC_BUFFER_SIZE];
// Буфер для работы с EEPROM (чтение и запись)
uint8_t memBuffer[MEM_BUFFER_SIZE];
// Статус зарядки, true - батарея находится на зарядке
bool isСharged = false;
// Количество циклов зарядки батареи записанные в EEPROM
uint16_t chargeCycles;

// Функция задачи для однократного запуска и получения данных из EEPROM о статусе батареи
void vBatStatusFunc(void *argument)
{
  for (;;) {
    // Ожидаем захвата мьютекса
    xSemaphoreTake(mutexMem, portMAX_DELAY);
    for (;;) {
      // Считаем из EEPROM флаг статуса зарядки и количество циклов зарядки батареи
      if (HAL_I2C_Mem_Read(&hi2c1, ADDRESS, 0x00, I2C_MEMADD_SIZE_16BIT, memBuffer, MEM_BUFFER_SIZE, 100) == HAL_OK) {
        // Статус зарядки
        // Вычисляем контрольную сумму из полученных из EEPROM данных
        uint32_t dataCRC32 = computeCRC32((uint32_t) memBuffer[0]);
        // Преобразуем считанную из EEPROM массив контрольной суммы в двойное слово
        uint32_t saveCRC32 = (uint32_t) memBuffer[1] << 24 | memBuffer[2] << 16 | memBuffer[3] << 8 | memBuffer[4];
        if (dataCRC32 == saveCRC32) {
          // Если было сохранено 1, записываем true, если записан 0 - false
          isСharged = (memBuffer[0] == 1);
        } else {
          // В случае ошибки при записи в EEPROM разядку запрещаем
          isСharged = false;
        }

        // Количество циклов заряда батареи
        // Вычисляем контрольную сумму из полученных из EEPROM данных
        dataCRC32 = computeCRC32((uint32_t) memBuffer[5] << 8 | memBuffer[6]);
        // Преобразуем считанную из EEPROM массив контрольной суммы в двойное слово
        saveCRC32 = (uint32_t) memBuffer[7] << 24 | memBuffer[8] << 16 | memBuffer[9] << 8 | memBuffer[10];
        if (dataCRC32 == saveCRC32) {
          chargeCycles = (uint16_t) memBuffer[5] << 8 | memBuffer[6];
        } else {
          // В случае ошибок при записи в EEPROM количество циклов считаем равным 0
          chargeCycles = 0;
        }
        HAL_IWDG_Refresh(&hiwdg); // Сбрасываем IWDG
        // Данные успешно получены, выходим из внутреннего цикла
        break;
      } else {
        // Ошибка при доступе к EEPROM, в группе событий модифицируем биты
        // Мьютекс не освобждаем
        // Отправляем уведомление, в качестве значения передаём дескриптор текущей задачи
        xTaskNotify(checkMemHandle, (uint32_t ) batStatusHandle, eSetValueWithOverwrite);
        // Переводим задачу в режим ожидания
        vTaskSuspend(NULL);

        // После возврата снова попытаемся получить значение по сохранённому адресу,
        // то есть запускаем новую итерацию внутреннего цикла
      }
    }
    // Освобождаем семафор
    xSemaphoreGive(mutexMem);
    // Успешно получили все данные, удаляем задачу
    vTaskDelete(NULL);
  }
}

// Функция задачи проверки готовности EEPROM и перезапуска линиии I2C в случае сбоя
void vCheckMemFunc(void *argument)
{
  for (;;) {
    uint32_t handleValue = 0;
    xTaskNotifyWait(0, 0, &handleValue, portMAX_DELAY);
    // Отключаем питание на микросхему EEPROM, высокий уровень сигнала отключает PNP транзистор
    HAL_GPIO_WritePin(EE_GPIO_Port, EE_Pin, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(1000));
    for (;;) {
      HAL_IWDG_Refresh(&hiwdg); // Сбрасываем IWDG
      // Подаём питание на микросхему EEPROM, низкий уровень сигнала включает PNP транзистор
      HAL_GPIO_WritePin(EE_GPIO_Port, EE_Pin, GPIO_PIN_RESET);
      vTaskDelay(pdMS_TO_TICKS(1000));
      // Проверям готовность, 5 попыток, таймаут 100 мс
      if (HAL_I2C_IsDeviceReady(&hi2c1, ADDRESS, 5, 100) == HAL_OK) {
        // EEPROM успешно перезапустился, выходим из внутреннего цикла
        break;
      } else {
        // Если возвращаемое значение не равно HAL_OK
        // Отключаем питание микросхемы EEPROM
        HAL_GPIO_WritePin(EE_GPIO_Port, EE_Pin, GPIO_PIN_SET);
        // Запускаем процедуру переиницализации I2C1
        I2C_ClearBusyFlagErratum(&hi2c1, 100);
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
    }
    HAL_IWDG_Refresh(&hiwdg); // Сбрасываем IWDG
    // Возвращаем вызвавшую задачу из режима ожидания
    vTaskResume((TaskHandle_t) handleValue);
    taskYIELD(); // Передача управления другим задачам
  }
}

void taskBatMonitorFunc(void *argument)
{
  for (;;) {
    // Ожидаем готовности преобразования напряжения на батарее
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // Ожидаем захвата мьютекса
    xSemaphoreTake(mutexMem, portMAX_DELAY);
    // Получаем среднее от измеренных значений
    uint32_t adcVref = 0;
    uint32_t adcCh0 = 0;
    for (uint8_t i = 0; i < 16; i++) {
      if (i % 2 == 0) {
        adcVref += adcBuffer[i];
      } else {
        adcCh0 += adcBuffer[i];
      }
    }
    adcVref >>= 3; // Среднее значение для Rank 1
    adcCh0 >>= 3; // Среднее значение для Rank 2

    // v_ref - это значение с канала Vrefint Channel, напряжение питания микроконтроллера
    double v_ref = INTERNAL_REF / adcVref;
    // v_bat - это значение с канала IN0, напряжение Li-Ion батареи
    double v_bat = adcCh0 * v_ref / 4095 + V_DIODE;
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

          // Сохраняем статус зарядки в память и на 1 увеличиваем количество циклов зарядки
          // Заполняем буфер для передачи в EEPROM
          memBuffer[0] = (uint8_t) isСharged;
          decompose32into8(computeCRC32((uint32_t) isСharged), memBuffer, 1);

          memBuffer[5] = (chargeCycles >> 8) & 0xFF;
          memBuffer[6] = chargeCycles & 0xFF;
          decompose32into8(computeCRC32((uint32_t) chargeCycles), memBuffer, 7);
          memWrite();
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

          // Сохраняем статус зарядки в память, количество циклов зарядки оставляем без изменения
          // Заполняем буфер для передачи в EEPROM
          memBuffer[0] = (uint8_t) isСharged;
          decompose32into8(computeCRC32((uint32_t) isСharged), memBuffer, 1);
          memWrite();
        }
      }
    }
    // Освобождаем семафор
    xSemaphoreGive(mutexMem);
  }
}

// Внутренняя функция для записи в EEPROM
static void memWrite()
{
  for (;;) {
    if (HAL_I2C_Mem_Write_IT(&hi2c1, ADDRESS, 0x00, I2C_MEMADD_SIZE_16BIT, memBuffer, MEM_BUFFER_SIZE) == HAL_OK) {
      // Если запись прошла успешно, выходим из внутреннего цикла
      break;
    } else {
      // Ошибка при доступе к EEPROM, в группе событий модифицируем биты
      // Мьютекс не освобждаем
      // Отправляем уведомление, в качестве значения передаём дескриптор текущей задачи
      xTaskNotify(checkMemHandle, (uint32_t ) taskBatMonitorHandle, eSetValueWithOverwrite);
      // Переводим задачу в режим ожидания
      vTaskSuspend(NULL);
      // После возврата снова попытаемся записать значние по указанному адресу,
      // то есть запускаем новую итерацию внутреннего цикла
    }
  }
}
