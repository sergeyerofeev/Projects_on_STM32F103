#include <stdio.h>
#include <stddef.h>
#include "my_config.h"

// Массив структур данных для работы с BMS
BmsData bmsData[] = {
    // Версия прошивки BMS
    {0x17, 11, { 0x5A, 0xA5, 0x01, 0x3D, 0x22, 0x01, 0x17, 0x02, 0x85, 0xFF }, ""},
    // Текущее значение остаточной мощности, 0-100%
    {0x32, 11, { 0x5A, 0xA5, 0x01, 0x3D, 0x22, 0x01, 0x32, 0x02, 0x6A, 0xFF }, ""},
    // Текущая остаточная емкость, в mAh
    {0x31, 11, { 0x5A, 0xA5, 0x01, 0x3D, 0x22, 0x01, 0x31, 0x02, 0x6B, 0xFF }, ""},
    // Текущее напряжение, в V
    {0x34, 11, { 0x5A, 0xA5, 0x01, 0x3D, 0x22, 0x01, 0x34, 0x02, 0x68, 0xFF }, ""},
    // Вендор зашитый в BMS
    {0x60, RX_BUFSIZE, { 0x5A, 0xA5, 0x01, 0x3D, 0x22, 0x01, 0x60, 0x04, 0x3A, 0xFF }, ""}
};

// Количество элементов массива
const size_t bmsDataCount = sizeof(bmsData) / sizeof(bmsData[0]);
// ---------------------------------------------------------------------------------------

// Массив структур для поиска вендора по коду
CodeMapping mappings[] = {
    {0x4E344007, "Urent"},
    {0x4E34300A, "Teltonika"},
    {0x4E34300E, "Woosh"},
    {0x4E34400C, "Borzhu"},
    {0x4E344003, "Marvel"},
    {0x4E34300C, "Voi"},
    {0x4E33000D, "Bolt"},
    {0x4E34100C, "Spin"},
    {0x4E344005, "Molniya"},
    {0x4E34400A, "Swing"},
};

char str[STR_SIZE] = { 0, };

const char* code_to_vendor(uint32_t code) {
  size_t count = sizeof(mappings) / sizeof(mappings[0]);

  for (size_t i = 0; i < count; i++) {
    if (mappings[i].code == code) {
      return mappings[i].name;
    }
  }
  // Если имя не найдено возвращаем код в строковом представлении
  snprintf(str, STR_SIZE, "%lX", code);

  return str; // если код не найден
}

// ---------------------------------------------------------------------------------------
