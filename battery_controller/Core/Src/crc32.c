#include "crc32.h"

// Вычисление контрольной суммы встроенными средствами STM32
uint32_t computeCRC32(uint32_t value)
{
  // Сброс CRC (очистка аккумулятора)
  CRC->CR |= CRC_CR_RESET;
  // Передаём данные для вычисления CRC32
  CRC->DR = value;
  // Получаем вычисленное значение CRC32
  return CRC->DR;
}

// Разложение двойного слова на набор из четырёх байт
void decompose32into8(uint32_t data, uint8_t array[], uint16_t start)
{
  array[start] = (data >> 24) & 0xFF;
  array[start + 1] = (data >> 16) & 0xFF;
  array[start + 2] = (data >> 8) & 0xFF;
  array[start + 3] = data & 0xFF;
}
