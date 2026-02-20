#include <stdio.h>
#include "util.h"

// Функция преобразования числа типа float в строку, с округлением до одного знака после запятой
int floatToString(char buffer[], size_t buffer_size, float value) {
  // Обработка отрицательных чисел
  const int negative = value < 0;
  if (negative)
    value = -value;

  int integerPart = (int) value;
  int fractional = (int) ((value - integerPart + 0.001) * 100);

  if (fractional % 10 == 9) {
    integerPart++;
    fractional = 0;
  } else if (fractional % 10 >= 5) {
    fractional = fractional / 10 + 1;
    if (fractional % 10 == 0) {
      integerPart++;
      fractional = 0;
    }
  } else {
    fractional /= 10;
  }

  if (negative)
    integerPart = -integerPart;

  // Возвращаем фактическую длину отформатированной строки без символа '\0'
  return snprintf(buffer, buffer_size, " %d,%d ", integerPart, fractional);
}

