#include "my_functions.h"

// Функция разложения числа типа float на целую и дробные части типа int, с округлением до одного знака после запятой
DataNum_t transformFloat(float value) {
  // Обработка отрицательных чисел
  const int negative = value < 0;
  if (negative)
    value = -value;

  int integerPart = (int)value;
  int fractional = (int)((value - integerPart + 0.001) * 100);

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

  return (DataNum_t){.integerPart = integerPart, .fractional = fractional};
}


// Преобразуем целую и дробную части числа в строку
int numToStr(char buffer[], size_t bufferSize, DataNum_t num) {
  // Возвращаем фактическую длину отформатированной строки без символа '\0'
  return snprintf(buffer, bufferSize, " %d,%d ", num.integerPart, num.fractional);
}
