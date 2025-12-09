#ifndef MY_FUNCTIONS_H_
#define MY_FUNCTIONS_H_

#include <stdio.h>

typedef struct {
  int integerPart;
  int fractional;
} DataNum_t;

// Функция разложения числа типа float на целую и дробные части типа int, с округлением до одного знака после запятой
DataNum_t transformFloat(float value);

// Преобразуем целую и дробную части числа в строку
int numToStr(char buffer[], size_t bufferSize, DataNum_t num);

#endif /* MY_FUNCTIONS_H_ */
