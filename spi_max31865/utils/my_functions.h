#ifndef MY_FUNCTIONS_H_
#define MY_FUNCTIONS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

typedef struct {
  int integerPart;
  int fractional;
} DataNum_t;

// Функция преобразования числа типа float в строку, с округлением до одного знака после запятой
int floatToString(char buffer[], size_t buffer_size, float value);

// Функция разложения числа типа float на целую и дробные части типа int, с округлением до одного знака после запятой
DataNum_t transformFloat(float value);

// Преобразуем целую и дробную части числа в строку
int numToStr(char buffer[], size_t bufferSize, DataNum_t num);

#ifdef __cplusplus
}
#endif

#endif /* MY_FUNCTIONS_H_ */
