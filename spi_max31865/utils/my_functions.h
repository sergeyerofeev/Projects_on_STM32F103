#ifndef MY_FUNCTIONS_H_
#define MY_FUNCTIONS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "definition.h"

// Функция преобразования числа типа float в строку, с округлением до одного знака после запятой
int floatToString(char buffer[], size_t buffer_size, float value);

// Функция разложения числа типа float в массив int8_t, с округлением до одного знака после запятой
DataNum_t transformFloat(float value);

#ifdef __cplusplus
}
#endif

#endif /* MY_FUNCTIONS_H_ */
