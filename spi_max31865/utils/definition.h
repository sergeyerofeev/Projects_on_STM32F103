#ifndef DEFINITION_H_
#define DEFINITION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// Данные с микросхемы датчика температуры max31865
typedef struct {
  float temp;     // Вычисленная температура
  float res;      // Текущее сопротивление датчика PT100
} varMAX31865_t;

// Данные принятые по USB
typedef struct {
  volatile uint8_t reportID;
  volatile uint8_t kp;
  volatile uint8_t ki;
  volatile uint8_t kd;
} varReceivingUSB_t;

// Структура для преобразования float числа в массив int8_t
typedef struct {
  int8_t array[3];
} DataNum_t;

#ifdef __cplusplus
}
#endif

#endif /* DEFINITION_H_ */
