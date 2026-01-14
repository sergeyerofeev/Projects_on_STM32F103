#ifndef DEFINITION_H_
#define DEFINITION_H_

// Данные с микросхемы датчика температуры max31865
typedef struct {
  float temp;     // Вычисленная температура
  float res;      // Текущее сопротивление датчика PT100
} varMAX31865_t;

// Данные принятые по USB
typedef struct {
  uint8_t reportID;
  uint8_t arrayKx[3];
} varReceivingUSB_t;

#endif /* DEFINITION_H_ */
