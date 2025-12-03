#ifndef MAX31865_H
#define MAX31865_H

#include "main.h"
#include <math.h>

#define MAX31865_RREF              (430.0f)  // Сопротивление эталонного резистора, в Ом
#define MAX31865_RNOMINAL          (100.0f)  // Сопротивление датчика PT100, в Ом

#define MAX31865_2_WIRE             2
#define MAX31865_3_WIRE             3
#define MAX31865_4_WIRE             4

/* Адреса регистров */
#define MAX31865_CONFIG_REG        0x00
#define MAX31865_RTD_MSB_REG       0x01

/* Биты и маски */
#define MAX31865_BIAS_ON           0x80
#define MAX31865_BIAS_OFF          0x7F
#define MAX31865_ONE_SHOT          0x20
#define MAX31865_3WIRE_MODE        0x10
#define MAX31865_FILTER_50HZ       0x01

#define RTD_A                      (3.9083e-3f)
#define RTD_B                      (-5.775e-7f)

#define MAX31865_CS_LOW(max)       HAL_GPIO_WritePin((max)->cs_gpio, (max)->cs_pin, GPIO_PIN_RESET)
#define MAX31865_CS_HIGH(max)      HAL_GPIO_WritePin((max)->cs_gpio, (max)->cs_pin, GPIO_PIN_SET)

/* MAX31865 Структура данных */
typedef struct {
  SPI_HandleTypeDef *spi;          // SPI handle
  GPIO_TypeDef *cs_gpio;           // GPIO port for Chip Select
  uint16_t cs_pin;                 // GPIO pin for Chip Select
} MAX31865_t;

/* Прототипы функций */
void Max31865_Init(MAX31865_t *max, int wire);
float Max31865_ReadTempC(MAX31865_t *max31865);

#endif /* MAX31865_H */
