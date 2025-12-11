#ifndef MAX31865_H
#define MAX31865_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MAX31865_RREF               (430.0f)    // Сопротивление эталонного резистора, в Ом
#define MAX31865_RNOMINAL           (100.193f)  // Сопротивление датчика PT100, в Ом

#define MAX31865_2_WIRE             2
#define MAX31865_3_WIRE             3
#define MAX31865_4_WIRE             4

/* Адреса регистров */
#define MAX31865_CONFIG_REG         0x00
#define MAX31865_RTD_MSB_REG        0x01

/* Биты и маски */
#define MAX31865_BIAS_ON            0x80
#define MAX31865_BIAS_OFF           0x7F
#define MAX31865_ONE_SHOT           0x20
#define MAX31865_3WIRE_MODE         0x10
#define MAX31865_FILTER_50HZ        0x01

#define RTD_A                       (3.9083e-3f)
#define RTD_B                       (-5.775e-7f)

typedef const struct {
  void *spi;                        // SPI_HandleTypeDef
  void *cs_gpio;                    // GPIO_TypeDef
  uint16_t cs_pin;
} MAX31865_t;

/* Прототипы функций */
void Max31865_Init(MAX31865_t *max, int wire);
float Max31865_ReadTempC(MAX31865_t *max31865, float *res);

#ifdef __cplusplus
}
#endif

#endif /* MAX31865_H */
