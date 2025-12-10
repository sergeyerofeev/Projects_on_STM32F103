#include "max31865.h"

#include <math.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

// Инлайн функции управления выбором микросхемы
static inline void max31865_cs_low(MAX31865_t *max) {
  HAL_GPIO_WritePin((GPIO_TypeDef*) max->cs_gpio, max->cs_pin, GPIO_PIN_RESET);
}

static inline void max31865_cs_high(MAX31865_t *max) {
  HAL_GPIO_WritePin((GPIO_TypeDef*) max->cs_gpio, max->cs_pin, GPIO_PIN_SET);
}

/* SPI чтение из регистра 8 бит */
static uint8_t Max31865_readRegister8(MAX31865_t *max, uint8_t addr) {
  uint8_t ret = 0;
  addr &= 0x7F; // Read Mode
  max31865_cs_low(max);
  HAL_SPI_Transmit((SPI_HandleTypeDef*) max->spi, &addr, 1, 100);
  HAL_SPI_Receive((SPI_HandleTypeDef*) max->spi, &ret, 1, 100);
  max31865_cs_high(max);
  return ret;
}

/* SPI чтение из двух регистров 16 бит */
static uint16_t Max31865_readRegister16(MAX31865_t *max, uint8_t addr) {
  uint8_t buffer[2] = { 0 };
  addr &= 0x7F; // Read Mode
  max31865_cs_low(max);
  HAL_SPI_Transmit(max->spi, &addr, 1, 100);
  HAL_SPI_Receive(max->spi, buffer, 2, 100);
  max31865_cs_high(max);
  return buffer[0] << 8 | buffer[1];
}

/* SPI запись данных в регистр */
static void Max31865_writeRegister(MAX31865_t *max, uint8_t addr, uint8_t data) {
  addr |= 0x80;  // Write Mode
  uint8_t txData[2] = { addr, data };
  max31865_cs_low(max);
  HAL_SPI_Transmit(max->spi, txData, 2, 100);
  max31865_cs_high(max);
}

/* Функция очистки регистра статуса от ошибок */
static void Max31865_clearFault(MAX31865_t *max) {
  uint8_t cfg = Max31865_readRegister8(max, MAX31865_CONFIG_REG);
  cfg = (cfg & ~0x2C) | 0x02;  // Сбросить D5,D3,D2, установить D1
  Max31865_writeRegister(max, MAX31865_CONFIG_REG, cfg);
}

/* Включаем BIAS */
static void Max31865_enableBias(MAX31865_t *max) {
  uint8_t cfg = Max31865_readRegister8(max, MAX31865_CONFIG_REG);
  Max31865_writeRegister(max, MAX31865_CONFIG_REG, cfg | MAX31865_BIAS_ON);
}

/* Отключаем BIAS */
static void Max31865_disableBias(MAX31865_t *max) {
  uint8_t cfg = Max31865_readRegister8(max, MAX31865_CONFIG_REG);
  Max31865_writeRegister(max, MAX31865_CONFIG_REG, cfg & ~MAX31865_BIAS_OFF);
}

/* Включаем режим однократного измерения */
static void Max31865_singleShot(MAX31865_t *max) {
  uint8_t cfg = Max31865_readRegister8(max, MAX31865_CONFIG_REG);
  Max31865_writeRegister(max, MAX31865_CONFIG_REG, cfg | MAX31865_ONE_SHOT);
}

/* Чтение RTD данных, 2 байта */
static uint16_t Max31865_readRTD(MAX31865_t *max) {
  Max31865_clearFault(max);
  Max31865_enableBias(max);
  vTaskDelay(pdMS_TO_TICKS(10));

  Max31865_singleShot(max);
  vTaskDelay(pdMS_TO_TICKS(65));

  uint16_t rtd = Max31865_readRegister16(max, MAX31865_RTD_MSB_REG) >> 1;
  Max31865_disableBias(max);
  return rtd;
}

/* Функция инициализации */
void Max31865_Init(MAX31865_t *max, int wire) {
  if (!max)
    return;

  Max31865_clearFault(max);

  uint8_t cfg = Max31865_readRegister8(max, MAX31865_CONFIG_REG);
  cfg = (wire == MAX31865_3_WIRE) ? (cfg | MAX31865_3WIRE_MODE) : (cfg & ~MAX31865_3WIRE_MODE);
  cfg |= MAX31865_FILTER_50HZ;
  Max31865_writeRegister(max, MAX31865_CONFIG_REG, cfg);
}

/* Преобразование значения АЦП в температуру */
float Max31865_ReadTempC(MAX31865_t *max, float *res) {
  static const float Z1 = -RTD_A;
  static const float Z2 = RTD_A * RTD_A - 4.0f * RTD_B;
  static const float Z3 = 4.0f * RTD_B / MAX31865_RNOMINAL;
  static const float Z4 = 2.0f * RTD_B;
  static const float RTD_CONV_FACTOR = MAX31865_RREF / 32768.0f;

  float Rt = (float) Max31865_readRTD(max) * RTD_CONV_FACTOR;

  // Получаем текущее сопротивление
  *res = Rt;

  float temp = Z2 + (Z3 * Rt);
  temp = (sqrt(temp) + Z1) / Z4;

  if (temp >= 0)
    return temp;

  // Вычисление для отрицательной температуры
  Rt /= MAX31865_RNOMINAL;
  Rt *= 100;

  float rpoly = Rt;

  temp = -242.02;
  temp += 2.2228 * rpoly;
  rpoly *= Rt; // ^2
  temp += 2.5859e-3 * rpoly;
  rpoly *= Rt; // ^3
  temp -= 4.8260e-6 * rpoly;
  rpoly *= Rt; // ^4
  temp -= 2.8183e-8 * rpoly;
  rpoly *= Rt; // ^5
  temp += 1.5243e-10 * rpoly;

  return temp;
}
