/*
 * init_device.h
 *
 *  Created on: Jan 4, 2024
 *      Author: Sergey
 */

#ifndef INC_INIT_DEVICE_H_
#define INC_INIT_DEVICE_H_

static HAL_StatusTypeDef init_MPU6050(I2C_HandleTypeDef *hi2c, uint16_t address) {
  // Регистры MPU6050
  uint16_t PWR_MGMT_1 = 0x6B;
  uint16_t GYRO_CONFIG = 0x1B;
  uint16_t ACCEL_CONFIG = 0x1C;
  // uint16_t SMPLRT_DIV = 0x19;
  // uint16_t INT_ENABLE = 0x38;

  uint8_t data;

  HAL_StatusTypeDef state = HAL_OK; // Результат выполнения запроса

  // Включение модуля MPU6050
  data = 0;
  state = HAL_I2C_Mem_Write(hi2c, address, PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY);
  if (state != HAL_OK)
    return state;

  // Конфигурируем фильтр нижних частот, регистр 26

  // Конфигурация гироскопа на ±500°/s
  data = 0x08;
  state = HAL_I2C_Mem_Write(hi2c, address, GYRO_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
  if (state != HAL_OK)
    return state;

  // Конфигурация акселерометра на ±8g
  data = 0x10;
  state = HAL_I2C_Mem_Write(hi2c, address, ACCEL_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
  if (state != HAL_OK)
    return state;

  /*  data = 0x07;
   state = HAL_I2C_Mem_Write(hi2c, address, SMPLRT_DIV, 1, &data, 1, HAL_MAX_DELAY);
   if (state != HAL_OK)
   return state;*/

/*  data = 0x01;
  state = HAL_I2C_Mem_Write(hi2c, address, INT_ENABLE, 1, &data, 1, HAL_MAX_DELAY);
  if (state != HAL_OK)
    return state;*/

  return HAL_OK;
}

#endif /* INC_INIT_DEVICE_H_ */
