/*
 * init_MLX90393.h
 *
 *  Created on: 28 дек. 2023 г.
 *      Author: Sergey
 */

#ifndef INC_INIT_DEVICE_H_
#define INC_INIT_DEVICE_H_

static HAL_StatusTypeDef init_MLX90393(I2C_HandleTypeDef *hi2c, uint16_t address) {
  uint8_t pTxData[1] = { 0 };
  uint8_t pRxData[1] = { 0 };
  HAL_StatusTypeDef state = HAL_OK; // Результат выполнения запроса

  pTxData[0] = 0x80; // Выполняем команду Exit
  state = HAL_I2C_Master_Transmit(hi2c, address, pTxData, 1, HAL_MAX_DELAY);
  if (state != HAL_OK)
    return state;
  state = HAL_I2C_Master_Receive(hi2c, address, pRxData, 1, HAL_MAX_DELAY);
  if (state != HAL_OK)
    return state;

  pTxData[0] = 0xF0; // Выполняем команду Reset
  state = HAL_I2C_Master_Transmit(hi2c, address, pTxData, 1, HAL_MAX_DELAY);
  if (state != HAL_OK)
    return state;
  state = HAL_I2C_Master_Receive(hi2c, address, pRxData, 1, HAL_MAX_DELAY);
  if (state != HAL_OK)
    return state;

  // Запускаем непрерывный режим передав конфигурацию 0x1E
  pTxData[0] = 0x1E;
  state = HAL_I2C_Master_Transmit(hi2c, address, pTxData, 1, HAL_MAX_DELAY);
  if (state != HAL_OK)
    return state;
  state = HAL_I2C_Master_Receive(hi2c, address, pRxData, 1, HAL_MAX_DELAY);
  if (state != HAL_OK)
    return state;

  return HAL_OK;
}

#endif /* INC_INIT_DEVICE_H_ */
