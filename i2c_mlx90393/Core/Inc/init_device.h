/*
 * init_MLX90393.h
 *
 *  Created on: 28 дек. 2023 г.
 *      Author: Sergey
 */

#ifndef INC_INIT_DEVICE_H_
#define INC_INIT_DEVICE_H_

static HAL_StatusTypeDef init_MLX90393(I2C_HandleTypeDef *hi2c, uint16_t address) {
  uint8_t pTxData[4] = { 0 };
  uint8_t pRxData[3] = { 0 };
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

  HAL_Delay(100);

/*  // Записываем в регистр 0x00, значение GAIN_SEL = 100, HALLCONF = 1100
  pTxData[0] = 0x60;
  // Записываем сначала старший байт, затем младщий
  pTxData[1] = 0x00;
  pTxData[2] = 0x4C;
  pTxData[3] = 0x00 << 2;
  state = HAL_I2C_Master_Transmit(hi2c, address, pTxData, 4, HAL_MAX_DELAY);
  if (state != HAL_OK)
    return state;
  state = HAL_I2C_Master_Receive(hi2c, address, pRxData, 1, HAL_MAX_DELAY);
  if (state != HAL_OK)
    return state;*/

  // Записываем в регистр 0x02, значение RES_XYZ = 000101, DIG_FILT = 111
  pTxData[0] = 0x60;
  // Записываем сначала старший байт, затем младщий
  pTxData[1] = 0x00;
  pTxData[2] = 0xBC;
  pTxData[3] = 0x02 << 2;
  state = HAL_I2C_Master_Transmit(hi2c, address, pTxData, 4, HAL_MAX_DELAY);
  if (state != HAL_OK)
    return state;
  state = HAL_I2C_Master_Receive(hi2c, address, pRxData, 1, HAL_MAX_DELAY);
  if (state != HAL_OK)
    return state;

  HAL_Delay(100);

  return HAL_OK;
}

#endif /* INC_INIT_DEVICE_H_ */
