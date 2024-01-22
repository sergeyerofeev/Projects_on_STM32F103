/*
 * mlx90393.h
 *
 *  Created on: 28 дек. 2023 г.
 *      Author: Sergey
 */

#ifndef INC_MLX90393_H_
#define INC_MLX90393_H_

enum {
  mlx90393_SB = 0x10,  /**< Start burst mode. */
  mlx90393_SW = 0x20,  /**< Start wakeup on change mode. */
  mlx90393_SM = 0x30,  /**> Start single-meas mode. */
  mlx90393_RM = 0x40,  /**> Read measurement. */
  mlx90393_RR = 0x50,  /**< Read register. */
  mlx90393_WR = 0x60,  /**< Write register. */
  mlx90393_EX = 0x80,  /**> Exit moode. */
  mlx90393_HR = 0xD0,  /**< Memory recall. */
  mlx90393_HS = 0x70,  /**< Memory store. */
  mlx90393_RT = 0xF0,  /**< Reset. */
  mlx90393_NOP = 0x00, /**< NOP. */
};

/** Gain settings for 0x00 register. */
enum {
  mlx90393_GAIN_5X = (0x00),
  mlx90393_GAIN_4X,
  mlx90393_GAIN_3X,
  mlx90393_GAIN_2_5X,
  mlx90393_GAIN_2X,
  mlx90393_GAIN_1_67X,
  mlx90393_GAIN_1_33X,
  mlx90393_GAIN_1X
};

/** Resolution settings for 0x02 register. */
enum {
  mlx90393_RES_0,
  mlx90393_RES_1,
  mlx90393_RES_2,
  mlx90393_RES_3,
};

/** Digital filter settings for 0x02 register. */
enum {
  mlx90393_FILTER_0,
  mlx90393_FILTER_1,
  mlx90393_FILTER_2,
  mlx90393_FILTER_3,
  mlx90393_FILTER_4,
  mlx90393_FILTER_5,
  mlx90393_FILTER_6,
  mlx90393_FILTER_7,
};

/** Oversampling settings for 0x02 register. */
enum {
  mlx90393_OSR_0,
  mlx90393_OSR_1,
  mlx90393_OSR_2,
  mlx90393_OSR_3,
};



static HAL_StatusTypeDef init_MLX90393(I2C_HandleTypeDef *hi2c, uint16_t address) {
  // Значение записываемое в выбранные регистр mlx90393
  uint16_t regData = 0;

  // Значение регистра статуса, единственный байт в ответе
  uint8_t pTxData[4] = { 0 };
  uint8_t pRxData[1] = { 0 };

  HAL_StatusTypeDef state = HAL_OK; // Результат выполнения запроса

  // Выполняем команду Exit
  pTxData[0] = mlx90393_EX;
  state = HAL_I2C_Master_Transmit(hi2c, address, pTxData, 1, HAL_MAX_DELAY);
  if (state != HAL_OK)
    return state;
  state = HAL_I2C_Master_Receive(hi2c, address, pRxData, 1, HAL_MAX_DELAY);
  if (state != HAL_OK)
    return state;

  // Выполняем команду Reset
  pTxData[0] = mlx90393_RT;
  state = HAL_I2C_Master_Transmit(hi2c, address, pTxData, 1, HAL_MAX_DELAY);
  if (state != HAL_OK)
    return state;
  state = HAL_I2C_Master_Receive(hi2c, address, pRxData, 1, HAL_MAX_DELAY);
  if (state != HAL_OK)
    return state;

  HAL_Delay(100);

/*  // Записываем в регистр 0x00, значение GAIN_SEL = 111, HALLCONF = 1100
  regData = mlx90393_GAIN_1X << 4 | 0xC;
  pTxData[0] = mlx90393_WR;
  // Записываем сначала старший байт, затем младщий
  pTxData[1] = regData >> 8;
  pTxData[2] = regData;
  pTxData[3] = 0x00 << 2;
  state = HAL_I2C_Master_Transmit(hi2c, address, pTxData, 4, HAL_MAX_DELAY);
  if (state != HAL_OK)
    return state;
  state = HAL_I2C_Master_Receive(hi2c, address, pRxData, 1, HAL_MAX_DELAY);
  if (state != HAL_OK)
    return state;*/

  // Записываем в регистр 0x02, значение RES_XYZ = 111111 (Внимание порядок Z,Y,X), DIG_FILT = 111
  regData = mlx90393_RES_3 << 9 | mlx90393_RES_3 << 7 | mlx90393_RES_3 << 5 | mlx90393_FILTER_7 << 2;
  pTxData[0] = mlx90393_WR;
  // Записываем сначала старший байт, затем младщий
  pTxData[1] = regData >> 8;
  pTxData[2] = regData;
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

#endif /* INC_MLX90393_H_ */
