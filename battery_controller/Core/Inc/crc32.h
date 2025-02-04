#ifndef INC_CRC32_H_
#define INC_CRC32_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>

uint32_t computeCRC32(uint32_t);

void decompose32into8(uint32_t, uint8_t[], uint16_t);

#endif /* INC_CRC32_H_ */
