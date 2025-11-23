#ifndef USART1_RING_H_
#define USART1_RING_H_

#include "main.h"

#define UART1 huart1                // Работаем с UART1
#define UART1_RX_BUFFER_SIZE 128    // Указываем размер приёмного буфера

uint16_t uart1_available(void);
uint8_t uart1_read(void);

#endif /* USART1_RING_H_ */
