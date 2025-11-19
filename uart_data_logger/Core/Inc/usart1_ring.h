#ifndef USART1_RING_H_
#define USART1_RING_H_

#include "main.h"

#define UART1 huart1                // Работаем с UART1
#define UART1_RX_BUFFER_SIZE 128    // Указываем размер приёмного буфера

uint8_t uart1_read(void);
void clear_uart1_buff();

#endif /* USART1_RING_H_ */
