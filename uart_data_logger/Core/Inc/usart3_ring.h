#ifndef USART3_RING_H_
#define USART3_RING_H_

#include "main.h"

#define UART3 huart3             // Работаем с UART3
#define UART3_RX_BUFFER_SIZE 128   // Указываем размер приёмного буфера

uint16_t uart3_available(void);
uint8_t uart3_read(void);
void clear_uart3_buff();

#endif /* USART3_RING_H_ */
