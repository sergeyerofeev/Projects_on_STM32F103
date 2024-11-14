#ifndef USART_RING_H_
#define USART_RING_H_

#include "main.h"

#define MYUART huart1             // Работаем с UART1
#define UART_RX_BUFFER_SIZE 128   // Указываем размер приёмного буфера

uint16_t uart_available(void);
uint8_t uart_read(void);
void clear_uart_buff();

#endif /* USART_RING_H_ */
