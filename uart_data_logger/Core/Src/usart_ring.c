#include "usart_ring.h"

extern UART_HandleTypeDef MYUART;

/////////////////// USART /////////////////////
volatile uint16_t rx_buffer_head = 0;
volatile uint16_t rx_buffer_tail = 0;
uint8_t rx_buffer[UART_RX_BUFFER_SIZE] = { 0, };

void clear_uart_buff()
{
  __HAL_UART_DISABLE_IT(&MYUART, UART_IT_RXNE);
  rx_buffer_head = 0;
  rx_buffer_tail = 0;
  __HAL_UART_ENABLE_IT(&MYUART, UART_IT_RXNE);
}

uint16_t uart_available(void)
{
  return ((uint16_t) (UART_RX_BUFFER_SIZE + rx_buffer_head - rx_buffer_tail)) % UART_RX_BUFFER_SIZE;
}

uint8_t uart_read(void)
{
  if (rx_buffer_head == rx_buffer_tail) {
    return 0;
  } else {
    uint8_t c = rx_buffer[rx_buffer_tail];
    rx_buffer_tail = (uint16_t) (rx_buffer_tail + 1) % UART_RX_BUFFER_SIZE;
    return c;
  }
}
