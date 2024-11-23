#include <usart3_ring.h>

extern UART_HandleTypeDef UART3;

/////////////////// USART /////////////////////
volatile uint16_t rx3_buffer_head = 0;
volatile uint16_t rx3_buffer_tail = 0;
uint8_t rx3_buffer[UART3_RX_BUFFER_SIZE] = { 0, };

void clear_uart3_buff()
{
  __HAL_UART_DISABLE_IT(&UART3, UART_IT_RXNE);
  rx3_buffer_head = 0;
  rx3_buffer_tail = 0;
  __HAL_UART_ENABLE_IT(&UART3, UART_IT_RXNE);
}

uint16_t uart3_available(void)
{
  return ((uint16_t) (UART3_RX_BUFFER_SIZE + rx3_buffer_head - rx3_buffer_tail)) % UART3_RX_BUFFER_SIZE;
}

uint8_t uart3_read(void)
{
  if (rx3_buffer_head == rx3_buffer_tail) {
    return 0;
  } else {
    uint8_t c = rx3_buffer[rx3_buffer_tail];
    rx3_buffer_tail = (uint16_t) (rx3_buffer_tail + 1) % UART3_RX_BUFFER_SIZE;
    return c;
  }
}
