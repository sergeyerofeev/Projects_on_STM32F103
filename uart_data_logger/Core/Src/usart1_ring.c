#include <usart1_ring.h>

extern UART_HandleTypeDef UART1;

/////////////////// USART /////////////////////
volatile uint16_t rx1_buffer_head = 0;
volatile uint16_t rx1_buffer_tail = 0;
uint8_t rx1_buffer[UART1_RX_BUFFER_SIZE] = { 0, };

void clear_uart1_buff()
{
  __HAL_UART_DISABLE_IT(&UART1, UART_IT_RXNE);
  rx1_buffer_head = 0;
  rx1_buffer_tail = 0;
  __HAL_UART_ENABLE_IT(&UART1, UART_IT_RXNE);
}

uint16_t uart1_available(void)
{
  return ((uint16_t) (UART1_RX_BUFFER_SIZE + rx1_buffer_head - rx1_buffer_tail)) % UART1_RX_BUFFER_SIZE;
}

uint8_t uart1_read(void)
{
  if (rx1_buffer_head == rx1_buffer_tail) {
    return 0;
  } else {
    uint8_t c = rx1_buffer[rx1_buffer_tail];
    rx1_buffer_tail = (uint16_t) (rx1_buffer_tail + 1) % UART1_RX_BUFFER_SIZE;
    return c;
  }
}
