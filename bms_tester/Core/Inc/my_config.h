#ifndef INC_MY_CONFIG_H_
#define INC_MY_CONFIG_H_

#include <stdint.h>

// Переменные для работы с UART
#define TX_BUFSIZE 10                 // Размер буфера для отправки данных
#define STR_SIZE 20                   // Размер массива для строки с результатом
#define RX_BUFSIZE 13                 // Максимальный размер буфера для приёма данных

// ---------------------------------------------------------------------------------------

// Структура данных для работы с BMS
typedef struct  {
  const uint8_t addr;                 // Адрес данных в BMS
  const uint8_t rxBufSize;            // Размер буфера для приёма данных по UART RX
  const uint8_t dataTx[TX_BUFSIZE];   // Массив данных для передачи по UART TX
  char strRes[STR_SIZE];              // Итоговая строка с результатом
} BmsData;

// ---------------------------------------------------------------------------------------

// Поиск названия вендора по коду
typedef const struct {
    uint32_t code;
    char *name;
} CodeMapping;

const char* code_to_vendor(uint32_t code);

#endif /* INC_MY_CONFIG_H_ */
