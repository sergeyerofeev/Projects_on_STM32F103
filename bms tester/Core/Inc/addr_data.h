#ifndef INC_ADDR_DATA_H_
#define INC_ADDR_DATA_H_

#define TX_BUFSIZE 10                 // Размер буфера для отправки данных
#define STR_SIZE 20                   // Размер массива для строки с результатом

// Данные для BMS
struct  {
  const uint8_t addr;                 // Адрес данных в BMS
  const uint8_t rxBufSize;            // Размер буфера для приёма данных по UART RX
  const uint8_t dataTx[TX_BUFSIZE];   // Массив данных для передачи по UART TX
  char strRes[STR_SIZE];              // Итоговая строка с результатом
} bmsData[] = {
    // Версия прошивки BMS
    {0x17, 11, { 0x5A, 0xA5, 0x01, 0x3D, 0x22, 0x01, 0x17, 0x02, 0x85, 0xFF }, ""},
    // Текущее значение остаточной мощности, 0-100%
    {0x32, 11, { 0x5A, 0xA5, 0x01, 0x3D, 0x22, 0x01, 0x32, 0x02, 0x6A, 0xFF }, ""},
    // Текущая остаточная емкость, в mAh
    {0x31, 11, { 0x5A, 0xA5, 0x01, 0x3D, 0x22, 0x01, 0x31, 0x02, 0x6B, 0xFF }, ""},
    // Текущее напряжение, в V
    {0x34, 11, { 0x5A, 0xA5, 0x01, 0x3D, 0x22, 0x01, 0x34, 0x02, 0x68, 0xFF }, ""},
    // Вендор зашитый в BMS
    {0x60, 13, { 0x5A, 0xA5, 0x01, 0x3D, 0x22, 0x01, 0x60, 0x04, 0x3A, 0xFF }, ""}
};

#endif /* INC_ADDR_DATA_H_ */
