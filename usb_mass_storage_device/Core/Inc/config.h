#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

// Стартовый адрес для записи во Flash память
#define STORAGE_ADDRESS 0x08009000
// Размер Flash памяти микроконтроллера, в страницах
#define FLASH_SIZE      64
// Количество занятых под саму программу страниц
#define PAGES_APP       35
// Количество страниц доступных для использования в качестве хранилища
#define STORAGE_SIZE    ( FLASH_SIZE - PAGES_APP )
// Размер одной страницы, в байтах
#define PAGE_SIZE       1024
#endif /* INC_CONFIG_H_ */
