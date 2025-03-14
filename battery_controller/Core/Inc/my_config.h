#ifndef INC_MY_CONFIG_H_
#define INC_MY_CONFIG_H_

// Два канала опрашиваем по одному разу каждый
#define ADC_BUFFER_SIZE 16
// Величина 4095 * 1.2 = 4914.0 используемая для получение v_ref
#define INTERNAL_REF 4914.0
// Период запуска программного таймера, 1 минута
#define ADC_PERIOD 60000

// Падение напряжение на последовательно подключенном диоде
#define V_DIODE 0.6
// Минимальное напряжение на батарее, при котором запускаем зарядку
#define MIN_CHARGE_V 3.2
// Максимальное напряжение на батарее, при котором зарядку прекращаем
#define MAX_CHARGE_V 3.8

// Адрес микросхемы EEPROM на линии I2C, 7 бит сдвинутых влево на единицу
#define ADDRESS (0x50 << 1)
// Размер массива для работы с EEPROM
#define MEM_BUFFER_SIZE 11

#endif /* INC_MY_CONFIG_H_ */
