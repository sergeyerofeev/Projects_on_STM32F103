#ifndef INC_MY_CONFIG_H_
#define INC_MY_CONFIG_H_

// Два канала опрашиваем по одному разу каждый
#define ADC_BUFFER_SIZE 16
// Величина 4095 * 1.2 = 4914.0 используемая для получение v_ref
#define INTERNAL_REF 4914.0
// Период запуска программного таймера, 1 минута
#define ADC_PERIOD 60000
// Период запуска тамера для сброса IWDT 4 секунды
#define IWDT_PERIOD 4000

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
/*
// Константы для работы с пьезокерамическим излучателем
u16 GL_BuzzerAllNotes[] = {
	261, 277, 294, 311, 329, 349, 370, 392, 415, 440, 466, 494,
	523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988,
	1046, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976,
	2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951,
	4186, 4434, 4699, 4978, 5274, 5588, 5920, 6272, 6645, 7040, 7459, 7902};

#define OCTAVE_ONE_START_INDEX		(0)
#define OCTAVE_TWO_START_INDEX		(OCTAVE_ONE_START_INDEX + 12)
#define OCTAVE_THREE_START_INDEX	(OCTAVE_TWO_START_INDEX + 12)
#define OCTAVE_FOUR_START_INDEX		(OCTAVE_THREE_START_INDEX + 12)
#define OCTAVE_FIVE_START_INDEX		(OCTAVE_FOUR_START_INDEX + 12)

#define BUZZER_DEFAULT_FREQ			(4186) //C8 - 5th octave "Do"
#define BUZZER_DEFAULT_DURATION		(20) //20ms
#define BUZZER_VOLUME_MAX			(10)
#define BUZZER_VOLUME_MUTE			(0)

#endif /* INC_MY_CONFIG_H_ */
*/
