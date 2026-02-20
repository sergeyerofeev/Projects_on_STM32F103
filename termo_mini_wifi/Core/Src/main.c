/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "max31865.h"
#include "util.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint32_t last_run;
  uint32_t interval;
  void (*callback)(void);
} TimerTask;

// Структура для преобразования float числа в массив int8_t
typedef struct {
  int8_t array[3];
} DataNum_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Тайминги (в мс)
#define DT 500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;

uint8_t reportID = 2;

MAX31865_t hMAX31865 = { .spi = &hspi1, .cs_gpio = CS_GPIO_Port, .cs_pin = CS_Pin };
float temp;

float kp = 30.0f;
float ki = 0.2f;

const float dt = DT / 1000.0f;
float setPoint = 220.0f;          // Целевая температура
float deltaPerCycle = 4.0f * dt;  // Скорость нагрева 2°C/цикл или 4°C/s
    /* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// Прототип функции задачи ПИД регулятора
static void _task1(void);
// Прототип функции задачи отправки данных по Wi-Fi
static void _task2(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
TimerTask tasks[] = { { 0, DT, _task1 }, { 0, 1000, _task2 } };
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  Max31865_Init(&hMAX31865, MAX31865_2_WIRE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    uint32_t current = HAL_GetTick();

    for (int i = 0; i < 2; i++) {
      if (current - tasks[i].last_run >= tasks[i].interval) {
        tasks[i].last_run = current;
        tasks[i].callback();
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }

  /** Enables the Clock Security System
   */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
// Функция задачи ПИД регулятора
static void _task1(void) {
  static float deltaTemp = 0.0f;        // Разность между уставкой setPoint и текущй температурой
  static float boostKp = 0.0f;          // На данном участке Kp увеличивается
  static float limitedSetpoint = 0.0f;  // Ограниченная уставка
  static float deltaSetpoint = 0.0f;

  static const float minOut = 0.0f;
  static const float maxOut = 4800.0f;
  //float maxOut = (float) htim4.Init.Period + 1;

  static float errorCurrent = 0.0f;
  static float errorIntegral = 0.0f;

  static float result = 0.0f;

  // Измеряем температуру
  temp = Max31865_ReadTempC(&hMAX31865);

  if (reportID == 1) {
    // Проверяем, активен ли выход канала 3, если нет, то запускаем ШИМ
    if ((TIM4->CCER & TIM_CCER_CC3E) == 0) {
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
      limitedSetpoint = temp;  // Начинаем с текущей температуры
    }

    // Вычисляем разность между целевой и ограниченной уставками
    deltaSetpoint = setPoint - limitedSetpoint;

    // Применение ограничения скорости
    if (deltaSetpoint > deltaPerCycle) {
      // Нагревание, с каждым циклом увеличиваем limitedSetpoint на 2°C/цикл
      limitedSetpoint += deltaPerCycle;
    } else if (deltaSetpoint < -deltaPerCycle) {
      // Остывание, с каждым циклом уменьшаем limited_setpoint на 2°C/цикл
      limitedSetpoint -= deltaPerCycle;
    } else {
      // Температура достигла уставки
      limitedSetpoint = setPoint;
    }

    // Вычисление ошибки, разница между уставкой и измеренной температурой
    errorCurrent = limitedSetpoint - temp;

    errorIntegral += errorCurrent * dt * ki;
    // Задаём размер участка на котором Kp увеличивается
    boostKp = setPoint / 5.0f;
    deltaTemp = setPoint - temp;
    if (deltaTemp < boostKp && deltaTemp > 0) {
      // Увеличение коэффициента kp включаем только при приближении к setPoint, снизу
      result = errorCurrent * kp * 2 + errorIntegral;
    } else {
      result = errorCurrent * kp + errorIntegral;
    }

    // Проверяем насыщение выхода
    if (result >= maxOut) {
      result = maxOut;
    } else if (result <= minOut) {
      result = minOut;
    }

    htim4.Instance->CCR3 = (uint16_t) result;
  } else if (reportID == 2) {
    // Проверяем, активен ли выход канала 3, если да, то останавливаем ШИМ
    if (TIM4->CCER & TIM_CCER_CC3E) {
      HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
      htim4.Instance->CCR3 = 0;
      errorIntegral = 400.0f;
    }
  }
}
// Функция задачи отправки данных по Wi-Fi
static void _task2(void) {
  static uint16_t timeCount = 0;
  static char strFloat[10];       // Строка для преобразованного значения типа float
  static char strOut[10];         // Строка для передачи по UART
  static int strLen = 0;          // Длина передаваемой строки

  // Отправляем измененную температуру и время в секундах, по линии UART
  if (reportID == 1) {
    strLen = snprintf(strOut, sizeof(strOut), "%d %d", (int) temp, timeCount);
    timeCount++;
  } else if (reportID == 2) {
    if (timeCount != 0)
      timeCount = 0;
    // Преобразуем значение температуры (тип float) в строку
    floatToString(strFloat, sizeof(strFloat), temp);
    strLen = snprintf(strOut, sizeof(strOut), "%s %d", strFloat, timeCount);
  }
  HAL_UART_Transmit_IT(&huart1, (uint8_t*) strOut, strLen);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
