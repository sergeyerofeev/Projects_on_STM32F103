/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_er.h"
#include "init_device.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_Address 0x68 << 1
#define RAD_TO_DEG 57.295779513082320876798154814105
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
HAL_StatusTypeDef state = HAL_OK; // Результат выполнения запроса

// Переменная итератор для запроса на получение данных каждые 20 мс
volatile uint8_t uiTicksCNT = 0;

// Буффер для raw данных с датчика MPU6050
uint8_t pRxData[20];

// Буффер для считанных данных
int16_t pData[6];

// Смещения для гироскопа
int16_t gx_offset = 0;
int16_t gy_offset = 0;
int16_t gz_offset = 0;

// Флаг окончания калибровки
uint8_t isCalibrate = 0;
// Флаг готовности выполнить вычисления, 1 раз в 20 мс
volatile uint8_t isCalculation = 0;

float roll = 0;
float pitch = 0;
float yaw = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void MPU6050_Calibrate(void);
void MPU6050_Get_All_Data(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  __HAL_RCC_I2C1_CLK_ENABLE();
  HAL_Delay(100);
  __HAL_RCC_I2C1_FORCE_RESET();
  HAL_Delay(100);
  __HAL_RCC_I2C1_RELEASE_RESET();
  HAL_Delay(100);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  // Проверяем готовность I2C и MLX90393
  state = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t) MPU6050_Address, 1, HAL_MAX_DELAY);
  if (state == HAL_BUSY) {
    // При данном флаге вызываем функцию переинициализации I2C1
    I2C_ClearBusyFlagErratum(&hi2c1, 1000);
  }

  // Производим инициализацию MPU6050
  state = init_MPU6050(&hi2c1, (uint16_t) MPU6050_Address);
  if (state != HAL_OK) {
    // Если при инициализации произошла ошибка, перезапускаем микроконтроллер
    __set_FAULTMASK(1); // Запрещаем все маскируемые прерывания
    NVIC_SystemReset(); // Программный сброс
  }

  // Калибруем MPU6050
  MPU6050_Calibrate();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */
    if (isCalculation) {
      isCalculation = 0;

      // Запрос на получение данных каждые 20 мс
      MPU6050_Get_All_Data();
      // Преобразуем данные
      roll += (float) (pData[3] - gx_offset) / 65.5 / 50;
      pitch += (float) (pData[4] - gy_offset) / 65.5 / 50;
      float _yaw = (float) (pData[5] - gz_offset) / 65.5 / 50;
      yaw += _yaw;

      if (_yaw > 1 || _yaw < -1) {
        // Если произошло изменение yaw, пересчитываем pitch и roll
        float _y = sin(_yaw * 3.1415 / 180);

        pitch += roll * _y;
        roll -= pitch * _y;
      }

      // Работа с акселерометром
      float roll4macc = atan2(pData[1], (sqrt(pow(pData[0], 2) + pow(pData[2], 2)))) * RAD_TO_DEG;
      float pitch4macc = atan2(pData[0], (sqrt(pow(pData[1], 2) + pow(pData[2], 2)))) * RAD_TO_DEG;

      float koef = 0.1;
      pitch = pitch * (1 - koef) + pitch4macc * koef;
      roll = roll * (1 - koef) + roll4macc * koef;
    }
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }

  /** Enables the Clock Security System
   */
  HAL_RCC_EnableCSS();
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_SYSTICK_Callback(void) {
  if (isCalibrate && uiTicksCNT >= 20) {
    uiTicksCNT = 0;

    isCalculation = 1;
    return;
  }
  uiTicksCNT++;
}

void MPU6050_Calibrate(void) {
  int32_t gx_cal = 0;
  int32_t gy_cal = 0;
  int32_t gz_cal = 0;
  // Калибровку выполняем за 3 секунды, 1000 итераций * 3 мс
  uint16_t count = 1000;
  for (uint16_t i = 0; i < count; i++) {
    MPU6050_Get_All_Data();
    gx_cal += pData[3];
    gy_cal += pData[4];
    gz_cal += pData[5];
    HAL_Delay(3);
  }
  gx_offset = (int16_t) (gx_cal / count);
  gy_offset = (int16_t) (gy_cal / count);
  gz_offset = (int16_t) (gz_cal / count);

  // Выставляем флаг о завершении калибровки
  isCalibrate = 1;
}

void MPU6050_Get_All_Data(void) {
  uint16_t ACCEL_XOUT_H = 0x3B;
  // Выполняем запрос на считывание всех данных
  state = HAL_I2C_Mem_Read(&hi2c1, (uint16_t) MPU6050_Address, ACCEL_XOUT_H, 1, pRxData, 14, HAL_MAX_DELAY);
  if (state == HAL_OK) {
    // Формируем 16 битные данные для осей акселерометра
    pData[0] = (int16_t) pRxData[0] << 8 | pRxData[1];    // X
    pData[1] = (int16_t) pRxData[2] << 8 | pRxData[3];    // Y
    pData[2] = (int16_t) pRxData[4] << 8 | pRxData[5];    // Z
    // Формируем 16 битные данные для осей гироскопа
    pData[3] = (int16_t) pRxData[8] << 8 | pRxData[9];    // X
    pData[4] = (int16_t) pRxData[10] << 8 | pRxData[11];  // Y
    pData[5] = (int16_t) pRxData[12] << 8 | pRxData[13];  // Z
  }
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

#ifdef  USE_FULL_ASSERT
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
