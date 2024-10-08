/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_customhid.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
// Флаг готовности полученных с хоста данных
extern volatile bool isReceived;
// Структура для сохранения reportId и массива данных
extern struct HIDDataPacket data;

// Отслеживаем разрыв соединения по USB
// Флаг устанавливается в true при сбросе напряжения в 0 на разъёме USB
volatile bool isDisconnected = false;
volatile uint32_t time_exti4 = 0;

// Количество шагов двигателя
volatile int16_t stepsCount = 400;
// Флаг завершения одного оборота двигателя
volatile bool isTurnComplete = false;
uint8_t sendReportId5[5] = { 5 };
// Массив из значения тактовой частоты
uint8_t sendReportId7[5] = { 7 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void parseReportId(void);
static void isDisconnectUSB(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

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
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  // Получаем значение тактовой частоты
  uint32_t sysclk = SystemCoreClock;
  // Формируем массив из значения тактовой частоты
  sendReportId7[1] = sysclk >> 24 & 0xFF;
  sendReportId7[2] = sysclk >> 16 & 0xFF;
  sendReportId7[3] = sysclk >> 8 & 0xFF;
  sendReportId7[4] = sysclk & 0xFF;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    if (isReceived) {
      // С хоста получили данные, обрабатываем в зависимости от переданного ReportId
      parseReportId();
    }

    if (isTurnComplete) {
      isTurnComplete = false;
      // Один оборот завершён, выключаем таймер
      HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);
      // Выключаем драйвер A4988, установив на выводе EN единицу
      HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
      // Отправляем на хост уведомление
      USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, sendReportId5, 5);
    }

    if (isDisconnected && (HAL_GetTick() - time_exti4) > 200) {
      // Произошла потеря питания линии USB
      isDisconnectUSB();
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
  RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }

  /** Enables the Clock Security System
   */
  HAL_RCC_EnableCSS();
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = { 0 };
  TIM_OC_InitTypeDef sConfigOC = { 0 };

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48000 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 1000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MS_1_Pin | MS_2_Pin | MS_3_Pin | DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : C13_Pin C14_Pin C15_Pin */
  GPIO_InitStruct.Pin = C13_Pin | C14_Pin | C15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : A1_Pin A2_Pin A8_Pin A9_Pin
   A10_Pin A15_Pin */
  GPIO_InitStruct.Pin = A1_Pin | A2_Pin | A8_Pin | A9_Pin | A10_Pin | A15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_Pin MS_1_Pin MS_2_Pin MS_3_Pin
   DIR_Pin */
  GPIO_InitStruct.Pin = EN_Pin | MS_1_Pin | MS_2_Pin | MS_3_Pin | DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : B0_Pin B1_Pin B2_Pin B10_Pin
   B11_Pin B13_Pin B14_Pin B15_Pin
   B3_Pin B5_Pin B6_Pin B7_Pin
   B8_Pin B9_Pin */
  GPIO_InitStruct.Pin = B0_Pin | B1_Pin | B2_Pin | B10_Pin | B11_Pin | B13_Pin | B14_Pin | B15_Pin | B3_Pin | B5_Pin | B6_Pin | B7_Pin | B8_Pin | B9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_DETECT_Pin */
  GPIO_InitStruct.Pin = USB_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(USB_DETECT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // Устанавливаем пин DP как выход
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  // Подтягиваем пин DP к GND
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  // Задержка для завершения процессов на хосте
  for (uint16_t i = 0; i < 1000; i++);

  // Переинициализируем пин DP для работы с USB
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  // Задержка для завершения процессов на хосте
  for (uint16_t i = 0; i < 1000; i++);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Сначала выполним проверку, может быть другой таймер или канал
  if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    if (stepsCount-- < 0) {
      // Один оборот заверён
      isTurnComplete = true;
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == USB_DETECT_Pin) {
    // Сразу же отключаем внешнее прерывания EXTI4
    HAL_NVIC_DisableIRQ(EXTI4_IRQn);
    // Устанавливаем флаг и сохраняем текущее время
    isDisconnected = true;
    time_exti4 = HAL_GetTick();
  }
}

static void parseReportId(void)
{
  isReceived = false;

  switch (data.reportId) {
    case 1:
      // Команда на остановку двигателя
      if ((&htim2)->ChannelState[0] == HAL_TIM_CHANNEL_STATE_BUSY) {
        // TIM_CHANNEL_1 работает, останавливаем его

        // Проверка наличия обработчика прерывания для таймера
        if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_CC1) == SET) {
          // Обработчик прерывания установлен, таймер работает в режиме с прерыванием
          HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);
        } else {
          // Обработчик прерывания не установлен, таймер работает без прерывания
          HAL_TIM_OC_Stop(&htim2, TIM_CHANNEL_1);
        }

        // Выключаем драйвер A4988, установив на выводе EN единицу
        HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
      }
      break;
    case 2:
      // Двигатель вращается, изменяем значение регистра ARR
      TIM2->ARR = (data.dataToReceive[0] << 8 | data.dataToReceive[1]);
      break;
    case 3:
      // Один оборот вала двигателя
      // Для этого режима нам необходимо установить количество шагов двигателя
      // на основании этого значения мы будем отслеживать один оборот вала
      // Дополнительно умножаем количество шагов двигателя на 2, т.к. ARR составляет половину периода
      switch (data.dataToReceive[2]) {
        case 0:
          stepsCount = 400 << 1;
          break;
        case 1:
          stepsCount = 200 << 1;
          break;
        case 2:
          stepsCount = 100 << 1;
          break;
        case 3:
          stepsCount = 64 << 1;
          break;
        case 4:
          stepsCount = 48 << 1;
          break;
      }
    case 4:
      // Непрерывное вращение
      // Устанавливаем микрошаговый режим
      switch (data.dataToReceive[0]) {
        case 0:
          // Полный шаг
          HAL_GPIO_WritePin(GPIOA, MS_1_Pin | MS_2_Pin | MS_3_Pin, GPIO_PIN_RESET);
          break;
        case 1:
          // 1/2 шага
          HAL_GPIO_WritePin(GPIOA, MS_1_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOA, MS_2_Pin | MS_3_Pin, GPIO_PIN_RESET);
          // Если выбран режим одного оборота, количество шагов умножаем на 2
          if (data.reportId == 3)
            stepsCount <<= 1;
          break;
        case 2:
          // 1/4 шага
          HAL_GPIO_WritePin(GPIOA, MS_2_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOA, MS_1_Pin | MS_3_Pin, GPIO_PIN_RESET);
          // Если выбран режим одного оборота, количество шагов умножаем на 4
          if (data.reportId == 3)
            stepsCount <<= 2;
          break;
        case 3:
          // 1/8 шага
          HAL_GPIO_WritePin(GPIOA, MS_1_Pin | MS_2_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOA, MS_3_Pin, GPIO_PIN_RESET);
          // Если выбран режим одного оборота, количество шагов умножаем на 8
          if (data.reportId == 3)
            stepsCount <<= 3;
          break;
        case 4:
          // 1/16 шага
          HAL_GPIO_WritePin(GPIOA, MS_1_Pin | MS_2_Pin | MS_3_Pin, GPIO_PIN_SET);
          // Если выбран режим одного оборота количество шагов умножаем на 16
          if (data.reportId == 3)
            stepsCount <<= 4;
          break;
      }
      // Устанавливаем направление вращения
      switch (data.dataToReceive[1]) {
        case 0:
          HAL_GPIO_WritePin(GPIOA, DIR_Pin, GPIO_PIN_RESET);
          break;
        case 1:
          HAL_GPIO_WritePin(GPIOA, DIR_Pin, GPIO_PIN_SET);
          break;
      }

      TIM2->PSC = (data.dataToReceive[3] << 8 | data.dataToReceive[4]);
      TIM2->ARR = (data.dataToReceive[5] << 8 | data.dataToReceive[6]);
      // Включаем драйвер A4988, сбросив вывод PA3 в ноль
      HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);

      if ((&htim2)->ChannelState[0] == HAL_TIM_CHANNEL_STATE_READY) {
        if (data.reportId == 3) {
          // Если выбран "Один оборот" запускаем таймер с прерыванием, чтобы
          // подсчитать количество шагов
          HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
        } else if (data.reportId == 4) {
          HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
        }
      }
      break;
    case 6:
      // Запрос на получение тактовой частоты
      USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, sendReportId7, 5);
      break;
  }
}

static void isDisconnectUSB(void)
{
  // Ожидаем 200 мс для повторной проверки питания линии USB
  if (HAL_GPIO_ReadPin(USB_DETECT_GPIO_Port, USB_DETECT_Pin) == GPIO_PIN_RESET) {
    // На линии USB отсутствует напряжние
    // Сбрасываем флаг
    isDisconnected = false;

    if ((&htim2)->ChannelState[0] == HAL_TIM_CHANNEL_STATE_BUSY) {
      // TIM_CHANNEL_1 работает, останавливаем его
      HAL_TIM_OC_Stop(&htim2, TIM_CHANNEL_1);
      // Выключаем драйвер A4988, установив на выводе EN единицу
      HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
    }
  }
  // Запускаем проверку для нового ослеживания разрыва соединения
  // При помощи макроса очищаем бит EXTI_PR
  __HAL_GPIO_EXTI_CLEAR_IT(USB_DETECT_Pin);
  // Очищаем бит NVIC_ICPRx
  NVIC_ClearPendingIRQ(EXTI4_IRQn);
  // Включаем внешнее прерывание
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
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
