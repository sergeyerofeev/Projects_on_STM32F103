/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
struct HIDDataPacket
{
  volatile uint8_t reportId;
  volatile uint8_t dataToReceive[7];
};
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define C13_Pin GPIO_PIN_13
#define C13_GPIO_Port GPIOC
#define C14_Pin GPIO_PIN_14
#define C14_GPIO_Port GPIOC
#define C15_Pin GPIO_PIN_15
#define C15_GPIO_Port GPIOC
#define A1_Pin GPIO_PIN_1
#define A1_GPIO_Port GPIOA
#define A2_Pin GPIO_PIN_2
#define A2_GPIO_Port GPIOA
#define EN_Pin GPIO_PIN_3
#define EN_GPIO_Port GPIOA
#define MS_1_Pin GPIO_PIN_4
#define MS_1_GPIO_Port GPIOA
#define MS_2_Pin GPIO_PIN_5
#define MS_2_GPIO_Port GPIOA
#define MS_3_Pin GPIO_PIN_6
#define MS_3_GPIO_Port GPIOA
#define DIR_Pin GPIO_PIN_7
#define DIR_GPIO_Port GPIOA
#define B0_Pin GPIO_PIN_0
#define B0_GPIO_Port GPIOB
#define B1_Pin GPIO_PIN_1
#define B1_GPIO_Port GPIOB
#define B2_Pin GPIO_PIN_2
#define B2_GPIO_Port GPIOB
#define B10_Pin GPIO_PIN_10
#define B10_GPIO_Port GPIOB
#define B11_Pin GPIO_PIN_11
#define B11_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOB
#define B13_Pin GPIO_PIN_13
#define B13_GPIO_Port GPIOB
#define B14_Pin GPIO_PIN_14
#define B14_GPIO_Port GPIOB
#define B15_Pin GPIO_PIN_15
#define B15_GPIO_Port GPIOB
#define A8_Pin GPIO_PIN_8
#define A8_GPIO_Port GPIOA
#define A9_Pin GPIO_PIN_9
#define A9_GPIO_Port GPIOA
#define A10_Pin GPIO_PIN_10
#define A10_GPIO_Port GPIOA
#define A15_Pin GPIO_PIN_15
#define A15_GPIO_Port GPIOA
#define B3_Pin GPIO_PIN_3
#define B3_GPIO_Port GPIOB
#define USB_DETECT_Pin GPIO_PIN_4
#define USB_DETECT_GPIO_Port GPIOB
#define USB_DETECT_EXTI_IRQn EXTI4_IRQn
#define B5_Pin GPIO_PIN_5
#define B5_GPIO_Port GPIOB
#define B6_Pin GPIO_PIN_6
#define B6_GPIO_Port GPIOB
#define B7_Pin GPIO_PIN_7
#define B7_GPIO_Port GPIOB
#define B8_Pin GPIO_PIN_8
#define B8_GPIO_Port GPIOB
#define B9_Pin GPIO_PIN_9
#define B9_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
