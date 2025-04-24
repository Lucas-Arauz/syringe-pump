/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STEPPER_EN_Pin GPIO_PIN_1
#define STEPPER_EN_GPIO_Port GPIOA
#define STEPPER_DIR_Pin GPIO_PIN_2
#define STEPPER_DIR_GPIO_Port GPIOA
#define STEPPER_STEP_Pin GPIO_PIN_3
#define STEPPER_STEP_GPIO_Port GPIOA
#define BTN_DOWN_Pin GPIO_PIN_4
#define BTN_DOWN_GPIO_Port GPIOA
#define BTN_DOWN_EXTI_IRQn EXTI4_IRQn
#define BTN_SELECT_Pin GPIO_PIN_5
#define BTN_SELECT_GPIO_Port GPIOA
#define BTN_SELECT_EXTI_IRQn EXTI9_5_IRQn
#define BTN_UP_Pin GPIO_PIN_6
#define BTN_UP_GPIO_Port GPIOA
#define BTN_UP_EXTI_IRQn EXTI9_5_IRQn
#define STEPPER_UART_Pin GPIO_PIN_9
#define STEPPER_UART_GPIO_Port GPIOA
#define DISPLAY_SCL_Pin GPIO_PIN_6
#define DISPLAY_SCL_GPIO_Port GPIOB
#define DISPLAY_SDA_Pin GPIO_PIN_7
#define DISPLAY_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
