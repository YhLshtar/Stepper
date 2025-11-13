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
#include "stm32g0xx_hal.h"

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
#define WARNING_LED_Pin GPIO_PIN_13
#define WARNING_LED_GPIO_Port GPIOC
#define TJA1051T_S_Pin GPIO_PIN_1
#define TJA1051T_S_GPIO_Port GPIOB
#define USER_IO2_Pin GPIO_PIN_11
#define USER_IO2_GPIO_Port GPIOB
#define USER_IO1_Pin GPIO_PIN_12
#define USER_IO1_GPIO_Port GPIOB
#define TMC2209_STDBY_Pin GPIO_PIN_13
#define TMC2209_STDBY_GPIO_Port GPIOB
#define TMC2209_DIR_Pin GPIO_PIN_14
#define TMC2209_DIR_GPIO_Port GPIOB
#define TMC2209_DIAG_Pin GPIO_PIN_15
#define TMC2209_DIAG_GPIO_Port GPIOA
#define TMC2209_MS2_Pin GPIO_PIN_0
#define TMC2209_MS2_GPIO_Port GPIOD
#define TMC2209_MS1_Pin GPIO_PIN_1
#define TMC2209_MS1_GPIO_Port GPIOD
#define TMC2209_SPR_Pin GPIO_PIN_2
#define TMC2209_SPR_GPIO_Port GPIOD
#define TMC2209_EN_Pin GPIO_PIN_8
#define TMC2209_EN_GPIO_Port GPIOB
#define MT6701_CS_Pin GPIO_PIN_9
#define MT6701_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
