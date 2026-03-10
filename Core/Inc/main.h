/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

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
#define PM1_PWM_UL_Pin GPIO_PIN_13
#define PM1_PWM_UL_GPIO_Port GPIOB
#define PM1_PWM_VL_Pin GPIO_PIN_14
#define PM1_PWM_VL_GPIO_Port GPIOB
#define PM1_PWM_WL_Pin GPIO_PIN_15
#define PM1_PWM_WL_GPIO_Port GPIOB
#define PM1_PWM_UH_Pin GPIO_PIN_8
#define PM1_PWM_UH_GPIO_Port GPIOA
#define PM__PWM_VH_Pin GPIO_PIN_9
#define PM__PWM_VH_GPIO_Port GPIOA
#define PM1_PWM_WH_Pin GPIO_PIN_10
#define PM1_PWM_WH_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
