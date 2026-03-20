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
#define SHUTDOWN_Pin GPIO_PIN_13
#define SHUTDOWN_GPIO_Port GPIOC
#define ADC_Temp_Pin GPIO_PIN_0
#define ADC_Temp_GPIO_Port GPIOC
#define ADC_U_Pin GPIO_PIN_1
#define ADC_U_GPIO_Port GPIOC
#define ADC_V_Pin GPIO_PIN_2
#define ADC_V_GPIO_Port GPIOC
#define ADC_W_Pin GPIO_PIN_3
#define ADC_W_GPIO_Port GPIOC
#define ADC_VBUS_Pin GPIO_PIN_3
#define ADC_VBUS_GPIO_Port GPIOA
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
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOE
#define LED0_Pin GPIO_PIN_1
#define LED0_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
