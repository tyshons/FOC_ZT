/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <string.h>

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN Private defines */
#define ADC1CURT    (0.03223f)   //ADC电流采集系数 = (3.3f / 4096.0f / 0.025f)
#define ADC1VOLT    (0.02014f) 	 //ADC电压采集系数 = (3.3f / 4096.0f / 0.0545f)
#define SAFE_INJECT_MAX_RETRIES 2

  extern float g_adc_current[3];   // 三相电流 (A)
  extern float g_adc_vbus;         // 母线电压 (V)
  extern float g_adc_temp;         // 温度 (原始值或转换后)
  extern float g_adc_offset[3];    // 电流零点偏移量 (ADC计数，保留小数)
  extern volatile uint8_t g_adc_calibrated;

  /* 双端点采样诊断量，可由调试器或诊断帧只读查看。 */
  extern volatile uint16_t g_adc_last_raw[4];
  extern volatile float g_adc_endpoint_delta_current[3];
  extern volatile uint32_t g_adc_sequence_count;
  extern volatile uint32_t g_adc_control_update_count;
  extern volatile uint8_t g_adc_pair_pending;

  // 错误标志
  extern int8_t adc_start_err;
  extern int8_t adc_timeout_err;
  extern int8_t adc_restor_fail;

/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */

  void adc_foc_init(void);          // 额外的初始化（如校准启动）
  void calibrate_current_offset(void); // 电流零点校准
  float read_temperature(void);     // 读取温度
  void ad_sample_process(void);
  void ADC_ResetCurrentProcessing(void);
  uint16_t adc_read_regular(uint32_t ch);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

