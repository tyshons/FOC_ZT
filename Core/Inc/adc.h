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

extern ADC_HandleTypeDef hadc3;

/* USER CODE BEGIN Private defines */
#define ADC1CURT    (0.03223f)   //ADC电流采集系数 = (3.3f / 4096.0f / 0.025f)
#define ADC1VOLT    (0.02014f) 	 //ADC电压采集系数 = (3.3f / 4096.0f / 0.0545f)
#define SAFE_INJECT_MAX_RETRIES 2

  extern float g_adc_current_sp[3];
  extern float g_adc_current_fy[3];
  extern float g_adc_vbus_sp;
  extern float g_adc_vbus_fy;
  extern float g_adc_temp_sp;
  extern float g_adc_temp_fy;
  extern float g_adc_offset_sp[3];
  extern float g_adc_offset_fy[3];
  extern volatile uint8_t g_adc_calibrated_sp;
  extern volatile uint8_t g_adc_calibrated_fy;

  /* 双端点采样诊断量，可由调试器或诊断帧只读查看。 */
  extern volatile uint16_t g_adc_last_raw_sp[4];
  extern volatile uint16_t g_adc_last_raw_fy[4];
  extern volatile float g_adc_endpoint_delta_current_sp[3];
  extern volatile float g_adc_endpoint_delta_current_fy[3];
  extern volatile uint32_t g_adc_sequence_count_sp;
  extern volatile uint32_t g_adc_sequence_count_fy;
  extern volatile uint32_t g_adc_control_update_count_sp;
  extern volatile uint32_t g_adc_control_update_count_fy;
  extern volatile uint8_t g_adc_pair_pending_sp;
  extern volatile uint8_t g_adc_pair_pending_fy;

  // 错误标志
  extern int8_t adc_start_err_sp;
  extern int8_t adc_start_err_fy;
  extern int8_t adc_timeout_err_sp;
  extern int8_t adc_timeout_err_fy;
  extern int8_t adc_restore_fail_sp;
  extern int8_t adc_restore_fail_fy;

/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC3_Init(void);

/* USER CODE BEGIN Prototypes */

  void adc_foc_init(void);          // 额外的初始化（如校准启动）
  void calibrate_current_offset_sp(void);
  void calibrate_current_offset_fy(void);
  float read_temperature_sp(void);
  float read_temperature_fy(void);
  void ad_sample_process_sp(void);
  void ad_sample_process_fy(void);
  void ADC_ResetCurrentProcessing_sp(void);
  void ADC_ResetCurrentProcessing_fy(void);
  uint16_t adc_read_regular_sp(uint32_t ch);
  uint16_t adc_read_regular_fy(uint32_t ch);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

