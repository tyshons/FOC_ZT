/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */
#include "BISS_C.h"
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_7;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 4;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_8;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_9;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_4;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* ADC1 clock enable */
    __HAL_RCC_ADC12_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PC0     ------> ADC1_IN6
    PC1     ------> ADC1_IN7
    PC2     ------> ADC1_IN8
    PC3     ------> ADC1_IN9
    PA3     ------> ADC1_IN4
    */
    GPIO_InitStruct.Pin = ADC_Temp_Pin|ADC_U_Pin|ADC_V_Pin|ADC_W_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ADC_VBUS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADC_VBUS_GPIO_Port, &GPIO_InitStruct);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC12_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PC0     ------> ADC1_IN6
    PC1     ------> ADC1_IN7
    PC2     ------> ADC1_IN8
    PC3     ------> ADC1_IN9
    PA3     ------> ADC1_IN4
    */
    HAL_GPIO_DeInit(GPIOC, ADC_Temp_Pin|ADC_U_Pin|ADC_V_Pin|ADC_W_Pin);

    HAL_GPIO_DeInit(ADC_VBUS_GPIO_Port, ADC_VBUS_Pin);

    /* ADC1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
float g_adc_current[3] = {0};
float g_adc_vbus = 0;
float g_adc_temp = 0;
int16_t g_adc_offset[3] = {0};

int8_t adc_start_err = 0;
int8_t adc_timeout_err = 0;
int8_t adc_restor_fail = 0;

static int safe_injected_start(ADC_HandleTypeDef *hadc)
{
    uint32_t retries = 0;
    while (retries++ < SAFE_INJECT_MAX_RETRIES)
    {
        // 若注入仍在进行，先尝试停止
        if (hadc->Instance->CR & ADC_CR_JADSTART)
        {
            HAL_ADCEx_InjectedStop(hadc);
            // 短暂延时可选，视具体情况而定
        }
        // 尝试启动
        if (HAL_ADCEx_InjectedStart(hadc) == HAL_OK)
            return 0;
    }
    // 最终失败则再尝试清理一次
    HAL_ADCEx_InjectedStop(hadc);
    return -1;
}

// ----- 电流零点校准函数 -----
void calibrate_current_offset(void)
{
    ADC_InjectionConfTypeDef config = {0};

    // 1. 禁用 JEOC 中断，防止校准过程中进入中断
    __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_JEOC);

    // 2. 强制停止当前的注入转换（如果是硬件触发状态）
    HAL_ADCEx_InjectedStop(&hadc1);

    // 3. 重新配置为软件触发
    config.InjectedSamplingTime = ADC_SAMPLETIME_12CYCLES_5;
    config.InjectedSingleDiff = ADC_SINGLE_ENDED;
    config.InjectedNbrOfConversion = 4;
    config.InjectedDiscontinuousConvMode = DISABLE;
    config.AutoInjectedConv = DISABLE;
    config.QueueInjectedContext = DISABLE;
    config.InjectedOffsetNumber = ADC_OFFSET_NONE;
    config.InjectedOffset = 0;
    // 关键：改为软件触发
    config.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
    config.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_NONE;

    // 配置四个通道 (顺序必须与初始化时一致: CH7, CH8, CH9, CH4)
    config.InjectedChannel = ADC_CHANNEL_7; config.InjectedRank = ADC_INJECTED_RANK_1;
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &config);
    config.InjectedChannel = ADC_CHANNEL_8; config.InjectedRank = ADC_INJECTED_RANK_2;
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &config);
    config.InjectedChannel = ADC_CHANNEL_9; config.InjectedRank = ADC_INJECTED_RANK_3;
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &config);
    config.InjectedChannel = ADC_CHANNEL_4; config.InjectedRank = ADC_INJECTED_RANK_4;
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &config);

    uint32_t sum[3] = {0};
    const uint16_t CALIB_SAMPLES = 128;

    for (uint16_t i = 0; i < CALIB_SAMPLES; i++)
    {
        if (safe_injected_start(&hadc1) != 0)
        {
            adc_start_err = 0x01;
            continue;
        }

        // 等待转换完成 (超时时间设为 1ms 或更大，取决于采样时间)
        if (HAL_ADCEx_InjectedPollForConversion(&hadc1, 5) != HAL_OK)
        {
            adc_timeout_err = 0x02;
            continue;
        }

        // 读取前三个通道 (电流)
        sum[0] += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        sum[1] += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
        sum[2] += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
    }

    g_adc_offset[0] = sum[0] / CALIB_SAMPLES;
    g_adc_offset[1] = sum[1] / CALIB_SAMPLES;
    g_adc_offset[2] = sum[2] / CALIB_SAMPLES;

    // 5. 恢复硬件触发配置
    config.InjectedSamplingTime = ADC_SAMPLETIME_12CYCLES_5;
    config.InjectedSingleDiff = ADC_SINGLE_ENDED;
    config.InjectedNbrOfConversion = 4;
    // 关键：改回定时器触发 (确保 CubeMX 中配置的 T1_TRGO 宏定义正确)
    config.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
    config.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;

    config.InjectedChannel = ADC_CHANNEL_7; config.InjectedRank = ADC_INJECTED_RANK_1;
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &config);
    config.InjectedChannel = ADC_CHANNEL_8; config.InjectedRank = ADC_INJECTED_RANK_2;
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &config);
    config.InjectedChannel = ADC_CHANNEL_9; config.InjectedRank = ADC_INJECTED_RANK_3;
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &config);
    config.InjectedChannel = ADC_CHANNEL_4; config.InjectedRank = ADC_INJECTED_RANK_4;
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &config);

    // 6. 恢复中断并启动
    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);

    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
    {
        adc_restor_fail = 0x03;

    }
}

void ad_sample_process(void)
{
    Biss_start_transfer();
    // 1. 读取原始值 (顺序对应 Rank 1~4)
    uint32_t u_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    uint32_t v_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
    uint32_t w_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
    uint32_t vbus_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4);

    // 2. 减去零点偏移
    int16_t ia_raw = u_raw - g_adc_offset[0];
    int16_t ib_raw = v_raw - g_adc_offset[1];
    int16_t ic_raw = w_raw - g_adc_offset[2];

    // 3. 低通滤波 (静态变量保持状态)
    static float ia_filt = 0, ib_filt = 0, ic_filt = 0;
    ia_filt = ALPHA * (float)ia_raw + (1.0f - ALPHA) * ia_filt;
    ib_filt = ALPHA * (float)ib_raw + (1.0f - ALPHA) * ib_filt;
    ic_filt = ALPHA * (float)ic_raw + (1.0f - ALPHA) * ic_filt;

    // 4. 转换为物理量
    g_adc_current[0] = ia_filt * ADC1CURT;
    g_adc_current[1] = ib_filt * ADC1CURT;
    g_adc_current[2] = ic_filt * ADC1CURT;
    g_adc_vbus = (float)vbus_raw * ADC1VOLT;

    // 5. 三相平衡处理 (消除共模误差)
    float mid_offset = (g_adc_current[0] + g_adc_current[1] + g_adc_current[2]) / 3.0f;
    g_adc_current[0] -= mid_offset;
    g_adc_current[1] -= mid_offset;
    g_adc_current[2] -= mid_offset;

    // 6. 调用 FOC 控制循环
    Control_Loop_test();

}

// ----- 温度读取 (规则通道) -----
uint16_t adc_read_regular(uint32_t ch)
{
    ADC_ChannelConfTypeDef s = {0};
    s.Channel = ch;
    s.Rank = ADC_REGULAR_RANK_1;
    s.SamplingTime = ADC_SAMPLETIME_12CYCLES_5; // 可根据需要调整

    HAL_ADC_ConfigChannel(&hadc1, &s);
    HAL_ADC_Start(&hadc1);

    // 超时时间适当加大
    if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        uint16_t val = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
        return val;
    }
    HAL_ADC_Stop(&hadc1);
    return 0;
}

float read_temperature(void)
{

    uint16_t raw = adc_read_regular(ADC_Temp_Pin);
    g_adc_temp = (float)raw; // 或者在这里做温度转换公式
    return g_adc_temp;
}

void adc_foc_init(void)
{
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  calibrate_current_offset();  // 电流零点校准
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1) {
    ad_sample_process();  // 处理 ADC 数据
  }
}

/* USER CODE END 1 */

