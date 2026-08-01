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
#include "FOC_Control.h"
#include "Experiment_Config.h"
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
  /*
   * U/V/W 相电流和母线电压组成一个注入转换序列。
   * 四个序列全部完成后只触发一次 FOC 回调，避免每个序列分别回调。
   */
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
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
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
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
float g_adc_offset[3] = {0};
volatile uint8_t g_adc_calibrated = 0U;

volatile uint16_t g_adc_last_raw[4] = {0U};
volatile float g_adc_endpoint_delta_current[3] = {0.0f};
volatile uint32_t g_adc_sequence_count = 0U;
volatile uint32_t g_adc_control_update_count = 0U;
volatile uint8_t g_adc_pair_pending = 0U;

static uint16_t
    adc_endpoint_raw[FOC_ADC_SEQUENCES_PER_CONTROL][4] = {{0U}};
static float adc_current_filtered_counts[3] = {0.0f};
static uint8_t adc_pair_index = 0U;

int8_t adc_start_err = 0;
int8_t adc_timeout_err = 0;
int8_t adc_restor_fail = 0;

void ADC_ResetCurrentProcessing(void)
{
    adc_pair_index = 0U;
    g_adc_pair_pending = 0U;
    for (uint32_t phase = 0U; phase < 3U; phase++)
    {
        adc_current_filtered_counts[phase] = 0.0f;
        g_adc_current[phase] = 0.0f;
        g_adc_endpoint_delta_current[phase] = 0.0f;
        g_adc_last_raw[phase] = 0U;
        adc_endpoint_raw[0][phase] = 0U;
        adc_endpoint_raw[1][phase] = 0U;
    }
    adc_endpoint_raw[0][3] = 0U;
    adc_endpoint_raw[1][3] = 0U;
    g_adc_last_raw[3] = 0U;
    g_adc_vbus = 0.0f;
}

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

    // 1. 禁用注入组中断，防止校准过程中进入控制中断
    __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_JEOC | ADC_IT_JEOS);

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

    uint64_t sum[3] = {0U};
    uint32_t valid_samples = 0U;
    g_adc_calibrated = 0U;
    adc_start_err = 0;
    adc_timeout_err = 0;
    adc_restor_fail = 0;

    for (uint32_t i = 0U; i < FOC_ADC_OFFSET_SAMPLE_COUNT; i++)
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
        valid_samples++;
    }

    if (valid_samples >= (FOC_ADC_OFFSET_SAMPLE_COUNT / 2U))
    {
        g_adc_offset[0] = (float)sum[0] / (float)valid_samples;
        g_adc_offset[1] = (float)sum[1] / (float)valid_samples;
        g_adc_offset[2] = (float)sum[2] / (float)valid_samples;
        g_adc_calibrated = 1U;
    }

    // 5. 恢复硬件触发配置
    config.InjectedSamplingTime = ADC_SAMPLETIME_12CYCLES_5;
    config.InjectedSingleDiff = ADC_SINGLE_ENDED;
    config.InjectedNbrOfConversion = 4;
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

    // 6. HAL 根据 EOCSelection 启用 JEOS 中断并启动
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
    {
        adc_restor_fail = 0x03;
        g_adc_calibrated = 0U;
    }

    ADC_ResetCurrentProcessing();
    g_adc_sequence_count = 0U;
    g_adc_control_update_count = 0U;
}

void ad_sample_process(void)
{
    const uint16_t raw[4] = {
        (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1),
        (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2),
        (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3),
        (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4)
    };
    const uint8_t endpoint = adc_pair_index;

    for (uint32_t channel = 0U; channel < 4U; channel++)
    {
        adc_endpoint_raw[endpoint][channel] = raw[channel];
        g_adc_last_raw[channel] = raw[channel];
    }
    g_adc_sequence_count++;

    /* 第一端点只缓存；第二端点到达后才形成一组20 kHz控制数据。 */
    if (endpoint == 0U)
    {
        adc_pair_index = 1U;
        g_adc_pair_pending = 1U;
        return;
    }
    adc_pair_index = 0U;
    g_adc_pair_pending = 0U;

    float endpoint_common_counts[FOC_ADC_SEQUENCES_PER_CONTROL] = {0.0f};
    for (uint32_t sample = 0U;
         sample < FOC_ADC_SEQUENCES_PER_CONTROL;
         sample++)
    {
        for (uint32_t phase = 0U; phase < 3U; phase++)
        {
            endpoint_common_counts[sample] +=
                (float)adc_endpoint_raw[sample][phase] - g_adc_offset[phase];
        }
        endpoint_common_counts[sample] /= 3.0f;
    }

    for (uint32_t phase = 0U; phase < 3U; phase++)
    {
        const float first_counts =
            (float)adc_endpoint_raw[0][phase] - g_adc_offset[phase] -
            endpoint_common_counts[0];
        const float second_counts =
            (float)adc_endpoint_raw[1][phase] - g_adc_offset[phase] -
            endpoint_common_counts[1];
        g_adc_endpoint_delta_current[phase] =
            (second_counts - first_counts) * ADC1CURT;

        const float averaged_counts =
            0.5f * ((float)adc_endpoint_raw[0][phase] +
                    (float)adc_endpoint_raw[1][phase]) -
            g_adc_offset[phase];
        adc_current_filtered_counts[phase] =
            FOC_CURRENT_FILTER_ALPHA * averaged_counts +
            (1.0f - FOC_CURRENT_FILTER_ALPHA) *
                adc_current_filtered_counts[phase];
        g_adc_current[phase] = adc_current_filtered_counts[phase] * ADC1CURT;
    }

    g_adc_vbus =
        0.5f * ((float)adc_endpoint_raw[0][3] +
                (float)adc_endpoint_raw[1][3]) * ADC1VOLT;

    /* 三相和强制为零，只消除共模误差，差分误差仍保留在诊断量中。 */
    const float current_common =
        (g_adc_current[0] + g_adc_current[1] + g_adc_current[2]) / 3.0f;
    g_adc_current[0] -= current_common;
    g_adc_current[1] -= current_common;
    g_adc_current[2] -= current_common;

    g_adc_control_update_count++;
    Control_Loop();
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

    uint16_t raw = adc_read_regular(ADC_CHANNEL_6);
    g_adc_temp = (float)raw; // 或者在这里做温度转换公式
    return g_adc_temp;
}

void adc_foc_init(void)
{
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
    adc_start_err = 0x04;
    g_adc_calibrated = 0U;
    return;
  }
  calibrate_current_offset();  // 电流零点校准
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1) {
    ad_sample_process();  // 处理 ADC 数据
  }
}

/* USER CODE END 1 */

