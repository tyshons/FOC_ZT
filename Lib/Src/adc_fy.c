#include "adc.h"

#include "Experiment_Config.h"
#include "FOC_Control.h"

#include <stdint.h>

float g_adc_current_fy[3] = {0.0f};
float g_adc_vbus_fy = 0.0f;
float g_adc_temp_fy = 0.0f;
float g_adc_offset_fy[3] = {0.0f};
volatile uint8_t g_adc_calibrated_fy = 0U;

volatile uint16_t g_adc_last_raw_fy[4] = {0U};
volatile float g_adc_endpoint_delta_current_fy[3] = {0.0f};
volatile uint32_t g_adc_sequence_count_fy = 0U;
volatile uint32_t g_adc_control_update_count_fy = 0U;
volatile uint8_t g_adc_pair_pending_fy = 0U;

int8_t adc_start_err_fy = 0;
int8_t adc_timeout_err_fy = 0;
int8_t adc_restore_fail_fy = 0;

static uint16_t
    adc_endpoint_raw_fy[FOC_ADC_SEQUENCES_PER_CONTROL][4] = {{0U}};
static float adc_current_filtered_counts_fy[3] = {0.0f};
static uint8_t adc_pair_index_fy = 0U;

static const uint32_t adc_injected_channels_fy[4] = {
    ADC_CHANNEL_4,
    ADC_CHANNEL_6,
    ADC_CHANNEL_14,
    ADC_CHANNEL_1,
};

static const uint32_t adc_injected_ranks_fy[4] = {
    ADC_INJECTED_RANK_1,
    ADC_INJECTED_RANK_2,
    ADC_INJECTED_RANK_3,
    ADC_INJECTED_RANK_4,
};

void ADC_ResetCurrentProcessing_fy(void)
{
  adc_pair_index_fy = 0U;
  g_adc_pair_pending_fy = 0U;
  for (uint32_t phase = 0U; phase < 3U; phase++) {
    adc_current_filtered_counts_fy[phase] = 0.0f;
    g_adc_current_fy[phase] = 0.0f;
    g_adc_endpoint_delta_current_fy[phase] = 0.0f;
    g_adc_last_raw_fy[phase] = 0U;
    adc_endpoint_raw_fy[0][phase] = 0U;
    adc_endpoint_raw_fy[1][phase] = 0U;
  }
  adc_endpoint_raw_fy[0][3] = 0U;
  adc_endpoint_raw_fy[1][3] = 0U;
  g_adc_last_raw_fy[3] = 0U;
  g_adc_vbus_fy = 0.0f;
}

static int safe_injected_start_fy(void)
{
  uint32_t retries = 0U;
  while (retries++ < SAFE_INJECT_MAX_RETRIES) {
    if ((hadc3.Instance->CR & ADC_CR_JADSTART) != 0U) {
      (void)HAL_ADCEx_InjectedStop(&hadc3);
    }
    if (HAL_ADCEx_InjectedStart(&hadc3) == HAL_OK) {
      return 0;
    }
  }
  (void)HAL_ADCEx_InjectedStop(&hadc3);
  return -1;
}

static HAL_StatusTypeDef configure_injected_channels_fy(
    uint32_t trigger,
    uint32_t trigger_edge)
{
  ADC_InjectionConfTypeDef config = {0};
  config.InjectedSamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  config.InjectedSingleDiff = ADC_SINGLE_ENDED;
  config.InjectedNbrOfConversion = 4U;
  config.InjectedDiscontinuousConvMode = DISABLE;
  config.AutoInjectedConv = DISABLE;
  config.QueueInjectedContext = DISABLE;
  config.InjectedOffsetNumber = ADC_OFFSET_NONE;
  config.InjectedOffset = 0U;
  config.ExternalTrigInjecConv = trigger;
  config.ExternalTrigInjecConvEdge = trigger_edge;
  config.InjecOversamplingMode = DISABLE;

  for (uint32_t index = 0U; index < 4U; index++) {
    config.InjectedChannel = adc_injected_channels_fy[index];
    config.InjectedRank = adc_injected_ranks_fy[index];
    if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &config) != HAL_OK) {
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}

void calibrate_current_offset_fy(void)
{
  __HAL_ADC_DISABLE_IT(&hadc3, ADC_IT_JEOC | ADC_IT_JEOS);
  (void)HAL_ADCEx_InjectedStop(&hadc3);

  g_adc_calibrated_fy = 0U;
  adc_start_err_fy = 0;
  adc_timeout_err_fy = 0;
  adc_restore_fail_fy = 0;

  if (configure_injected_channels_fy(
          ADC_INJECTED_SOFTWARE_START,
          ADC_EXTERNALTRIGINJECCONV_EDGE_NONE) != HAL_OK) {
    adc_restore_fail_fy = 0x05;
    return;
  }

  uint64_t sum[3] = {0U};
  uint32_t valid_samples = 0U;
  for (uint32_t index = 0U; index < FOC_ADC_OFFSET_SAMPLE_COUNT; index++) {
    if (safe_injected_start_fy() != 0) {
      adc_start_err_fy = 0x01;
      continue;
    }
    if (HAL_ADCEx_InjectedPollForConversion(&hadc3, 5U) != HAL_OK) {
      adc_timeout_err_fy = 0x02;
      continue;
    }
    sum[0] += HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1);
    sum[1] += HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_2);
    sum[2] += HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_3);
    valid_samples++;
  }

  if (valid_samples >= (FOC_ADC_OFFSET_SAMPLE_COUNT / 2U)) {
    for (uint32_t phase = 0U; phase < 3U; phase++) {
      g_adc_offset_fy[phase] = (float)sum[phase] / (float)valid_samples;
    }
    g_adc_calibrated_fy = 1U;
  }

  if (configure_injected_channels_fy(
          ADC_EXTERNALTRIGINJEC_T8_TRGO,
          ADC_EXTERNALTRIGINJECCONV_EDGE_RISING) != HAL_OK) {
    adc_restore_fail_fy = 0x05;
    g_adc_calibrated_fy = 0U;
    return;
  }
  if (HAL_ADCEx_InjectedStart_IT(&hadc3) != HAL_OK) {
    adc_restore_fail_fy = 0x03;
    g_adc_calibrated_fy = 0U;
  }

  ADC_ResetCurrentProcessing_fy();
  g_adc_sequence_count_fy = 0U;
  g_adc_control_update_count_fy = 0U;
}

void ad_sample_process_fy(void)
{
  const uint16_t raw[4] = {
      (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1),
      (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_2),
      (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_3),
      (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_4),
  };
  const uint8_t endpoint = adc_pair_index_fy;

  for (uint32_t channel = 0U; channel < 4U; channel++) {
    adc_endpoint_raw_fy[endpoint][channel] = raw[channel];
    g_adc_last_raw_fy[channel] = raw[channel];
  }
  g_adc_sequence_count_fy++;

  if (endpoint == 0U) {
    adc_pair_index_fy = 1U;
    g_adc_pair_pending_fy = 1U;
    return;
  }
  adc_pair_index_fy = 0U;
  g_adc_pair_pending_fy = 0U;

  float endpoint_common_counts[FOC_ADC_SEQUENCES_PER_CONTROL] = {0.0f};
  for (uint32_t sample = 0U;
       sample < FOC_ADC_SEQUENCES_PER_CONTROL;
       sample++) {
    for (uint32_t phase = 0U; phase < 3U; phase++) {
      endpoint_common_counts[sample] +=
          (float)adc_endpoint_raw_fy[sample][phase] - g_adc_offset_fy[phase];
    }
    endpoint_common_counts[sample] /= 3.0f;
  }

  for (uint32_t phase = 0U; phase < 3U; phase++) {
    const float first_counts =
        (float)adc_endpoint_raw_fy[0][phase] - g_adc_offset_fy[phase] -
        endpoint_common_counts[0];
    const float second_counts =
        (float)adc_endpoint_raw_fy[1][phase] - g_adc_offset_fy[phase] -
        endpoint_common_counts[1];
    g_adc_endpoint_delta_current_fy[phase] =
        (second_counts - first_counts) * ADC1CURT;

    const float averaged_counts =
        0.5f * ((float)adc_endpoint_raw_fy[0][phase] +
                (float)adc_endpoint_raw_fy[1][phase]) -
        g_adc_offset_fy[phase];
    adc_current_filtered_counts_fy[phase] =
        FOC_CURRENT_FILTER_ALPHA * averaged_counts +
        (1.0f - FOC_CURRENT_FILTER_ALPHA) *
            adc_current_filtered_counts_fy[phase];
    g_adc_current_fy[phase] =
        adc_current_filtered_counts_fy[phase] * ADC1CURT;
  }

  g_adc_vbus_fy =
      0.5f * ((float)adc_endpoint_raw_fy[0][3] +
              (float)adc_endpoint_raw_fy[1][3]) * ADC1VOLT;

  const float current_common =
      (g_adc_current_fy[0] + g_adc_current_fy[1] + g_adc_current_fy[2]) /
      3.0f;
  g_adc_current_fy[0] -= current_common;
  g_adc_current_fy[1] -= current_common;
  g_adc_current_fy[2] -= current_common;

  g_adc_control_update_count_fy++;
  Control_Loop(FOC_AXIS_FY);
}

uint16_t adc_read_regular_fy(uint32_t channel)
{
  ADC_ChannelConfTypeDef config = {0};
  config.Channel = channel;
  config.Rank = ADC_REGULAR_RANK_1;
  config.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  config.SingleDiff = ADC_SINGLE_ENDED;
  config.OffsetNumber = ADC_OFFSET_NONE;
  config.Offset = 0U;
  if (HAL_ADC_ConfigChannel(&hadc3, &config) != HAL_OK) {
    return 0U;
  }
  if (HAL_ADC_Start(&hadc3) != HAL_OK) {
    return 0U;
  }
  if (HAL_ADC_PollForConversion(&hadc3, 10U) == HAL_OK) {
    const uint16_t value = (uint16_t)HAL_ADC_GetValue(&hadc3);
    (void)HAL_ADC_Stop(&hadc3);
    return value;
  }
  (void)HAL_ADC_Stop(&hadc3);
  return 0U;
}

float read_temperature_fy(void)
{
  g_adc_temp_fy = (float)adc_read_regular_fy(ADC_CHANNEL_2);
  return g_adc_temp_fy;
}
