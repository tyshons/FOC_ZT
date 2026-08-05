#include "Experiment_FixedFF_Sweep.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define SWEEP_PI 3.14159265358979323846f
#define SWEEP_TWO_PI (2.0f * SWEEP_PI)

/* 上位机或调试器写入请求，状态机在新一轮扫频开始时锁存这些参数。 */
volatile uint8_t g_sweep_enable_request = 0U;
volatile uint8_t g_sweep_reset_request = 0U;
volatile uint8_t g_sweep_start_order_request = EXPERIMENT_SWEEP_START_ORDER;
volatile uint8_t g_sweep_end_order_request = EXPERIMENT_SWEEP_END_ORDER;
volatile float g_sweep_injection_amplitude_request_a =
    EXPERIMENT_SWEEP_DEFAULT_INJECTION_A;

volatile uint8_t g_sweep_state = FIXED_FF_SWEEP_IDLE;
volatile uint8_t g_sweep_active_order = 0U;
volatile uint8_t g_sweep_stage_revolution_count = 0U;
volatile uint8_t g_sweep_speed_ready = 0U;
volatile uint8_t g_sweep_current_ready = 0U;
volatile uint8_t g_sweep_complete = 0U;
volatile uint8_t g_sweep_completed_order_count = 0U;
volatile float g_iq_sweep_a = 0.0f;
volatile float g_sweep_last_revolution_mean_rpm = 0.0f;
volatile float g_sweep_last_revolution_coverage = 0.0f;
volatile uint16_t g_sweep_rejected_revolution_count = 0U;

volatile float g_sweep_baseline_cos_rpm[PAPER_LFF_MAX_ORDER + 1U];
volatile float g_sweep_baseline_sin_rpm[PAPER_LFF_MAX_ORDER + 1U];
volatile float g_sweep_response_cos_rpm_per_a[PAPER_LFF_MAX_ORDER + 1U];
volatile float g_sweep_response_sin_rpm_per_a[PAPER_LFF_MAX_ORDER + 1U];
volatile float
    g_sweep_response_magnitude_rpm_per_a[PAPER_LFF_MAX_ORDER + 1U];
volatile float
    g_sweep_recommended_cos_current_a[PAPER_LFF_MAX_ORDER + 1U];
volatile float
    g_sweep_recommended_sin_current_a[PAPER_LFF_MAX_ORDER + 1U];
volatile float
    g_sweep_positive_coefficient_std_rpm[PAPER_LFF_MAX_ORDER + 1U];
volatile float
    g_sweep_negative_coefficient_std_rpm[PAPER_LFF_MAX_ORDER + 1U];
volatile float
    g_sweep_baseline_standard_error_rpm[PAPER_LFF_MAX_ORDER + 1U];
volatile float
    g_sweep_response_standard_error_rpm_per_a[PAPER_LFF_MAX_ORDER + 1U];
volatile float g_sweep_response_snr[PAPER_LFF_MAX_ORDER + 1U];
volatile float
    g_sweep_recommended_standard_error_a[PAPER_LFF_MAX_ORDER + 1U];
volatile uint8_t
    g_sweep_statistics_revolution_count[PAPER_LFF_MAX_ORDER + 1U];
volatile uint8_t g_sweep_result_valid[PAPER_LFF_MAX_ORDER + 1U];

/* 当前扫频锁存值。 */
static uint8_t active_end_order = EXPERIMENT_SWEEP_END_ORDER;
static float active_injection_amplitude_a =
    EXPERIMENT_SWEEP_DEFAULT_INJECTION_A;
static float previous_angle_rad = 0.0f;
static uint8_t previous_angle_valid = 0U;
static uint8_t stage_boundary_seen = 0U;

/* 阶段统计只累计质量合格的完整机械圈，并用Welford算法避免差分消减。 */
static uint8_t stage_statistics_count = 0U;
static float stage_cos_mean_rpm = 0.0f;
static float stage_sin_mean_rpm = 0.0f;
static float stage_cos_m2_rpm2 = 0.0f;
static float stage_sin_m2_rpm2 = 0.0f;
static float positive_cos_rpm = 0.0f;
static float positive_sin_rpm = 0.0f;
static float positive_cos_variance_rpm2 = 0.0f;
static float positive_sin_variance_rpm2 = 0.0f;
static uint8_t positive_statistics_count = 0U;

/* 一圈内的质量和等角度投影状态。 */
static float revolution_speed_sum = 0.0f;
static uint32_t revolution_speed_sample_count = 0U;
static float revolution_cos_sum = 0.0f;
static float revolution_sin_sum = 0.0f;
static uint16_t revolution_unique_bin_count = 0U;
static uint8_t revolution_invalid = 0U;
static uint8_t revolution_bin_seen[PAPER_LFF_POSITION_BINS];
static uint16_t active_position_bin = 0U;
static uint8_t active_position_bin_valid = 0U;
static float active_position_bin_speed_sum = 0.0f;
static uint32_t active_position_bin_sample_count = 0U;

static float ClampFloat(float value, float minimum, float maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

static uint8_t ClampOrder(uint8_t order) {
  if (order < 1U) {
    return 1U;
  }
  if (order > PAPER_LFF_MAX_ORDER) {
    return PAPER_LFF_MAX_ORDER;
  }
  return order;
}

static uint8_t IsMeasurementState(void) {
  return ((g_sweep_state == FIXED_FF_SWEEP_MEASURE_POSITIVE) ||
          (g_sweep_state == FIXED_FF_SWEEP_MEASURE_NEGATIVE))
             ? 1U
             : 0U;
}

static float NormalizeMechanicalAngle(float angle_rad) {
  float normalized = fmodf(angle_rad, SWEEP_TWO_PI);
  if (normalized < 0.0f) {
    normalized += SWEEP_TWO_PI;
  }
  return normalized;
}

static uint16_t PositionBinFromAngle(float angle_rad) {
  const float normalized = NormalizeMechanicalAngle(angle_rad);
  uint32_t bin = (uint32_t)(normalized *
                            (float)PAPER_LFF_POSITION_BINS /
                            SWEEP_TWO_PI);
  if (bin >= PAPER_LFF_POSITION_BINS) {
    bin = PAPER_LFF_POSITION_BINS - 1U;
  }
  return (uint16_t)bin;
}

static void ResetRevolutionAccumulators(void) {
  revolution_speed_sum = 0.0f;
  revolution_speed_sample_count = 0U;
  revolution_cos_sum = 0.0f;
  revolution_sin_sum = 0.0f;
  revolution_unique_bin_count = 0U;
  revolution_invalid = 0U;
  active_position_bin = 0U;
  active_position_bin_valid = 0U;
  active_position_bin_speed_sum = 0.0f;
  active_position_bin_sample_count = 0U;
  memset(revolution_bin_seen, 0, sizeof(revolution_bin_seen));
}

static void ClearStageAccumulators(void) {
  stage_statistics_count = 0U;
  stage_cos_mean_rpm = 0.0f;
  stage_sin_mean_rpm = 0.0f;
  stage_cos_m2_rpm2 = 0.0f;
  stage_sin_m2_rpm2 = 0.0f;
}

static void ClearResults(void) {
  for (size_t index = 0U; index <= (size_t)PAPER_LFF_MAX_ORDER; ++index) {
    g_sweep_baseline_cos_rpm[index] = 0.0f;
    g_sweep_baseline_sin_rpm[index] = 0.0f;
    g_sweep_response_cos_rpm_per_a[index] = 0.0f;
    g_sweep_response_sin_rpm_per_a[index] = 0.0f;
    g_sweep_response_magnitude_rpm_per_a[index] = 0.0f;
    g_sweep_recommended_cos_current_a[index] = 0.0f;
    g_sweep_recommended_sin_current_a[index] = 0.0f;
    g_sweep_positive_coefficient_std_rpm[index] = 0.0f;
    g_sweep_negative_coefficient_std_rpm[index] = 0.0f;
    g_sweep_baseline_standard_error_rpm[index] = 0.0f;
    g_sweep_response_standard_error_rpm_per_a[index] = 0.0f;
    g_sweep_response_snr[index] = 0.0f;
    g_sweep_recommended_standard_error_a[index] = 0.0f;
    g_sweep_statistics_revolution_count[index] = 0U;
    g_sweep_result_valid[index] = 0U;
  }
}

static void ResetRuntimeState(void) {
  g_sweep_state = FIXED_FF_SWEEP_IDLE;
  g_sweep_active_order = 0U;
  g_sweep_stage_revolution_count = 0U;
  g_sweep_speed_ready = 0U;
  g_sweep_current_ready = 0U;
  g_sweep_complete = 0U;
  g_sweep_completed_order_count = 0U;
  g_iq_sweep_a = 0.0f;
  g_sweep_last_revolution_mean_rpm = 0.0f;
  g_sweep_last_revolution_coverage = 0.0f;
  g_sweep_rejected_revolution_count = 0U;
  previous_angle_rad = 0.0f;
  previous_angle_valid = 0U;
  stage_boundary_seen = 0U;
  positive_cos_rpm = 0.0f;
  positive_sin_rpm = 0.0f;
  positive_cos_variance_rpm2 = 0.0f;
  positive_sin_variance_rpm2 = 0.0f;
  positive_statistics_count = 0U;
  ClearStageAccumulators();
  ResetRevolutionAccumulators();
}

static void BeginStage(Experiment_FixedFF_Sweep_State state) {
  g_sweep_state = (uint8_t)state;
  g_sweep_stage_revolution_count = 0U;
  /* 阶段切换发生在过零边界，因此当前过零点就是新阶段的起点。 */
  stage_boundary_seen = 1U;
  ClearStageAccumulators();
  ResetRevolutionAccumulators();
}

static void BeginRequestedSweep(void) {
  const uint8_t start_order = ClampOrder(g_sweep_start_order_request);
  uint8_t end_order = ClampOrder(g_sweep_end_order_request);
  if (end_order < start_order) {
    end_order = start_order;
  }

  float amplitude_a = fabsf(g_sweep_injection_amplitude_request_a);
  if (!isfinite(amplitude_a) || (amplitude_a <= 0.0f)) {
    amplitude_a = EXPERIMENT_SWEEP_DEFAULT_INJECTION_A;
  }

  active_end_order = end_order;
  active_injection_amplitude_a =
      ClampFloat(amplitude_a, 0.0f, EXPERIMENT_SWEEP_MAX_INJECTION_A);
  g_sweep_start_order_request = start_order;
  g_sweep_end_order_request = end_order;
  g_sweep_injection_amplitude_request_a = active_injection_amplitude_a;

  ClearResults();
  ResetRuntimeState();
  g_sweep_active_order = start_order;
  g_sweep_state = FIXED_FF_SWEEP_SETTLE_POSITIVE;
  /* 开始时可能处在一圈中间，先等下一次正向过零再计完整圈。 */
  stage_boundary_seen = 0U;
}

static float SampleVariance(float m2, uint8_t sample_count) {
  if (sample_count < 2U) {
    return 0.0f;
  }
  return m2 / (float)(sample_count - 1U);
}

static void UpdateStageStatistics(float cos_rpm, float sin_rpm) {
  ++stage_statistics_count;
  const float count = (float)stage_statistics_count;

  const float cos_delta = cos_rpm - stage_cos_mean_rpm;
  stage_cos_mean_rpm += cos_delta / count;
  stage_cos_m2_rpm2 += cos_delta * (cos_rpm - stage_cos_mean_rpm);

  const float sin_delta = sin_rpm - stage_sin_mean_rpm;
  stage_sin_mean_rpm += sin_delta / count;
  stage_sin_m2_rpm2 += sin_delta * (sin_rpm - stage_sin_mean_rpm);
}

static void StoreOrderResult(float negative_cos_rpm,
                             float negative_sin_rpm,
                             float negative_cos_variance_rpm2,
                             float negative_sin_variance_rpm2,
                             uint8_t negative_statistics_count) {
  const uint8_t order = g_sweep_active_order;
  const float baseline_cos_rpm =
      0.5f * (positive_cos_rpm + negative_cos_rpm);
  const float baseline_sin_rpm =
      0.5f * (positive_sin_rpm + negative_sin_rpm);
  const float response_cos_rpm_per_a =
      (positive_cos_rpm - negative_cos_rpm) /
      (2.0f * active_injection_amplitude_a);
  const float response_sin_rpm_per_a =
      (positive_sin_rpm - negative_sin_rpm) /
      (2.0f * active_injection_amplitude_a);
  const float response_squared =
      (response_cos_rpm_per_a * response_cos_rpm_per_a) +
      (response_sin_rpm_per_a * response_sin_rpm_per_a);
  const float response_magnitude = sqrtf(response_squared);
  const float baseline_magnitude =
      sqrtf((baseline_cos_rpm * baseline_cos_rpm) +
            (baseline_sin_rpm * baseline_sin_rpm));

  const float positive_count = (float)positive_statistics_count;
  const float negative_count = (float)negative_statistics_count;
  float baseline_standard_error_rpm = 0.0f;
  float response_standard_error_rpm_per_a = 0.0f;
  float response_snr = 0.0f;
  float recommended_standard_error_a = 0.0f;
  float positive_coefficient_std_rpm = 0.0f;
  float negative_coefficient_std_rpm = 0.0f;
  uint8_t statistics_ready = 0U;
  if ((positive_statistics_count >= 2U) &&
      (negative_statistics_count >= 2U)) {
    const float cos_mean_variance_rpm2 =
        (positive_cos_variance_rpm2 / positive_count) +
        (negative_cos_variance_rpm2 / negative_count);
    const float sin_mean_variance_rpm2 =
        (positive_sin_variance_rpm2 / positive_count) +
        (negative_sin_variance_rpm2 / negative_count);
    const float mean_vector_standard_error_rpm =
        sqrtf(cos_mean_variance_rpm2 + sin_mean_variance_rpm2);

    baseline_standard_error_rpm =
        0.5f * mean_vector_standard_error_rpm;
    response_standard_error_rpm_per_a =
        mean_vector_standard_error_rpm /
        (2.0f * active_injection_amplitude_a);
    positive_coefficient_std_rpm =
        sqrtf(positive_cos_variance_rpm2 +
              positive_sin_variance_rpm2);
    negative_coefficient_std_rpm =
        sqrtf(negative_cos_variance_rpm2 +
              negative_sin_variance_rpm2);

    if (response_standard_error_rpm_per_a > 1.0e-12f) {
      response_snr =
          response_magnitude / response_standard_error_rpm_per_a;
    } else {
      response_snr = 1.0e30f;
    }
    statistics_ready = 1U;
  }

  float recommended_cos_current_a = 0.0f;
  float recommended_sin_current_a = 0.0f;
  uint8_t result_valid = 0U;
  if (isfinite(response_squared) &&
      (response_magnitude >= EXPERIMENT_SWEEP_MIN_RESPONSE_RPM_PER_A)) {
    recommended_cos_current_a =
        -((baseline_cos_rpm * response_cos_rpm_per_a) +
          (baseline_sin_rpm * response_sin_rpm_per_a)) /
        response_squared;
    recommended_sin_current_a =
        ((baseline_cos_rpm * response_sin_rpm_per_a) -
         (baseline_sin_rpm * response_cos_rpm_per_a)) /
        response_squared;
    const float recommended_magnitude =
        sqrtf((recommended_cos_current_a * recommended_cos_current_a) +
              (recommended_sin_current_a * recommended_sin_current_a));
    if (statistics_ready != 0U) {
      const float baseline_uncertainty_a =
          baseline_standard_error_rpm / response_magnitude;
      const float response_uncertainty_a =
          (baseline_magnitude * response_standard_error_rpm_per_a) /
          response_squared;
      recommended_standard_error_a =
          sqrtf((baseline_uncertainty_a * baseline_uncertainty_a) +
                (response_uncertainty_a * response_uncertainty_a));
    }
    if ((statistics_ready != 0U) && isfinite(recommended_magnitude) &&
        isfinite(recommended_standard_error_a) &&
        (recommended_magnitude <=
         EXPERIMENT_SWEEP_MAX_RECOMMENDED_ORDER_CURRENT_A) &&
        (response_snr >= EXPERIMENT_SWEEP_MIN_RESPONSE_SNR) &&
        (recommended_standard_error_a <=
         EXPERIMENT_SWEEP_MAX_RECOMMENDED_STANDARD_ERROR_A)) {
      result_valid = 1U;
    }
  }

  g_sweep_baseline_cos_rpm[order] = baseline_cos_rpm;
  g_sweep_baseline_sin_rpm[order] = baseline_sin_rpm;
  g_sweep_response_cos_rpm_per_a[order] = response_cos_rpm_per_a;
  g_sweep_response_sin_rpm_per_a[order] = response_sin_rpm_per_a;
  g_sweep_response_magnitude_rpm_per_a[order] = response_magnitude;
  g_sweep_recommended_cos_current_a[order] = recommended_cos_current_a;
  g_sweep_recommended_sin_current_a[order] = recommended_sin_current_a;
  g_sweep_positive_coefficient_std_rpm[order] =
      positive_coefficient_std_rpm;
  g_sweep_negative_coefficient_std_rpm[order] =
      negative_coefficient_std_rpm;
  g_sweep_baseline_standard_error_rpm[order] =
      baseline_standard_error_rpm;
  g_sweep_response_standard_error_rpm_per_a[order] =
      response_standard_error_rpm_per_a;
  g_sweep_response_snr[order] = response_snr;
  g_sweep_recommended_standard_error_a[order] =
      recommended_standard_error_a;
  g_sweep_statistics_revolution_count[order] =
      (positive_statistics_count < negative_statistics_count)
          ? positive_statistics_count
          : negative_statistics_count;
  g_sweep_result_valid[order] = result_valid;
  ++g_sweep_completed_order_count;
}

static void FinishMeasuredStage(void) {
  const float measured_cos_rpm = stage_cos_mean_rpm;
  const float measured_sin_rpm = stage_sin_mean_rpm;
  const float measured_cos_variance_rpm2 =
      SampleVariance(stage_cos_m2_rpm2, stage_statistics_count);
  const float measured_sin_variance_rpm2 =
      SampleVariance(stage_sin_m2_rpm2, stage_statistics_count);

  if (g_sweep_state == FIXED_FF_SWEEP_MEASURE_POSITIVE) {
    positive_cos_rpm = measured_cos_rpm;
    positive_sin_rpm = measured_sin_rpm;
    positive_cos_variance_rpm2 = measured_cos_variance_rpm2;
    positive_sin_variance_rpm2 = measured_sin_variance_rpm2;
    positive_statistics_count = stage_statistics_count;
    BeginStage(FIXED_FF_SWEEP_SETTLE_NEGATIVE);
    return;
  }

  StoreOrderResult(measured_cos_rpm,
                   measured_sin_rpm,
                   measured_cos_variance_rpm2,
                   measured_sin_variance_rpm2,
                   stage_statistics_count);
  if (g_sweep_active_order >= active_end_order) {
    g_sweep_state = FIXED_FF_SWEEP_COMPLETE;
    g_sweep_complete = 1U;
    g_sweep_enable_request = 0U;
    g_sweep_stage_revolution_count = 0U;
    g_iq_sweep_a = 0.0f;
    previous_angle_valid = 0U;
    ClearStageAccumulators();
    ResetRevolutionAccumulators();
    return;
  }

  ++g_sweep_active_order;
  positive_cos_rpm = 0.0f;
  positive_sin_rpm = 0.0f;
  positive_cos_variance_rpm2 = 0.0f;
  positive_sin_variance_rpm2 = 0.0f;
  positive_statistics_count = 0U;
  BeginStage(FIXED_FF_SWEEP_SETTLE_POSITIVE);
}

static void MarkBinSeen(uint16_t bin) {
  if (revolution_bin_seen[bin] == 0U) {
    revolution_bin_seen[bin] = 1U;
    ++revolution_unique_bin_count;
  }
}

static void FinalizeActiveMeasurementBin(void) {
  if ((active_position_bin_valid == 0U) ||
      (active_position_bin_sample_count == 0U)) {
    return;
  }

  const uint16_t bin = active_position_bin;
  if (revolution_bin_seen[bin] == 0U) {
    const float bin_mean_speed_rpm =
        active_position_bin_speed_sum /
        (float)active_position_bin_sample_count;
    const float bin_center_rad =
        (((float)bin + 0.5f) * SWEEP_TWO_PI) /
        (float)PAPER_LFF_POSITION_BINS;
    const float order_angle = (float)g_sweep_active_order * bin_center_rad;
    revolution_cos_sum += bin_mean_speed_rpm * cosf(order_angle);
    revolution_sin_sum += bin_mean_speed_rpm * sinf(order_angle);
    MarkBinSeen(bin);
  }

  active_position_bin_valid = 0U;
  active_position_bin_speed_sum = 0.0f;
  active_position_bin_sample_count = 0U;
}

static void RecordRevolutionSample(float angle_rad, float speed_rpm) {
  revolution_speed_sum += speed_rpm;
  ++revolution_speed_sample_count;

  const uint16_t bin = PositionBinFromAngle(angle_rad);
  if (IsMeasurementState() == 0U) {
    MarkBinSeen(bin);
    return;
  }

  if (active_position_bin_valid == 0U) {
    active_position_bin = bin;
    active_position_bin_valid = 1U;
    active_position_bin_speed_sum = speed_rpm;
    active_position_bin_sample_count = 1U;
    return;
  }

  if (bin == active_position_bin) {
    active_position_bin_speed_sum += speed_rpm;
    ++active_position_bin_sample_count;
    return;
  }

  FinalizeActiveMeasurementBin();
  active_position_bin = bin;
  active_position_bin_valid = 1U;
  active_position_bin_speed_sum = speed_rpm;
  active_position_bin_sample_count = 1U;
}

static uint8_t RevolutionQualityIsValid(void) {
  if (revolution_speed_sample_count == 0U) {
    g_sweep_last_revolution_mean_rpm = 0.0f;
    g_sweep_last_revolution_coverage = 0.0f;
    return 0U;
  }

  g_sweep_last_revolution_mean_rpm =
      revolution_speed_sum / (float)revolution_speed_sample_count;
  g_sweep_last_revolution_coverage =
      (float)revolution_unique_bin_count /
      (float)PAPER_LFF_POSITION_BINS;

  if (revolution_invalid != 0U) {
    return 0U;
  }
  if (fabsf(g_sweep_last_revolution_mean_rpm -
            EXPERIMENT_SWEEP_TARGET_SPEED_RPM) >
      EXPERIMENT_SWEEP_REV_MEAN_TOLERANCE_RPM) {
    return 0U;
  }
  if (g_sweep_last_revolution_coverage <
      EXPERIMENT_SWEEP_MIN_POSITION_COVERAGE) {
    return 0U;
  }
  return 1U;
}

static void AdvanceAfterAcceptedRevolution(void) {
  ++g_sweep_stage_revolution_count;
  switch ((Experiment_FixedFF_Sweep_State)g_sweep_state) {
    case FIXED_FF_SWEEP_SETTLE_POSITIVE:
      if (g_sweep_stage_revolution_count >=
          EXPERIMENT_SWEEP_SETTLE_REVOLUTIONS) {
        BeginStage(FIXED_FF_SWEEP_MEASURE_POSITIVE);
      }
      break;
    case FIXED_FF_SWEEP_MEASURE_POSITIVE:
      if (g_sweep_stage_revolution_count >=
          EXPERIMENT_SWEEP_MEASURE_REVOLUTIONS) {
        FinishMeasuredStage();
      }
      break;
    case FIXED_FF_SWEEP_SETTLE_NEGATIVE:
      if (g_sweep_stage_revolution_count >=
          EXPERIMENT_SWEEP_SETTLE_REVOLUTIONS) {
        BeginStage(FIXED_FF_SWEEP_MEASURE_NEGATIVE);
      }
      break;
    case FIXED_FF_SWEEP_MEASURE_NEGATIVE:
      if (g_sweep_stage_revolution_count >=
          EXPERIMENT_SWEEP_MEASURE_REVOLUTIONS) {
        FinishMeasuredStage();
      }
      break;
    default:
      break;
  }
}

static void HandleMechanicalWrap(void) {
  FinalizeActiveMeasurementBin();

  /* 第一次过零只建立完整圈边界，不评价开始前的残缺圈。 */
  if (stage_boundary_seen == 0U) {
    stage_boundary_seen = 1U;
    ResetRevolutionAccumulators();
    return;
  }

  const uint8_t valid_revolution = RevolutionQualityIsValid();
  g_sweep_speed_ready = valid_revolution;
  if (valid_revolution != 0U) {
    if (IsMeasurementState() != 0U) {
      const float coefficient_scale =
          2.0f / (float)revolution_unique_bin_count;
      UpdateStageStatistics(coefficient_scale * revolution_cos_sum,
                            coefficient_scale * revolution_sin_sum);
    }
    AdvanceAfterAcceptedRevolution();
  } else {
    ++g_sweep_rejected_revolution_count;
  }

  ResetRevolutionAccumulators();
}

void Experiment_FixedFFSweep_Init(void) {
  g_sweep_enable_request = 0U;
  g_sweep_reset_request = 0U;
  g_sweep_start_order_request = EXPERIMENT_SWEEP_START_ORDER;
  g_sweep_end_order_request = EXPERIMENT_SWEEP_END_ORDER;
  g_sweep_injection_amplitude_request_a =
      EXPERIMENT_SWEEP_DEFAULT_INJECTION_A;
  ClearResults();
  ResetRuntimeState();
}

void Experiment_FixedFFSweep_Stop(void) {
  g_sweep_enable_request = 0U;
  g_iq_sweep_a = 0.0f;
  g_sweep_speed_ready = 0U;
  g_sweep_current_ready = 0U;
  previous_angle_valid = 0U;
  ClearStageAccumulators();
  ResetRevolutionAccumulators();
  if (g_sweep_state != FIXED_FF_SWEEP_COMPLETE) {
    g_sweep_state = FIXED_FF_SWEEP_IDLE;
    g_sweep_active_order = 0U;
    g_sweep_stage_revolution_count = 0U;
  }
}

void Experiment_FixedFFSweep_Background(void) {
  if (g_sweep_reset_request != 0U) {
    g_sweep_enable_request = 0U;
    ClearResults();
    ResetRuntimeState();
    g_sweep_reset_request = 0U;
  }
}

float Experiment_FixedFFSweep_Update(float mechanical_angle_rad,
                                     float speed_rpm,
                                     float feedback_iq_a) {
  if (g_sweep_enable_request == 0U) {
    g_iq_sweep_a = 0.0f;
    g_sweep_speed_ready = 0U;
    g_sweep_current_ready = 0U;
    previous_angle_valid = 0U;
    if ((g_sweep_state != FIXED_FF_SWEEP_IDLE) &&
        (g_sweep_state != FIXED_FF_SWEEP_COMPLETE)) {
      g_sweep_state = FIXED_FF_SWEEP_IDLE;
      g_sweep_active_order = 0U;
      g_sweep_stage_revolution_count = 0U;
      stage_boundary_seen = 0U;
      ClearStageAccumulators();
      ResetRevolutionAccumulators();
    }
    return 0.0f;
  }

  if (!isfinite(mechanical_angle_rad) || !isfinite(speed_rpm) ||
      !isfinite(feedback_iq_a)) {
    g_iq_sweep_a = 0.0f;
    g_sweep_current_ready = 0U;
    previous_angle_valid = 0U;
    revolution_invalid = 1U;
    return 0.0f;
  }

  if ((g_sweep_state == FIXED_FF_SWEEP_IDLE) ||
      (g_sweep_state == FIXED_FF_SWEEP_COMPLETE)) {
    BeginRequestedSweep();
  }

  const float normalized_angle_rad =
      NormalizeMechanicalAngle(mechanical_angle_rad);

  /* 过零判定不因瞬时转速波动而复位，保证总能形成完整圈。 */
  if (previous_angle_valid != 0U) {
    if ((previous_angle_rad - normalized_angle_rad) > SWEEP_PI) {
      HandleMechanicalWrap();
    }
  } else {
    previous_angle_valid = 1U;
  }
  previous_angle_rad = normalized_angle_rad;

  if (g_sweep_state == FIXED_FF_SWEEP_COMPLETE) {
    g_iq_sweep_a = 0.0f;
    return 0.0f;
  }

  const uint8_t speed_inside_hard_boundary =
      ((speed_rpm >= EXPERIMENT_SWEEP_HARD_SPEED_MIN_RPM) &&
       (speed_rpm <= EXPERIMENT_SWEEP_HARD_SPEED_MAX_RPM))
          ? 1U
          : 0U;
  const float feedback_limit_a =
      EXPERIMENT_IQ_REFERENCE_MAX_A - active_injection_amplitude_a -
      EXPERIMENT_SWEEP_CURRENT_HEADROOM_A;
  const uint8_t current_ready =
      (fabsf(feedback_iq_a) <= feedback_limit_a) ? 1U : 0U;
  g_sweep_current_ready = current_ready;

  if ((speed_inside_hard_boundary == 0U) || (current_ready == 0U)) {
    /* 当前圈作废，但保留过零和阶段状态；下一圈可自动恢复。 */
    revolution_invalid = 1U;
  }

  RecordRevolutionSample(normalized_angle_rad, speed_rpm);

  if ((speed_inside_hard_boundary == 0U) || (current_ready == 0U)) {
    g_iq_sweep_a = 0.0f;
    return 0.0f;
  }

  const Experiment_FixedFF_Sweep_State state =
      (Experiment_FixedFF_Sweep_State)g_sweep_state;
  const float order_angle =
      (float)g_sweep_active_order * normalized_angle_rad;
  float injection_a = active_injection_amplitude_a * cosf(order_angle);
  if ((state == FIXED_FF_SWEEP_SETTLE_NEGATIVE) ||
      (state == FIXED_FF_SWEEP_MEASURE_NEGATIVE)) {
    injection_a = -injection_a;
  }

  g_iq_sweep_a = injection_a;
  return injection_a;
}
