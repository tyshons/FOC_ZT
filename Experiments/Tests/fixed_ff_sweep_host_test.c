#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "Experiment_FixedFF_Sweep.h"

#define TEST_PI 3.14159265358979323846f
#define TEST_TWO_PI (2.0f * TEST_PI)

static int NearlyEqual(float actual, float expected, float tolerance) {
  return fabsf(actual - expected) <= tolerance;
}

static void RunSweep(float revolution_noise_rpm,
                     uint8_t inject_bad_sample) {
  Experiment_FixedFFSweep_Init();
  g_sweep_start_order_request = 1U;
  g_sweep_end_order_request = 1U;
  g_sweep_injection_amplitude_request_a = 0.03f;
  g_sweep_enable_request = 1U;

  const float paused_injection_a = Experiment_FixedFFSweep_Update(
      0.0f, 5.0f, EXPERIMENT_IQ_REFERENCE_MAX_A - 0.01f);
  assert(paused_injection_a == 0.0f);
  assert(g_sweep_current_ready == 0U);

  const float test_sample_rate_hz = 400.0f;
  const uint32_t maximum_samples = 28U * 5000U;
  float theta_rad = 0.01f;
  float previous_injection_a = 0.0f;
  uint8_t bad_sample_done = 0U;
  uint32_t revolution_index = 0U;

  for (uint32_t sample = 0U;
       (sample < maximum_samples) && (g_sweep_complete == 0U);
       ++sample) {
    const float alternating_noise_rpm =
        ((revolution_index & 1U) == 0U)
            ? revolution_noise_rpm
            : -revolution_noise_rpm;
    float speed_rpm =
        5.0f + ((0.20f + alternating_noise_rpm) * cosf(theta_rad)) +
                      (2.00f * previous_injection_a);

    /* 单个异常样本只淘汰当前圈，不得让状态机永久回到圈边界等待。 */
    if ((inject_bad_sample != 0U) && (bad_sample_done == 0U) &&
        (g_sweep_state == FIXED_FF_SWEEP_MEASURE_POSITIVE) &&
        (g_sweep_stage_revolution_count == 0U) &&
        (theta_rad > TEST_PI)) {
      speed_rpm = 7.0f;
      bad_sample_done = 1U;
    }

    previous_injection_a =
        Experiment_FixedFFSweep_Update(theta_rad, speed_rpm, 0.20f);
    theta_rad += speed_rpm * TEST_TWO_PI / (60.0f * test_sample_rate_hz);
    while (theta_rad >= TEST_TWO_PI) {
      theta_rad -= TEST_TWO_PI;
      ++revolution_index;
    }
  }

  assert(bad_sample_done == inject_bad_sample);
  assert(g_sweep_complete == 1U);
  assert(g_sweep_completed_order_count == 1U);
  assert(g_sweep_statistics_revolution_count[1U] ==
         EXPERIMENT_SWEEP_MEASURE_REVOLUTIONS);
  assert(g_iq_sweep_a == 0.0f);
}

int main(void) {
  /*
   * 被控对象含有一阶位置纹波，并且角度步长随转速变化。
   * 这种非均匀时间采样会使旧的等时间投影产生偏差，等角度投影应仍能恢复系数。
   */
  RunSweep(0.0f, 1U);

  assert(g_sweep_rejected_revolution_count >= 1U);
  assert(g_sweep_result_valid[1U] == 1U);
  assert(g_sweep_last_revolution_coverage >=
         EXPERIMENT_SWEEP_MIN_POSITION_COVERAGE);
  assert(NearlyEqual(g_sweep_baseline_cos_rpm[1U], 0.20f, 0.012f));
  assert(NearlyEqual(g_sweep_baseline_sin_rpm[1U], 0.0f, 0.012f));
  assert(NearlyEqual(g_sweep_response_cos_rpm_per_a[1U], 2.00f, 0.05f));
  assert(NearlyEqual(g_sweep_response_sin_rpm_per_a[1U], 0.0f, 0.05f));
  assert(NearlyEqual(g_sweep_recommended_cos_current_a[1U], -0.10f, 0.012f));
  assert(NearlyEqual(g_sweep_recommended_sin_current_a[1U], 0.0f, 0.012f));
  assert(g_sweep_response_snr[1U] >= EXPERIMENT_SWEEP_MIN_RESPONSE_SNR);
  assert(g_sweep_recommended_standard_error_a[1U] <=
         EXPERIMENT_SWEEP_MAX_RECOMMENDED_STANDARD_ERROR_A);

  /* 逐圈交替变化的纹波虽然平均值正确，但方差过大时结果必须判为不可用。 */
  RunSweep(0.50f, 0U);
  assert(g_sweep_result_valid[1U] == 0U);
  assert(g_sweep_positive_coefficient_std_rpm[1U] > 0.40f);
  assert(g_sweep_negative_coefficient_std_rpm[1U] > 0.40f);
  assert((g_sweep_response_snr[1U] < EXPERIMENT_SWEEP_MIN_RESPONSE_SNR) ||
         (g_sweep_recommended_standard_error_a[1U] >
          EXPERIMENT_SWEEP_MAX_RECOMMENDED_STANDARD_ERROR_A));

  puts("fixed_ff_sweep_host_test: PASS");
  return 0;
}
