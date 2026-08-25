#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "Experiment_Config.h"
#include "Experiment_Baseline.h"
#include "Experiment_Control.h"
#include "Experiment_ESO.h"
#include "Experiment_FixedFeedforward.h"
#include "Experiment_LearningFeedforward.h"

#define TEST_PI      3.14159265358979323846f
#define TEST_TWO_PI  (2.0f * TEST_PI)

static int IsClose(float actual, float expected, float tolerance) {
  return fabsf(actual - expected) <= tolerance;
}

static float FixedFeedforwardExpected(float theta_rad) {
  return EXPERIMENT_FIXED_FF_GAIN *
         (EXPERIMENT_FIXED_FF_ORDER1_COS_CURRENT_A * cosf(theta_rad) +
          EXPERIMENT_FIXED_FF_ORDER1_SIN_CURRENT_A * sinf(theta_rad) +
          EXPERIMENT_FIXED_FF_ORDER2_COS_CURRENT_A * cosf(2.0f * theta_rad) +
          EXPERIMENT_FIXED_FF_ORDER2_SIN_CURRENT_A * sinf(2.0f * theta_rad) +
          EXPERIMENT_FIXED_FF_ORDER4_COS_CURRENT_A * cosf(4.0f * theta_rad) +
          EXPERIMENT_FIXED_FF_ORDER4_SIN_CURRENT_A * sinf(4.0f * theta_rad));
}

static void CompleteLearningProfileRevolutions(uint32_t revolutions,
                                               Position_Lff_Mode mode,
                                               float dc_torque_nm,
                                               float sine_torque_nm) {
  const uint32_t samples_per_revolution = 4800U;
  const uint32_t total_samples =
      revolutions * samples_per_revolution + 1U;

  for (uint32_t sample = 0U; sample < total_samples; ++sample) {
    const uint32_t within_revolution = sample % samples_per_revolution;
    const float theta =
        TEST_TWO_PI * (float)within_revolution /
        (float)samples_per_revolution;
    const float repeatable_residual =
        dc_torque_nm + sine_torque_nm * sinf(theta);
    Position_Learning_Sample(theta, repeatable_residual, 1U, mode);
    Position_Learning_Background();
  }
}

static void CompleteLearningRevolutions(uint32_t revolutions,
                                        Position_Lff_Mode mode) {
  CompleteLearningProfileRevolutions(revolutions, mode, 0.0f, 0.04f);
}

int main(void) {
  Experiment_Control_Init();

  /* 所有论文扩展路径关闭时，应保留已验证的速度PI输出。 */
  const float baseline = Experiment_Control_Update(90.0f, 5.0f, 0.3f);
  assert(IsClose(baseline, 0.3f, 1.0e-6f));
  assert(IsClose(g_iq_fixed_feedforward_a, 0.0f, 1.0e-6f));
  assert(IsClose(g_iq_learning_feedforward_a, 0.0f, 1.0e-6f));
  assert(IsClose(g_iq_eso_a, 0.0f, 1.0e-6f));

  /* ESO上电必须使用保守增益、独立限幅和已配置的默认带宽。 */
  Experiment_Eso_Config eso_config = Experiment_ESO_GetConfig();
  assert(IsClose(eso_config.bandwidth_rad_s,
                 PAPER_ESO_BANDWIDTH_RAD_S,
                 1.0e-6f));
  assert(IsClose(eso_config.compensation_gain,
                 EXPERIMENT_ESO_COMPENSATION_GAIN_DEFAULT,
                 1.0e-6f));
  assert(IsClose(eso_config.current_limit_a,
                 EXPERIMENT_ESO_CURRENT_LIMIT_DEFAULT_A,
                 1.0e-6f));
  assert(Experiment_ESO_Configure(NAN, 0.2f, 0.3f) == 0U);
  assert(Experiment_ESO_Configure(30.0f, 1.1f, 0.3f) == 0U);

  /* 固定前馈系数采用电流域约定，并包含已辨识的1、2、4阶分量。 */
  assert(IsClose(Experiment_FixedFeedforward_CurrentA(0.0f),
                 FixedFeedforwardExpected(0.0f),
                 1.0e-7f));
  assert(IsClose(Experiment_FixedFeedforward_CurrentA(0.5f * TEST_PI),
                 FixedFeedforwardExpected(0.5f * TEST_PI),
                 1.0e-7f));
  assert(IsClose(Experiment_FixedFeedforward_CurrentA(NAN),
                 0.0f,
                 1.0e-7f));

  /* 固定前馈模式必须与ESO实验相互隔离。 */
  g_experiment_mode_request = PHYSICAL_EXPERIMENT_FIXED_FEEDFORWARD;
  const float fixed_current =
      Experiment_FixedFeedforward_CurrentA(0.5f * TEST_PI);
  const float fixed_expected =
      Experiment_Baseline_ClampIq(0.3f + fixed_current);
  const float fixed_output =
      Experiment_Control_Update(90.0f, 5.0f, 0.3f);
  assert(g_experiment_active_mode ==
         PHYSICAL_EXPERIMENT_FIXED_FEEDFORWARD);
  assert(IsClose(fixed_output, fixed_expected, 1.0e-5f));
  assert(IsClose(g_iq_eso_a, 0.0f, 1.0e-6f));

  /* 未知模式请求必须安全回退到基线实验。 */
  g_experiment_mode_request = UINT8_MAX;
  const float fallback =
      Experiment_Control_Update(90.0f, 5.0f, 0.3f);
  assert(g_experiment_active_mode == PHYSICAL_EXPERIMENT_BASELINE);
  assert(IsClose(fallback, 0.3f, 1.0e-6f));

  /* 普通学习在首次完整覆盖一圈后更新。 */
  Position_Learning_ResetAll();
  CompleteLearningRevolutions(1U, POSITION_LFF_MODE_ORDINARY);
  assert(g_lff_revolution_count == 1U);
  assert(g_lff_covered_revolution_count == 1U);
  assert(g_lff_update_count == 1U);
  assert(g_lff_last_coverage >= PAPER_LFF_MIN_COVERAGE);
  assert(g_lff_selected_order_count ==
         PAPER_LFF_MAX_ORDER - PAPER_LFF_MIN_ORDER + 1U);
  assert(IsClose(g_lff_rho_orders[0], 0.0f, 1.0e-7f));
  assert(g_lff_table_rms_nm > 0.0f);
  assert(g_lff_table_peak_abs_nm > g_lff_table_rms_nm);
  const float learned_quadrature =
      Position_Learning_GetOutput(0.5f * TEST_PI);
  assert(IsClose(learned_quadrature, 0.01f, 7.5e-4f));

  /* 与论文一致：0阶只用于诊断，不能进入学习输出；1~40阶表保持零均值。 */
  Position_Learning_ResetAll();
  CompleteLearningProfileRevolutions(
      1U, POSITION_LFF_MODE_ORDINARY, 0.12f, 0.04f);
  assert(IsClose(g_lff_last_residual_a_nm[0], 0.12f, 7.5e-4f));
  assert(IsClose(g_lff_dc_torque_nm, 0.0f, 1.0e-7f));
  assert(fabsf(g_lff_table_mean_nm) < 1.0e-5f);
  assert(IsClose(Position_Learning_GetOutput(0.5f * TEST_PI),
                 0.01f,
                 1.5e-3f));
  assert(IsClose(Position_Learning_GetOutput(0.0f), 0.0f, 1.5e-3f));
  assert(IsClose(g_lff_learned_a_nm[0], 0.0f, 1.0e-7f));
  assert(IsClose(g_lff_rho_orders[0], 0.0f, 1.0e-7f));

  /* 选择性学习需等到连续系数对数量达到配置值后才提交学习表。 */
  Position_Learning_ResetAll();
  CompleteLearningRevolutions(4U, POSITION_LFF_MODE_SELECTIVE);
  assert(g_lff_update_count == 0U);
  CompleteLearningRevolutions(1U, POSITION_LFF_MODE_SELECTIVE);
  assert(g_lff_update_count == 1U);
  assert(IsClose(g_lff_rho_orders[0], 0.0f, 1.0e-7f));
  assert(g_lff_rho_orders[1] >= PAPER_LFF_RHO_THRESHOLD);
  assert(g_lff_selected_order_count >= 1U);

  /* 全局相关学习同样只能汇总1~40阶，稳定直流负载不得进入输出。 */
  Position_Learning_ResetAll();
  CompleteLearningProfileRevolutions(
      4U, POSITION_LFF_MODE_GLOBAL, 0.12f, 0.04f);
  assert(g_lff_update_count == 0U);
  CompleteLearningProfileRevolutions(
      1U, POSITION_LFF_MODE_GLOBAL, 0.12f, 0.04f);
  assert(g_lff_update_count == 1U);
  assert(g_lff_global_rho >= PAPER_LFF_RHO_THRESHOLD);
  assert(IsClose(g_lff_rho_orders[0], 0.0f, 1.0e-7f));
  assert(IsClose(g_lff_dc_torque_nm, 0.0f, 1.0e-7f));
  assert(IsClose(Position_Learning_GetOutput(0.5f * TEST_PI),
                 0.01f,
                 1.5e-3f));

  /* ESO符号约定必须能够估计并抵消阻力转矩。 */
  Experiment_Control_Init();
  g_experiment_mode_request = PHYSICAL_EXPERIMENT_ESO;
  float plant_speed_rad_s = 0.0f;
  float plant_theta_rad = 0.0f;
  const float applied_disturbance_nm = 0.03f;
  for (uint32_t sample = 0U; sample < 4000U; ++sample) {
    const float plant_speed_rpm =
        plant_speed_rad_s * 60.0f / TEST_TWO_PI;
    const float iq_command =
        Experiment_Control_Update(plant_theta_rad * 180.0f / TEST_PI,
                                  plant_speed_rpm,
                                  0.0f);
    const float acceleration =
        (MOTOR_TORQUE_CONSTANT_NM_PER_A * iq_command -
         MOTOR_VISCOUS_DAMPING_NMS * plant_speed_rad_s -
         applied_disturbance_nm) /
        TURNTABLE_EFFECTIVE_INERTIA_KGM2;
    plant_speed_rad_s += FOC_SPEED_LOOP_PERIOD_S * acceleration;
    plant_theta_rad += FOC_SPEED_LOOP_PERIOD_S * plant_speed_rad_s;
  }
  assert(IsClose(g_residual_disturbance_nm,
                 applied_disturbance_nm,
                 2.0e-3f));
  assert(g_iq_eso_a > 0.0f);

  /* 独立限幅只能限制实际补偿输出，不能把原始扰动估计压回同一数值。 */
  Experiment_Control_Init();
  assert(Experiment_ESO_Configure(30.0f, 1.0f, 0.05f) == 1U);
  g_experiment_mode_request = PHYSICAL_EXPERIMENT_ESO;
  plant_speed_rad_s = 0.0f;
  plant_theta_rad = 0.0f;
  for (uint32_t sample = 0U; sample < 4000U; ++sample) {
    const float plant_speed_rpm =
        plant_speed_rad_s * 60.0f / TEST_TWO_PI;
    const float iq_command =
        Experiment_Control_Update(plant_theta_rad * 180.0f / TEST_PI,
                                  plant_speed_rpm,
                                  0.0f);
    const float acceleration =
        (MOTOR_TORQUE_CONSTANT_NM_PER_A * iq_command -
         MOTOR_VISCOUS_DAMPING_NMS * plant_speed_rad_s -
         applied_disturbance_nm) /
        TURNTABLE_EFFECTIVE_INERTIA_KGM2;
    plant_speed_rad_s += FOC_SPEED_LOOP_PERIOD_S * acceleration;
    plant_theta_rad += FOC_SPEED_LOOP_PERIOD_S * plant_speed_rad_s;
  }
  assert(fabsf(g_iq_eso_a) <= 0.05f + 1.0e-6f);
  assert(fabsf(g_iq_eso_raw_a) > fabsf(g_iq_eso_a));
  assert(g_eso_saturated != 0U);
  assert(g_eso_saturation_count > 0U);
  assert(g_eso_update_count >= g_eso_saturation_count);

  /* ESO计算结果必须保持有限值，总q轴电流必须满足安全限制。 */
  Experiment_Control_Init();
  g_experiment_mode_request = PHYSICAL_EXPERIMENT_ESO;
  for (uint32_t sample = 0U; sample < 2000U; ++sample) {
    (void)Experiment_Control_Update(0.0f, 0.0f, 1.0f);
  }
  assert(isfinite(g_eso_z1_rad_s));
  assert(isfinite(g_eso_z2_rad_s2));
  assert(fabsf(g_iq_composite_a) <=
         EXPERIMENT_IQ_REFERENCE_MAX_A + 1.0e-6f);

  /* 同一种子必须生成可重复的训练扰动，冻结学习后扰动应立即撤销。 */
  float first_noise_sequence[64];
  for (uint32_t run = 0U; run < 2U; ++run) {
    Experiment_Control_Init();
    g_experiment_mode_request =
        PHYSICAL_EXPERIMENT_ESO_LEARNING_FEEDFORWARD;
    g_learning_update_request = 1U;
    g_training_noise_enable_request = 1U;
    g_training_noise_amplitude_request_a = 0.10f;
    g_training_noise_seed_request = 0x2468ACE1UL;
    for (uint32_t sample = 0U; sample < 64U; ++sample) {
      (void)Experiment_Control_Update(0.0f, 5.0f, 0.0f);
      assert(fabsf(g_iq_training_noise_a) <= 0.10f + 1.0e-6f);
      if (run == 0U) {
        first_noise_sequence[sample] = g_iq_training_noise_a;
      } else {
        assert(IsClose(g_iq_training_noise_a,
                       first_noise_sequence[sample],
                       1.0e-7f));
      }
    }
  }
  assert(fabsf(first_noise_sequence[63]) > 1.0e-4f);
  g_learning_update_request = 0U;
  (void)Experiment_Control_Update(0.0f, 5.0f, 0.0f);
  assert(IsClose(g_iq_training_noise_a, 0.0f, 1.0e-7f));

  puts("experiment_control_host_test: PASS");
  return 0;
}
