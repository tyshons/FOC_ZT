#include "Experiment_Control.h"

#include <math.h>
#include "Experiment_Baseline.h"
#include "Experiment_Config.h"
#include "Experiment_ESO.h"
#include "Experiment_FixedFeedforward.h"

#define EXPERIMENT_PI       3.14159265358979323846f
#define RPM_TO_RAD_S        (2.0f * EXPERIMENT_PI / 60.0f)
#define DEG_TO_RAD          (EXPERIMENT_PI / 180.0f)

/* 调试器写入请求量；活动量只在Update中完成安全切换后才更新。 */
volatile uint8_t g_experiment_mode_request = PHYSICAL_EXPERIMENT_BASELINE;
volatile uint8_t g_experiment_active_mode = PHYSICAL_EXPERIMENT_BASELINE;
volatile uint8_t g_learning_update_request = 0U;
volatile uint8_t g_learning_reset_request = 0U;
volatile uint8_t g_learning_mode_request = POSITION_LFF_MODE_SELECTIVE;
volatile uint8_t g_training_noise_enable_request = 0U;
volatile float g_training_noise_amplitude_request_a = 0.0f;
volatile uint32_t g_training_noise_seed_request =
    PAPER_LFF_NOISE_DEFAULT_SEED;

/* 供调试器记录的电流分量和扰动估计，单位见变量名。 */
volatile float g_iq_feedback_a = 0.0f;
volatile float g_iq_fixed_feedforward_a = 0.0f;
volatile float g_iq_learning_feedforward_a = 0.0f;
volatile float g_iq_learning_dc_a = 0.0f;
volatile float g_iq_learning_ac_a = 0.0f;
volatile float g_iq_training_noise_a = 0.0f;
volatile float g_iq_eso_raw_a = 0.0f;
volatile float g_iq_eso_a = 0.0f;
volatile float g_iq_composite_a = 0.0f;
volatile float g_fixed_disturbance_nm = 0.0f;
volatile float g_learning_disturbance_nm = 0.0f;
volatile float g_residual_disturbance_nm = 0.0f;
volatile float g_eso_z1_rad_s = 0.0f;
volatile float g_eso_z2_rad_s2 = 0.0f;
volatile float g_eso_speed_error_rad_s = 0.0f;
volatile uint32_t g_eso_saturation_count = 0U;
volatile uint32_t g_eso_update_count = 0U;
volatile uint8_t g_eso_saturated = 0U;
volatile uint8_t g_lff_learning_active = 0U;

/* ESO使用上一拍不含训练扰动的已知i_q；学习边沿用于丢弃不完整的一转。 */
static float previous_known_iq_command_a = 0.0f;
static uint8_t learning_was_active = 0U;
static uint8_t training_noise_was_active = 0U;
static uint32_t training_noise_state = PAPER_LFF_NOISE_DEFAULT_SEED;
static uint32_t training_noise_active_seed = PAPER_LFF_NOISE_DEFAULT_SEED;
static uint32_t training_noise_hold_count = 0U;
static float training_noise_target_a = 0.0f;
static float training_noise_filtered_a = 0.0f;

static void ResetTrainingNoise(void) {
  uint32_t seed = g_training_noise_seed_request;
  if (seed == 0U) {
    seed = PAPER_LFF_NOISE_DEFAULT_SEED;
  }
  training_noise_state = seed;
  training_noise_active_seed = seed;
  training_noise_hold_count = 0U;
  training_noise_target_a = 0.0f;
  training_noise_filtered_a = 0.0f;
  training_noise_was_active = 0U;
  g_iq_training_noise_a = 0.0f;
}

static float UpdateTrainingNoise(uint8_t learning_active) {
  const uint8_t noise_active =
      ((learning_active != 0U) &&
       (g_training_noise_enable_request != 0U) &&
       isfinite(g_training_noise_amplitude_request_a) &&
       (g_training_noise_amplitude_request_a > 0.0f))
          ? 1U
          : 0U;
  uint32_t requested_seed = g_training_noise_seed_request;
  if (requested_seed == 0U) {
    requested_seed = PAPER_LFF_NOISE_DEFAULT_SEED;
  }
  if (noise_active == 0U) {
    ResetTrainingNoise();
    return 0.0f;
  }
  if ((training_noise_was_active == 0U) ||
      (training_noise_active_seed != requested_seed)) {
    ResetTrainingNoise();
    training_noise_was_active = 1U;
  }

  float amplitude_a = fabsf(g_training_noise_amplitude_request_a);
  if (amplitude_a > PAPER_LFF_NOISE_MAX_CURRENT_A) {
    amplitude_a = PAPER_LFF_NOISE_MAX_CURRENT_A;
  }
  if (training_noise_hold_count == 0U) {
    uint32_t state = training_noise_state;
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    training_noise_state = state;
    training_noise_target_a =
        ((state & 1U) != 0U) ? amplitude_a : -amplitude_a;
    training_noise_hold_count = PAPER_LFF_NOISE_HOLD_UPDATES;
  } else {
    training_noise_hold_count--;
  }
  training_noise_filtered_a += PAPER_LFF_NOISE_FILTER_ALPHA *
      (training_noise_target_a - training_noise_filtered_a);
  return training_noise_filtered_a;
}

static Physical_Experiment_Mode RequestedMode(void) {
#if EXPERIMENT_RUNTIME_MODES_ENABLED == 0U
  /* 编译期锁定时忽略调试器请求，确保固件只能运行安全基线。 */
  return PHYSICAL_EXPERIMENT_BASELINE;
#else
  /* 只接受定义过的枚举值，越界请求自动回退到基线。 */
  if (g_experiment_mode_request <=
      PHYSICAL_EXPERIMENT_FIXED_FF_SWEEP) {
    return (Physical_Experiment_Mode)g_experiment_mode_request;
  }
  return PHYSICAL_EXPERIMENT_BASELINE;
#endif
}

static Position_Lff_Mode RequestedLearningMode(void) {
  /* 选择性学习是默认策略，也是非法请求的安全回退值。 */
  if (g_learning_mode_request == POSITION_LFF_MODE_ORDINARY) {
    return POSITION_LFF_MODE_ORDINARY;
  }
  if (g_learning_mode_request == POSITION_LFF_MODE_GLOBAL) {
    return POSITION_LFF_MODE_GLOBAL;
  }
  return POSITION_LFF_MODE_SELECTIVE;
}

static uint8_t ModeUsesEso(Physical_Experiment_Mode mode) {
  /* 以显式白名单决定组合模式的功能，避免枚举数值耦合。 */
  return ((mode == PHYSICAL_EXPERIMENT_ESO) ||
          (mode == PHYSICAL_EXPERIMENT_ESO_FIXED_FEEDFORWARD) ||
          (mode == PHYSICAL_EXPERIMENT_ESO_LEARNING_FEEDFORWARD) ||
          (mode == PHYSICAL_EXPERIMENT_FULL))
             ? 1U
             : 0U;
}

static uint8_t ModeUsesFixedFeedforward(Physical_Experiment_Mode mode) {
  return ((mode == PHYSICAL_EXPERIMENT_FIXED_FEEDFORWARD) ||
          (mode == PHYSICAL_EXPERIMENT_ESO_FIXED_FEEDFORWARD) ||
          (mode == PHYSICAL_EXPERIMENT_FULL))
             ? 1U
             : 0U;
}

static uint8_t ModeUsesLearningFeedforward(Physical_Experiment_Mode mode) {
  return ((mode == PHYSICAL_EXPERIMENT_ESO_LEARNING_FEEDFORWARD) ||
          (mode == PHYSICAL_EXPERIMENT_FULL))
             ? 1U
             : 0U;
}

static void ClearFastTelemetry(void) {
  /* 停用时清零可见量，避免调试器误把历史数据当作当前补偿。 */
  g_iq_feedback_a = 0.0f;
  g_iq_fixed_feedforward_a = 0.0f;
  g_iq_learning_feedforward_a = 0.0f;
  g_iq_learning_dc_a = 0.0f;
  g_iq_learning_ac_a = 0.0f;
  g_iq_training_noise_a = 0.0f;
  g_iq_eso_raw_a = 0.0f;
  g_iq_eso_a = 0.0f;
  g_iq_composite_a = 0.0f;
  g_fixed_disturbance_nm = 0.0f;
  g_learning_disturbance_nm = 0.0f;
  g_residual_disturbance_nm = 0.0f;
  g_eso_z1_rad_s = 0.0f;
  g_eso_z2_rad_s2 = 0.0f;
  g_eso_speed_error_rad_s = 0.0f;
  g_eso_saturation_count = 0U;
  g_eso_update_count = 0U;
  g_eso_saturated = 0U;
  g_iq_sweep_a = 0.0f;
}

void Experiment_Control_Init(void) {
  /* 子模块先复位，再开放基线模式，确保上电不自动注入补偿。 */
  Experiment_ESO_Init();
  Experiment_FixedFFSweep_Init();
  Position_Learning_Init();
  previous_known_iq_command_a = 0.0f;
  learning_was_active = 0U;
  ResetTrainingNoise();
  g_experiment_mode_request = PHYSICAL_EXPERIMENT_BASELINE;
  g_experiment_active_mode = PHYSICAL_EXPERIMENT_BASELINE;
  g_learning_update_request = 0U;
  g_learning_reset_request = 0U;
  g_learning_mode_request = POSITION_LFF_MODE_SELECTIVE;
  g_training_noise_enable_request = 0U;
  g_training_noise_amplitude_request_a = 0.0f;
  g_training_noise_seed_request = PAPER_LFF_NOISE_DEFAULT_SEED;
  g_lff_learning_active = 0U;
  ClearFastTelemetry();
}

void Experiment_Control_FastDisable(void) {
  /* 该函数可由快速控制路径调用，因此只做常量时间的复位和标志更新。 */
  Experiment_ESO_Reset();
  Experiment_FixedFFSweep_Stop();
  Position_Learning_AbortCollection();
  previous_known_iq_command_a = 0.0f;
  learning_was_active = 0U;
  ResetTrainingNoise();
  g_lff_learning_active = 0U;
  ClearFastTelemetry();
}

float Experiment_Control_Update(float mechanical_angle_deg,
                                float speed_rpm,
                                float feedback_iq_a) {
  const float feedback =
      Experiment_Baseline_ClampIq(feedback_iq_a);
  /* 角度或速度异常时停止所有实验状态，并仅保留已限幅的反馈输出。 */
  if (!isfinite(mechanical_angle_deg) || !isfinite(speed_rpm)) {
    Experiment_Control_FastDisable();
    return feedback;
  }

  const Physical_Experiment_Mode mode = RequestedMode();
  if (mode != (Physical_Experiment_Mode)g_experiment_active_mode) {
    /* 模式切换不能继承ESO积分或未完成的一转学习数据。 */
    if ((g_experiment_active_mode ==
         PHYSICAL_EXPERIMENT_FIXED_FF_SWEEP) &&
        (mode != PHYSICAL_EXPERIMENT_FIXED_FF_SWEEP)) {
      Experiment_FixedFFSweep_Stop();
    }
    Experiment_ESO_Reset();
    Position_Learning_AbortCollection();
    previous_known_iq_command_a = feedback;
    learning_was_active = 0U;
    ResetTrainingNoise();
    g_experiment_active_mode = (uint8_t)mode;
  }

  /* 实验模型统一采用弧度和rad/s；底层速度环仍提供度和rpm。 */
  const float theta_rad = mechanical_angle_deg * DEG_TO_RAD;
  const float speed_rad_s = speed_rpm * RPM_TO_RAD_S;
  const uint8_t use_eso = ModeUsesEso(mode);
  const uint8_t use_fixed = ModeUsesFixedFeedforward(mode);
  const uint8_t use_learning = ModeUsesLearningFeedforward(mode);
  float sweep_current_a = 0.0f;
  if (mode == PHYSICAL_EXPERIMENT_FIXED_FF_SWEEP) {
    /* 扫频模式独占额外电流注入，不与前馈或ESO叠加。 */
    sweep_current_a =
        Experiment_FixedFFSweep_Update(theta_rad,
                                      speed_rpm,
                                      feedback);
  } else {
    g_iq_sweep_a = 0.0f;
  }

  float fixed_current_a = 0.0f;
  if (use_fixed != 0U) {
    fixed_current_a =
        Experiment_FixedFeedforward_CurrentA(theta_rad);
  }
  const float fixed_torque_nm =
      fixed_current_a * MOTOR_TORQUE_CONSTANT_NM_PER_A;

  float learned_torque_nm = 0.0f;
  if ((use_learning != 0U) && (g_learning_reset_request == 0U)) {
    learned_torque_nm = Position_Learning_GetOutput(theta_rad);
  }

  Experiment_Eso_Output eso = {0};
  if (use_eso != 0U) {
    /* 传入已知前馈，使ESO只估计尚未被前馈覆盖的残余扰动。 */
    eso = Experiment_ESO_Update(speed_rad_s,
                                previous_known_iq_command_a,
                                fixed_torque_nm + learned_torque_nm);
  } else {
    Experiment_ESO_Reset();
  }

  const uint8_t learning_active =
      ((use_eso != 0U) &&
       (use_learning != 0U) &&
       (g_learning_update_request != 0U) &&
       (g_learning_reset_request == 0U))
          ? 1U
          : 0U;
  if (learning_active != learning_was_active) {
    /* 开始或暂停学习时从下一完整转重新采样，避免混合两种运行状态。 */
    Position_Learning_AbortCollection();
    learning_was_active = learning_active;
  }
  /* 快速路径只采集一转轮廓；较重的傅里叶和表更新留给后台。 */
  Position_Learning_Sample(theta_rad,
                           eso.residual_torque_nm,
                           learning_active,
                           RequestedLearningMode());

  /* 随机扰动只在学习期间生效，并且不作为ESO已知输入。 */
  const float training_noise_requested_a =
      UpdateTrainingNoise(learning_active);

  /* 将全部转矩前馈换算为q轴电流，再与反馈输出合成并统一限幅。 */
  const float learned_current_a =
      learned_torque_nm / MOTOR_TORQUE_CONSTANT_NM_PER_A;
  const float known_nonfeedback_current_a =
      fixed_current_a + learned_current_a +
      eso.compensation_current_a + sweep_current_a;
  const float known_composite =
      Experiment_Baseline_ClampIq(feedback + known_nonfeedback_current_a);
  const float composite_requested =
      feedback + known_nonfeedback_current_a + training_noise_requested_a;
  const float composite =
      Experiment_Baseline_ClampIq(composite_requested);
  previous_known_iq_command_a = known_composite;

  g_iq_feedback_a = feedback;
  g_iq_fixed_feedforward_a = fixed_current_a;
  g_iq_learning_feedforward_a = learned_current_a;
  g_iq_learning_dc_a =
      g_lff_dc_torque_nm / MOTOR_TORQUE_CONSTANT_NM_PER_A;
  g_iq_learning_ac_a = learned_current_a - g_iq_learning_dc_a;
  g_iq_training_noise_a = composite - known_composite;
  g_iq_eso_raw_a = eso.raw_compensation_current_a;
  g_iq_eso_a = eso.compensation_current_a;
  g_iq_composite_a = composite;
  g_fixed_disturbance_nm = fixed_torque_nm;
  g_learning_disturbance_nm = learned_torque_nm;
  g_residual_disturbance_nm = eso.residual_torque_nm;
  g_eso_z1_rad_s = eso.z1_rad_s;
  g_eso_z2_rad_s2 = eso.z2_rad_s2;
  g_eso_speed_error_rad_s = eso.speed_error_rad_s;
  g_eso_saturation_count = eso.saturation_count;
  g_eso_update_count = eso.update_count;
  g_eso_saturated = eso.compensation_saturated;
  g_lff_learning_active = learning_active;
  return composite;
}

void Experiment_Control_Background(void) {
  /* 复位请求在后台执行，避免清表操作破坏速度环实时性。 */
  if (g_learning_reset_request != 0U) {
    Position_Learning_ResetAll();
    g_learning_reset_request = 0U;
    learning_was_active = 0U;
    g_lff_learning_active = 0U;
  }
  Position_Learning_Background();
  Experiment_FixedFFSweep_Background();
}
