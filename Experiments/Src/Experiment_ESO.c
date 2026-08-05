#include "Experiment_ESO.h"

#include <math.h>
#include <stdint.h>
#include "Experiment_Config.h"

/* z1为速度估计，z2为折算到角加速度的总残余扰动估计。 */
static float eso_z1_rad_s = 0.0f;
static float eso_z2_rad_s2 = 0.0f;
static uint8_t eso_initialized = 0U;

/* 运行时参数由通信层设置，4 kHz更新入口只读取本地副本。 */
static volatile float eso_bandwidth_rad_s = PAPER_ESO_BANDWIDTH_RAD_S;
static volatile float eso_compensation_gain =
    EXPERIMENT_ESO_COMPENSATION_GAIN_DEFAULT;
static volatile float eso_current_limit_a =
    EXPERIMENT_ESO_CURRENT_LIMIT_DEFAULT_A;
static volatile uint32_t eso_saturation_count = 0U;
static volatile uint32_t eso_update_count = 0U;

static float ClampSymmetric(float value, float limit) {
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

void Experiment_ESO_Init(void) {
  eso_bandwidth_rad_s = PAPER_ESO_BANDWIDTH_RAD_S;
  eso_compensation_gain = EXPERIMENT_ESO_COMPENSATION_GAIN_DEFAULT;
  eso_current_limit_a = EXPERIMENT_ESO_CURRENT_LIMIT_DEFAULT_A;
  Experiment_ESO_Reset();
}

void Experiment_ESO_Reset(void) {
  /* 不能保留上一模式的积分状态，否则切换时会造成补偿电流突变。 */
  eso_z1_rad_s = 0.0f;
  eso_z2_rad_s2 = 0.0f;
  eso_initialized = 0U;
  eso_saturation_count = 0U;
  eso_update_count = 0U;
}

uint8_t Experiment_ESO_Configure(float bandwidth_rad_s,
                                 float compensation_gain,
                                 float current_limit_a) {
  if (!isfinite(bandwidth_rad_s) || !isfinite(compensation_gain) ||
      !isfinite(current_limit_a)) {
    return 0U;
  }
  if ((bandwidth_rad_s < EXPERIMENT_ESO_BANDWIDTH_MIN_RAD_S) ||
      (bandwidth_rad_s > EXPERIMENT_ESO_BANDWIDTH_MAX_RAD_S) ||
      (compensation_gain < 0.0f) ||
      (compensation_gain > EXPERIMENT_ESO_COMPENSATION_GAIN_MAX) ||
      (current_limit_a < 0.0f) ||
      (current_limit_a > EXPERIMENT_ESO_CURRENT_LIMIT_MAX_A)) {
    return 0U;
  }

  eso_bandwidth_rad_s = bandwidth_rad_s;
  eso_compensation_gain = compensation_gain;
  eso_current_limit_a = current_limit_a;
  Experiment_ESO_Reset();
  return 1U;
}

Experiment_Eso_Config Experiment_ESO_GetConfig(void) {
  const Experiment_Eso_Config config = {
      eso_bandwidth_rad_s,
      eso_compensation_gain,
      eso_current_limit_a,
  };
  return config;
}

Experiment_Eso_Output Experiment_ESO_Update(float speed_rad_s,
                                            float applied_iq_a,
                                            float disturbance_hat_nm) {
  Experiment_Eso_Output out = {0};
  /* 输入无效时不更新积分器，并让上层回退到无ESO补偿的输出。 */
  if (!isfinite(speed_rad_s) || !isfinite(applied_iq_a) ||
      !isfinite(disturbance_hat_nm)) {
    Experiment_ESO_Reset();
    return out;
  }

  /* 将机械模型写成：omega_dot = -a*omega + b0*i_q + disturbance。 */
  const float inertia = TURNTABLE_EFFECTIVE_INERTIA_KGM2;
  const float torque_constant = MOTOR_TORQUE_CONSTANT_NM_PER_A;
  const float a = MOTOR_VISCOUS_DAMPING_NMS / inertia;
  const float b0 = torque_constant / inertia;
  const float bandwidth_rad_s = eso_bandwidth_rad_s;
  const float compensation_gain = eso_compensation_gain;
  const float current_limit_a = eso_current_limit_a;
  const float beta1 = 2.0f * bandwidth_rad_s;
  const float beta2 = bandwidth_rad_s * bandwidth_rad_s;

  if (eso_initialized == 0U) {
    /* 首次运行以实测速度对齐z1，防止冷启动产生大的观测误差。 */
    eso_z1_rad_s = speed_rad_s;
    eso_z2_rad_s2 = 0.0f;
    eso_initialized = 1U;
  }

  /* 线性ESO：误差经beta1/beta2分别校正速度与总扰动状态。 */
  const float speed_error = speed_rad_s - eso_z1_rad_s;
  const float z1_dot =
      -a * eso_z1_rad_s +
      beta1 * speed_error +
      b0 * applied_iq_a -
      disturbance_hat_nm / inertia +
      eso_z2_rad_s2;
  const float z2_dot = beta2 * speed_error;

  /* 使用速度环固定采样周期的前向欧拉离散化。 */
  eso_z1_rad_s += FOC_SPEED_LOOP_PERIOD_S * z1_dot;
  eso_z2_rad_s2 += FOC_SPEED_LOOP_PERIOD_S * z2_dot;

  if (!isfinite(eso_z1_rad_s) || !isfinite(eso_z2_rad_s2)) {
    eso_z1_rad_s = speed_rad_s;
    eso_z2_rad_s2 = 0.0f;
    eso_saturation_count = 0U;
    eso_update_count = 0U;
  }

  /*
   * 原始补偿电流只限制观测状态的极端值；实验增益和独立输出限幅不会
   * 反向写入z2，因此运行时调小补偿不会破坏残余扰动估计。
   */
  float raw_compensation_current_a = -eso_z2_rad_s2 / b0;
  raw_compensation_current_a =
      ClampSymmetric(raw_compensation_current_a,
                     EXPERIMENT_ESO_OBSERVER_CURRENT_LIMIT_A);
  eso_z2_rad_s2 = -b0 * raw_compensation_current_a;

  const float requested_compensation_current_a =
      compensation_gain * raw_compensation_current_a;
  const float compensation_current_a =
      ClampSymmetric(requested_compensation_current_a, current_limit_a);
  const uint8_t compensation_saturated =
      (fabsf(requested_compensation_current_a) > current_limit_a) ? 1U : 0U;

  if (eso_update_count < UINT32_MAX) {
    eso_update_count++;
  }
  if ((compensation_saturated != 0U) &&
      (eso_saturation_count < UINT32_MAX)) {
    eso_saturation_count++;
  }

  out.compensation_current_a = compensation_current_a;
  out.raw_compensation_current_a = raw_compensation_current_a;
  out.residual_torque_nm = -inertia * eso_z2_rad_s2;
  out.z1_rad_s = eso_z1_rad_s;
  out.z2_rad_s2 = eso_z2_rad_s2;
  out.speed_error_rad_s = speed_error;
  out.saturation_count = eso_saturation_count;
  out.update_count = eso_update_count;
  out.compensation_saturated = compensation_saturated;
  return out;
}
