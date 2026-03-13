//
// Created by tyshon on 2026/3/6.
//

#include "tim.h"
#include "controller.h"

#include <math.h>

#include "posUpdata.h"
#include "FOC_Math.h"
#include <stdint.h>

// 水平轴（motor_id = 1）
float current_angle_sp = 0.0f;
float current_speed_sp = 0.0f;
float position_given_sp = 0.0f;
float speed_given_sp = 0.0f;
float iq_given_sp = 0.0f;

float iq_num[4], iq_den[4];
float speed_num[4], speed_den[4];
float pos_num[4], pos_den[4];

pid_state_t pos_pid_sp = {0}; // 水平轴位置环状态
pid_state_t spd_pid_sp = {0}; // 水平轴速度环状态
pid_state_t iq_pid_sp = {0};

void Control_Loop(void) {
  static float g_target_position =90.0f;
  static float g_iq_ref_from_speed = 0.0f;
  static float g_speed_from_position = 0.0f;
  static uint8_t cnt = 0;
  cnt++;

  Pos_Update(&current_angle_sp);

  if (cnt==20)
  {
    cnt = 0;

    // 水平轴控制
    speed_given_sp = position_pid(position_given_sp,&pos_pid_sp, pos_num, pos_den);
    iq_given_sp = speed_pid((speed_given_sp - current_speed_sp), &spd_pid_sp, speed_num, speed_den);
    const float pidout_sp = iq_pid(iq_given_sp, &iq_pid_sp, iq_num, iq_den);
  }
  float theta ;
  Get_Electrical_Angle(&theta);

  float ia = g_adc_current[0];
  float ib = g_adc_current[1];
  float ic = g_adc_current[2];

  float i_alpha = 0.0f , i_beta = 0.0f;
  Clark_transform(ia, ib, ic, &i_alpha, &i_beta);

  float id = 0.0f, iq = 0.0f;
  park_transform(i_alpha, i_beta, theta, &id, &iq);

  float id_ref = 0.0f, iq_ref = g_iq_ref_from_speed;
  float Ud = PID(&Id_PID, id_ref, id);
  float Uq = PID(&Iq_PID, iq_ref, iq);

  float mag = sqrtf(Ud * Ud + Uq * Uq);
  if (mag > VOLTAGE_LIMIT) {
    Ud = Ud * VOLTAGE_LIMIT / mag;
    Uq = Uq * VOLTAGE_LIMIT / mag;
  }

  float U_alpha, U_beta;
  inverse_park_transform(Ud, Uq, theta, &U_alpha, &U_beta);

  uint32_t ccrA, ccrB, ccrC;
  SVPWM_Generate(U_alpha, U_beta, g_adc_vbus, 4250, &ccrA, &ccrB, &ccrC);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccrA);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccrB);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccrC);

}

float position_pid(const float error, pid_state_t* state, const float num[4], const float den[4])
{
  const float in = error;
  const float out =
      num[0] * in +
      num[1] * state->in_prev[0] +
      num[2] * state->in_prev[1] +
      num[3] * state->in_prev[2] +
      den[1] * state->out_prev[0] +
      den[2] * state->out_prev[1] +
      den[3] * state->out_prev[2];

  // 更新历史
  state->in_prev[2] = state->in_prev[1];
  state->in_prev[1] = state->in_prev[0];
  state->in_prev[0] = in;

  state->out_prev[2] = state->out_prev[1];
  state->out_prev[1] = state->out_prev[0];
  state->out_prev[0] = out;

  return out;
}

float speed_pid(const float speed_error, pid_state_t* state, const float num[4], const float den[4])
{
  const float in = speed_error;
  float out =
      num[0] * in +
      num[1] * state->in_prev[0] +
      num[2] * state->in_prev[1] +
      num[3] * state->in_prev[2] +
      den[1] * state->out_prev[0] +
      den[2] * state->out_prev[1] +
      den[3] * state->out_prev[2];

  // 输出限幅（±8400  对应PWM）
  if (out > 8400.0f) out = 8400.0f;
  if (out < -8400.0f) out = -8400.0f;

  // 更新历史
  state->in_prev[2] = state->in_prev[1];
  state->in_prev[1] = state->in_prev[0];
  state->in_prev[0] = in;

  state->out_prev[2] = state->out_prev[1];
  state->out_prev[1] = state->out_prev[0];
  state->out_prev[0] = out;

  return out;
}

float iq_pid(const float iq_error, pid_state_t* state, const float num[4], const float den[4]){
  const float iq = iq_error;
  float out =
      num[0] * in +
      num[1] * state->in_prev[0] +
      num[2] * state->in_prev[1] +
      num[3] * state->in_prev[2] +
      den[1] * state->out_prev[0] +
      den[2] * state->out_prev[1] +
      den[3] * state->out_prev[2];
  if (sqrtf(Ud*Ud+Uq*Uq)>VOLTAGE_LIMIT) {
    Ud = Ud * VOLTAGE_LIMIT / sqrtf(Ud*Ud+Uq*Uq);
    Uq = Uq * VOLTAGE_LIMIT / sqrtf(Ud*Ud+Uq*Uq);
  }
}
