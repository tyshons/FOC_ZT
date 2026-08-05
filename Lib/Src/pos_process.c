//
// 创建于 2026/5/25。
//

#include "pos_process.h"

#include <math.h>
#include <stdint.h>

#include "Experiment_Config.h"

#define PI 3.14159265358979323846f

static uint8_t is_initialized = 0U;
static float filtered_speed = 0.0f;
static float angle_history[FOC_SPEED_ESTIMATOR_WINDOW_UPDATES] = {0.0f};
static float period_history_s[FOC_SPEED_ESTIMATOR_WINDOW_UPDATES] = {0.0f};
static uint32_t angle_history_index = 0U;
static uint32_t angle_history_count = 0U;
static float window_period_s = 0.0f;

void Encoder_Speed_Reset(float current_angle_deg)
{
  for (uint32_t index = 0U;
       index < FOC_SPEED_ESTIMATOR_WINDOW_UPDATES;
       index++) {
    angle_history[index] = current_angle_deg;
    period_history_s[index] = 0.0f;
  }
  angle_history_index = 0U;
  angle_history_count = 0U;
  window_period_s = 0.0f;
  filtered_speed = 0.0f;
  is_initialized = 1U;
}

void Encoder_Speed_Update(float *speed_out,const float *current_angle_sp,float sample_period_s) {
  if ((speed_out == NULL) ||
      (current_angle_sp == NULL) ||
      (sample_period_s <= 0.0f)) {
    return;
  }

  float current_angle = * current_angle_sp ;

  if (!is_initialized) {
    Encoder_Speed_Reset(current_angle);
    *speed_out = 0.0f;
    return;
  }

  const float previous_angle = angle_history[angle_history_index];
  const float previous_period_s = period_history_s[angle_history_index];
  angle_history[angle_history_index] = current_angle;
  period_history_s[angle_history_index] = sample_period_s;
  window_period_s += sample_period_s - previous_period_s;
  angle_history_index++;
  if (angle_history_index >= FOC_SPEED_ESTIMATOR_WINDOW_UPDATES) {
    angle_history_index = 0U;
  }

  if (angle_history_count < FOC_SPEED_ESTIMATOR_WINDOW_UPDATES) {
    angle_history_count++;
    if (angle_history_count < FOC_SPEED_ESTIMATOR_WINDOW_UPDATES) {
      *speed_out = 0.0f;
      return;
    }
  }

  float delta_angle = current_angle - previous_angle;

  if (delta_angle > 180.0f)  delta_angle -= 360.0f;
  if (delta_angle < -180.0f) delta_angle += 360.0f;

  if (window_period_s <= 0.0f) {
    return;
  }
  const float instant_speed = delta_angle / (window_period_s * 6.0f);

  const float filter_alpha =
      1.0f - expf(-2.0f * PI * FOC_SPEED_FILTER_CUTOFF_HZ * sample_period_s);
  filtered_speed +=
      filter_alpha * (instant_speed - filtered_speed);
  *speed_out = filtered_speed;

}

void Get_Electrical_Angle(float *theta_out,const float *current_angle_sp) {
  if ((theta_out == NULL) || (current_angle_sp == NULL)) {
    return;
  }

  float theta_mech =
      ((*current_angle_sp - MOTOR_ELECTRICAL_OFFSET_DEG) / 180.0f) * PI;

  float theta_elec = theta_mech * MOTOR_POLE_PAIRS;

  theta_elec = fmodf(theta_elec, 2.0f * PI);
  if (theta_elec < 0) {
    theta_elec += 2.0f * PI;
  }

  *theta_out = theta_elec;
}
