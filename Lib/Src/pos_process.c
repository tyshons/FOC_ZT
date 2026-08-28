#include "pos_process.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "Experiment_Config.h"

#define PI 3.14159265358979323846f

typedef struct {
  uint8_t is_initialized;
  float filtered_speed;
  float angle_history[FOC_SPEED_ESTIMATOR_WINDOW_UPDATES];
  float period_history_s[FOC_SPEED_ESTIMATOR_WINDOW_UPDATES];
  uint32_t angle_history_index;
  uint32_t angle_history_count;
  float window_period_s;
} EncoderSpeedState;

static EncoderSpeedState encoder_speed_state_sp;
static EncoderSpeedState encoder_speed_state_fy;

static void encoder_speed_reset(EncoderSpeedState *state,
                                float current_angle_deg)
{
  for (uint32_t index = 0U;
       index < FOC_SPEED_ESTIMATOR_WINDOW_UPDATES;
       index++) {
    state->angle_history[index] = current_angle_deg;
    state->period_history_s[index] = 0.0f;
  }
  state->angle_history_index = 0U;
  state->angle_history_count = 0U;
  state->window_period_s = 0.0f;
  state->filtered_speed = 0.0f;
  state->is_initialized = 1U;
}

static void encoder_speed_update(EncoderSpeedState *state,
                                 float *speed_out,
                                 const float *current_angle,
                                 float sample_period_s)
{
  if ((state == NULL) || (speed_out == NULL) || (current_angle == NULL) ||
      (sample_period_s <= 0.0f)) {
    return;
  }

  if (state->is_initialized == 0U) {
    encoder_speed_reset(state, *current_angle);
    *speed_out = 0.0f;
    return;
  }

  const uint32_t history_index = state->angle_history_index;
  const float previous_angle = state->angle_history[history_index];
  const float previous_period_s = state->period_history_s[history_index];
  state->angle_history[history_index] = *current_angle;
  state->period_history_s[history_index] = sample_period_s;
  state->window_period_s += sample_period_s - previous_period_s;

  state->angle_history_index++;
  if (state->angle_history_index >= FOC_SPEED_ESTIMATOR_WINDOW_UPDATES) {
    state->angle_history_index = 0U;
  }

  if (state->angle_history_count < FOC_SPEED_ESTIMATOR_WINDOW_UPDATES) {
    state->angle_history_count++;
    if (state->angle_history_count < FOC_SPEED_ESTIMATOR_WINDOW_UPDATES) {
      *speed_out = 0.0f;
      return;
    }
  }

  float delta_angle = *current_angle - previous_angle;
  if (delta_angle > 180.0f) {
    delta_angle -= 360.0f;
  }
  if (delta_angle < -180.0f) {
    delta_angle += 360.0f;
  }
  if (state->window_period_s <= 0.0f) {
    return;
  }

  const float instant_speed =
      delta_angle / (state->window_period_s * 6.0f);
  const float filter_alpha =
      1.0f - expf(-2.0f * PI * FOC_SPEED_FILTER_CUTOFF_HZ * sample_period_s);
  state->filtered_speed +=
      filter_alpha * (instant_speed - state->filtered_speed);
  *speed_out = state->filtered_speed;
}

void Encoder_Speed_Reset_sp(float current_angle_deg)
{
  encoder_speed_reset(&encoder_speed_state_sp, current_angle_deg);
}

void Encoder_Speed_Reset_fy(float current_angle_deg)
{
  encoder_speed_reset(&encoder_speed_state_fy, current_angle_deg);
}

void Encoder_Speed_Update_sp(float *speed_out,
                             const float *current_angle_sp,
                             float sample_period_s)
{
  encoder_speed_update(&encoder_speed_state_sp, speed_out, current_angle_sp,
                       sample_period_s);
}

void Encoder_Speed_Update_fy(float *speed_out,
                             const float *current_angle_fy,
                             float sample_period_s)
{
  encoder_speed_update(&encoder_speed_state_fy, speed_out, current_angle_fy,
                       sample_period_s);
}

void Get_Electrical_Angle(float *theta_out,
                          const float *current_angle,
                          float electrical_offset_deg)
{
  if ((theta_out == NULL) || (current_angle == NULL) ||
      !isfinite(electrical_offset_deg)) {
    return;
  }

  const float theta_mech =
      ((*current_angle - electrical_offset_deg) / 180.0f) * PI;
  float theta_elec = fmodf(theta_mech * MOTOR_POLE_PAIRS, 2.0f * PI);
  if (theta_elec < 0.0f) {
    theta_elec += 2.0f * PI;
  }
  *theta_out = theta_elec;
}
