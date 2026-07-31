//
// Created by tyshon on 2026/5/25.
//

#include "pos_process.h"

#include <math.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"

#define PI 3.14159265358979323846f
#define SPEED_FILTER_ALPHA 0.1f

static float last_angle = 0.0f;
static uint8_t is_initialized = 0U;
static float filtered_speed = 0.0f;

void Encoder_Speed_Reset(float current_angle_deg)
{
  last_angle = current_angle_deg;
  filtered_speed = 0.0f;
  is_initialized = 1U;
}

void Encoder_Speed_Update(float *speed_out,const float *current_angle_sp) {
  const float dt = 0.00025f;

  if (speed_out == NULL || current_angle_sp == NULL) {
    return;
  }

  float current_angle = * current_angle_sp ;

  if (!is_initialized) {
    last_angle = current_angle;
    filtered_speed = 0.0f;
    *speed_out = 0.0f;
    is_initialized = 1;
    return;
  }
  //uint32_t current_time = HAL_GetTick();

  float delta_angle = current_angle - last_angle;

  if (delta_angle > 180.0f)  delta_angle -= 360.0f;
  if (delta_angle < -180.0f) delta_angle += 360.0f;

  float instant_speed = delta_angle / (dt * 6.0f); //rpm

  filtered_speed = SPEED_FILTER_ALPHA * instant_speed +
                     (1.0f - SPEED_FILTER_ALPHA) * filtered_speed;
  *speed_out = filtered_speed;

  last_angle = current_angle;

}

void Get_Electrical_Angle(float *theta_out,const float *current_angle_sp) {
  static float theta_filt = 0.0f;
  float angle_offset = 96.0f;

  float theta_mech = ((*current_angle_sp - angle_offset) / 180.0f) * PI;

  float theta_elec = theta_mech * 20.0f;

  theta_elec = fmodf(theta_elec, 2.0f * PI);
  if (theta_elec < 0) {
    theta_elec += 2.0f * PI;
  }

  //theta_filt = 0.8f * theta_elec + 0.2f * theta_filt;

  *theta_out = theta_elec;
}
