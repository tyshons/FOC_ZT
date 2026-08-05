#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "Experiment_Config.h"
#include "pos_process.h"

static float WrapAngle(float angle_deg) {
  while (angle_deg >= 360.0f) {
    angle_deg -= 360.0f;
  }
  while (angle_deg < 0.0f) {
    angle_deg += 360.0f;
  }
  return angle_deg;
}

int main(void) {
  float speed_rpm = 0.0f;
  float angle_deg = 359.95f;
  const float target_speed_rpm = 5.0f;
  const float angle_step_deg =
      target_speed_rpm * 6.0f * FOC_SPEED_LOOP_PERIOD_S;

  Encoder_Speed_Reset(angle_deg);
  for (uint32_t update = 0U; update < 2000U; update++) {
    angle_deg = WrapAngle(angle_deg + angle_step_deg);
    Encoder_Speed_Update(&speed_rpm,
                         &angle_deg,
                         FOC_SPEED_LOOP_PERIOD_S);
  }
  assert(fabsf(speed_rpm - target_speed_rpm) < 0.01f);

  /* 控制回调有抖动时，测速窗口必须累计真实间隔，而不是乘固定周期。 */
  angle_deg = 10.0f;
  Encoder_Speed_Reset(angle_deg);
  for (uint32_t update = 0U; update < 3000U; update++) {
    const float variable_period_s =
        ((update % 3U) == 0U) ? 0.00015f : 0.00030f;
    angle_deg = WrapAngle(
        angle_deg + target_speed_rpm * 6.0f * variable_period_s);
    Encoder_Speed_Update(&speed_rpm, &angle_deg, variable_period_s);
  }
  assert(fabsf(speed_rpm - target_speed_rpm) < 0.01f);

  /* 静止时相邻编码器计数交替，不应再形成正负速度脉冲。 */
  const float stationary_angle_deg = 120.0f;
  const float one_count_deg = 360.0f / SSI_ENCODER_COUNTS_PER_REV;
  Encoder_Speed_Reset(stationary_angle_deg);
  for (uint32_t update = 0U; update < 1000U; update++) {
    angle_deg = stationary_angle_deg +
                ((update & 1U) != 0U ? one_count_deg : 0.0f);
    Encoder_Speed_Update(&speed_rpm,
                         &angle_deg,
                         FOC_SPEED_LOOP_PERIOD_S);
  }
  assert(fabsf(speed_rpm) < 1.0e-4f);

  puts("pos_process_host_test: PASS");
  return 0;
}
