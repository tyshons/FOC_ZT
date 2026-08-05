#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "PID_Control.h"

static int IsClose(float actual, float expected, float tolerance) {
  return fabsf(actual - expected) <= tolerance;
}

int main(void) {
  PID_TypeDef pid = {
      .Kp = 1.0f,
      .Ki = 1.0f,
      .Kd = 0.0f,
      .output_min = -10.0f,
      .output_max = 10.0f,
      .integral_limit = 10.0f,
      .low_pass_filter_time_constant = 0.01f};
  PID_Init(&pid);

  /* PID复位后不得累计复位前的不确定时间区间。 */
  const float first = PID_Update(&pid, 1.0f, 900000U);
  assert(IsClose(first, 1.0f, 1.0e-6f));
  const float second = PID_Update(&pid, 1.0f, 901000U);
  assert(IsClose(second, 1.001f, 1.0e-6f));

  PID_Reset(&pid);
  PID_SetOutputLimits(&pid, -1.0f, 1.0f);
  pid.Kp = 2.0f;
  (void)PID_Update(&pid, 1.0f, 1000U);
  const float saturated = PID_Update(&pid, 1.0f, 2000U);
  assert(IsClose(saturated, 1.0f, 1.0e-6f));
  assert(IsClose(pid.integral, 0.0f, 1.0e-6f));

  puts("pid_control_host_test: PASS");
  return 0;
}
