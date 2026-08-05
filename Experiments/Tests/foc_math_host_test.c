#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "Experiment_Config.h"
#include "FOC_Math.h"

int main(void) {
  float alpha = 0.0f;
  float beta = 0.0f;
  clarke_transform(1.0f, -0.5f, -0.5f, &alpha, &beta);
  assert(fabsf(alpha - 1.0f) <= 1.0e-6f);
  assert(fabsf(beta) <= 1.0e-6f);

  uint32_t a = 0U;
  uint32_t b = 0U;
  uint32_t c = 0U;
  svpwm_generate(NAN, 0.0f, 48.0f, &a, &b, &c);
  assert(a == FOC_PWM_NEUTRAL_COMPARE);
  assert(b == FOC_PWM_NEUTRAL_COMPARE);
  assert(c == FOC_PWM_NEUTRAL_COMPARE);

  svpwm_generate(5.0f, 3.0f, 48.0f, &a, &b, &c);
  assert(a <= FOC_PWM_COMPARE_MAX);
  assert(b <= FOC_PWM_COMPARE_MAX);
  assert(c <= FOC_PWM_COMPARE_MAX);

  puts("foc_math_host_test: PASS");
  return 0;
}
