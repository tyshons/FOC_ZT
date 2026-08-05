#include "Experiment_Baseline.h"

#include <math.h>
#include "Experiment_Config.h"

float Experiment_Baseline_ClampIq(float iq_command_a) {
  /* 异常值不能进入PWM调制链路，直接退回零转矩指令。 */
  if (!isfinite(iq_command_a)) {
    return 0.0f;
  }
  /* 对称限幅，保持正反转时相同的安全电流边界。 */
  if (iq_command_a > EXPERIMENT_IQ_REFERENCE_MAX_A) {
    return EXPERIMENT_IQ_REFERENCE_MAX_A;
  }
  if (iq_command_a < -EXPERIMENT_IQ_REFERENCE_MAX_A) {
    return -EXPERIMENT_IQ_REFERENCE_MAX_A;
  }
  return iq_command_a;
}
