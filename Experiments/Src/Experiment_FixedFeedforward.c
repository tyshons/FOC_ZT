#include "Experiment_FixedFeedforward.h"

#include <math.h>
#include "Experiment_Config.h"

float Experiment_FixedFeedforward_CurrentA(float mechanical_angle_rad) {
  if (!isfinite(mechanical_angle_rad)) {
    return 0.0f;
  }

  /* 当前只应用已在实机上重复验证的一阶机械角同步分量。 */
  return EXPERIMENT_FIXED_FF_GAIN *
         (EXPERIMENT_FIXED_FF_ORDER1_COS_CURRENT_A *
              cosf(mechanical_angle_rad) +
          EXPERIMENT_FIXED_FF_ORDER1_SIN_CURRENT_A *
              sinf(mechanical_angle_rad) +
          EXPERIMENT_FIXED_FF_ORDER2_COS_CURRENT_A *
              cosf(2.0f * mechanical_angle_rad) +
          EXPERIMENT_FIXED_FF_ORDER2_SIN_CURRENT_A *
              sinf(2.0f * mechanical_angle_rad) +
          EXPERIMENT_FIXED_FF_ORDER4_COS_CURRENT_A *
              cosf(4.0f * mechanical_angle_rad) +
          EXPERIMENT_FIXED_FF_ORDER4_SIN_CURRENT_A *
              sinf(4.0f * mechanical_angle_rad));
}
