#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "Experiment_Config.h"
#include "Experiment_Control.h"

int main(void) {
  Experiment_Control_Init();

  /* 即使允许运行时切换模式，上电后也必须保持基线模式。 */
  const float baseline =
      Experiment_Control_Update(120.0f, 5.0f, 0.20f);
  assert(g_experiment_active_mode == PHYSICAL_EXPERIMENT_BASELINE);
  assert(fabsf(baseline - 0.20f) <= 1.0e-6f);

  g_experiment_mode_request = PHYSICAL_EXPERIMENT_ESO;
  const float eso_output =
      Experiment_Control_Update(120.0f, 5.0f, 0.20f);
  assert(EXPERIMENT_RUNTIME_MODES_ENABLED == 1U);
  assert(g_experiment_active_mode == PHYSICAL_EXPERIMENT_ESO);
  assert(isfinite(eso_output));
  assert(fabsf(eso_output) <=
         EXPERIMENT_IQ_REFERENCE_MAX_A + 1.0e-6f);

  puts("merged_runtime_mode_host_test: PASS");
  return 0;
}
