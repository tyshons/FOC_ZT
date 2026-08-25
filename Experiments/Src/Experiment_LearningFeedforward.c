#include "Experiment_LearningFeedforward.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define LFF_PI        3.14159265358979323846f
#define LFF_TWO_PI    (2.0f * LFF_PI)
#define LFF_EPSILON   1.0e-12f

/* 双缓冲学习表：快速路径只读活动表，后台只写另一个表。 */
float g_lff_table_bank_nm[2][PAPER_LFF_POSITION_BINS];
volatile uint8_t g_lff_active_table_index = 0U;
volatile uint16_t g_lff_current_bin = 0U;
volatile float g_lff_rho_mean = 0.0f;
volatile float g_lff_global_rho = 0.0f;
float g_lff_rho_orders[PAPER_LFF_MAX_ORDER + 1U];
float g_lff_last_residual_a_nm[PAPER_LFF_MAX_ORDER + 1U];
float g_lff_last_residual_b_nm[PAPER_LFF_MAX_ORDER + 1U];
float g_lff_learned_a_nm[PAPER_LFF_MAX_ORDER + 1U];
float g_lff_learned_b_nm[PAPER_LFF_MAX_ORDER + 1U];
volatile uint32_t g_lff_revolution_count = 0U;
volatile uint32_t g_lff_covered_revolution_count = 0U;
volatile uint32_t g_lff_update_count = 0U;
volatile uint32_t g_lff_dropped_revolution_count = 0U;
volatile float g_lff_last_coverage = 0.0f;
volatile uint8_t g_lff_selected_order_count = 0U;
volatile float g_lff_table_rms_nm = 0.0f;
volatile float g_lff_table_peak_abs_nm = 0.0f;
volatile float g_lff_table_mean_nm = 0.0f;
volatile float g_lff_dc_torque_nm = 0.0f;

/* 双缓冲的一转采集桶；每桶保存位置处的残余扰动累加值和样本数。 */
static float profile_sum_nm[2][PAPER_LFF_POSITION_BINS];
static uint16_t profile_sample_count[2][PAPER_LFF_POSITION_BINS];
static float current_profile_nm[PAPER_LFF_POSITION_BINS];
static float learning_profile_nm[PAPER_LFF_POSITION_BINS];

/* 相邻两转的傅里叶系数及其指数滑动相关性统计量。 */
static float previous_a[PAPER_LFF_MAX_ORDER + 1U];
static float previous_b[PAPER_LFF_MAX_ORDER + 1U];
static float cross_average[PAPER_LFF_MAX_ORDER + 1U];
static float previous_energy_average[PAPER_LFF_MAX_ORDER + 1U];
static float current_energy_average[PAPER_LFF_MAX_ORDER + 1U];
static float magnitude_average[PAPER_LFF_MAX_ORDER + 1U];
static float magnitude_sq_average[PAPER_LFF_MAX_ORDER + 1U];

/* pending_profile_bank由快速路径提交、后台取走，避免两侧处理同一采样表。 */
static volatile uint8_t active_profile_bank = 0U;
static volatile int8_t pending_profile_bank = -1;
static volatile uint8_t collection_paused = 0U;
static volatile uint8_t abort_collection_requested = 0U;
static Position_Lff_Mode pending_mode = POSITION_LFF_MODE_SELECTIVE;
static uint8_t previous_theta_valid = 0U;
static float previous_theta_rad = 0.0f;
static uint8_t have_previous_coefficients = 0U;
static uint32_t coefficient_pair_count = 0U;

static float ClampFloat(float value, float minimum, float maximum) {
  if (value > maximum) {
    return maximum;
  }
  if (value < minimum) {
    return minimum;
  }
  return value;
}

static float NormalizeAngle(float theta_rad) {
  /* 统一到[0, 2π)，让位置表索引和整转回绕检测不依赖编码器零点。 */
  if (!isfinite(theta_rad)) {
    return 0.0f;
  }

  theta_rad = fmodf(theta_rad, LFF_TWO_PI);
  if (theta_rad < 0.0f) {
    theta_rad += LFF_TWO_PI;
  }
  return theta_rad;
}

static uint16_t AngleToBin(float theta_rad) {
  /* 每个桶覆盖相同的机械角度区间，2π边界归入最后一个有效桶。 */
  const float normalized = NormalizeAngle(theta_rad);
  uint32_t bin = (uint32_t)(normalized *
                            ((float)PAPER_LFF_POSITION_BINS / LFF_TWO_PI));
  if (bin >= PAPER_LFF_POSITION_BINS) {
    bin = PAPER_LFF_POSITION_BINS - 1U;
  }
  return (uint16_t)bin;
}

static void ClearProfileBank(uint8_t bank) {
  memset(profile_sum_nm[bank], 0, sizeof(profile_sum_nm[bank]));
  memset(profile_sample_count[bank], 0, sizeof(profile_sample_count[bank]));
}

static void FinishPendingProfile(uint8_t processed_bank) {
  /* 后台处理完的桶才能重用；采集曾暂停时，同时清理重新启用的活动桶。 */
  ClearProfileBank(processed_bank);

  if (collection_paused != 0U) {
    ClearProfileBank(active_profile_bank);
    collection_paused = 0U;
    previous_theta_valid = 0U;
  }

  pending_profile_bank = -1;
}

void Position_Learning_Init(void) {
  Position_Learning_ResetAll();
}

void Position_Learning_ResetAll(void) {
  /* 此函数只应由初始化或后台复位路径调用，避免快速环清大数组。 */
  memset(g_lff_table_bank_nm, 0, sizeof(g_lff_table_bank_nm));
  memset(g_lff_rho_orders, 0, sizeof(g_lff_rho_orders));
  memset(g_lff_last_residual_a_nm, 0,
         sizeof(g_lff_last_residual_a_nm));
  memset(g_lff_last_residual_b_nm, 0,
         sizeof(g_lff_last_residual_b_nm));
  memset(g_lff_learned_a_nm, 0, sizeof(g_lff_learned_a_nm));
  memset(g_lff_learned_b_nm, 0, sizeof(g_lff_learned_b_nm));
  memset(previous_a, 0, sizeof(previous_a));
  memset(previous_b, 0, sizeof(previous_b));
  memset(cross_average, 0, sizeof(cross_average));
  memset(previous_energy_average, 0, sizeof(previous_energy_average));
  memset(current_energy_average, 0, sizeof(current_energy_average));
  memset(magnitude_average, 0, sizeof(magnitude_average));
  memset(magnitude_sq_average, 0, sizeof(magnitude_sq_average));
  ClearProfileBank(0U);
  ClearProfileBank(1U);

  active_profile_bank = 0U;
  pending_profile_bank = -1;
  collection_paused = 0U;
  abort_collection_requested = 0U;
  previous_theta_valid = 0U;
  previous_theta_rad = 0.0f;
  have_previous_coefficients = 0U;
  coefficient_pair_count = 0U;
  g_lff_active_table_index = 0U;
  g_lff_current_bin = 0U;
  g_lff_rho_mean = 0.0f;
  g_lff_global_rho = 0.0f;
  g_lff_revolution_count = 0U;
  g_lff_covered_revolution_count = 0U;
  g_lff_update_count = 0U;
  g_lff_dropped_revolution_count = 0U;
  g_lff_last_coverage = 0.0f;
  g_lff_selected_order_count = 0U;
  g_lff_table_rms_nm = 0.0f;
  g_lff_table_peak_abs_nm = 0.0f;
  g_lff_table_mean_nm = 0.0f;
  g_lff_dc_torque_nm = 0.0f;
}

void Position_Learning_AbortCollection(void) {
  /* 设置标志由后台清空两个采集桶，保证快速路径不与memset并发。 */
  abort_collection_requested = 1U;
  previous_theta_valid = 0U;
}

float Position_Learning_GetOutput(float theta_rad) {
  /* 论文只允许非零阶位置同步分量进入学习前馈，位置表始终保持零均值。 */
  const uint16_t bin = AngleToBin(theta_rad);
  const uint8_t table_bank = g_lff_active_table_index;
  g_lff_current_bin = bin;
  return g_lff_table_bank_nm[table_bank][bin];
}

void Position_Learning_Sample(float theta_rad,
                              float residual_torque_nm,
                              uint8_t learning_enabled,
                              Position_Lff_Mode mode) {
  if ((learning_enabled == 0U) || !isfinite(theta_rad) ||
      !isfinite(residual_torque_nm) ||
      (abort_collection_requested != 0U)) {
    previous_theta_valid = 0U;
    return;
  }

  /* 由角度回绕识别整转完成，并把刚完成的采样表交给后台。 */
  theta_rad = NormalizeAngle(theta_rad);
  if (previous_theta_valid == 0U) {
    previous_theta_rad = theta_rad;
    previous_theta_valid = 1U;
  } else if (theta_rad < (previous_theta_rad - LFF_PI)) {
    g_lff_revolution_count++;
    if (pending_profile_bank < 0) {
      /* 交换采集桶后立即继续下一转采样，实现后台与快速路径并行。 */
      const uint8_t completed_bank = active_profile_bank;
      active_profile_bank = (uint8_t)(1U - completed_bank);
      pending_mode = mode;
      pending_profile_bank = (int8_t)completed_bank;
    } else {
      /* 后台尚未消费上一转时宁可丢弃本转，也不覆盖待处理数据。 */
      g_lff_dropped_revolution_count++;
      collection_paused = 1U;
    }
  }
  previous_theta_rad = theta_rad;

  if (collection_paused != 0U) {
    return;
  }

  /* 采样值加上当前完整学习输出，重建下一次学习所需的扰动轮廓。 */
  const uint16_t bin = AngleToBin(theta_rad);
  uint16_t *const count = &profile_sample_count[active_profile_bank][bin];
  if (*count < UINT16_MAX) {
    const uint8_t table_bank = g_lff_active_table_index;
    const float table_value = g_lff_table_bank_nm[table_bank][bin];
    profile_sum_nm[active_profile_bank][bin] +=
        residual_torque_nm + table_value;
    (*count)++;
  }
}

void Position_Learning_Background(void) {
  if (abort_collection_requested != 0U) {
    /* 将异步中止请求落实为清空采集缓冲，保留已经生效的学习表。 */
    ClearProfileBank(0U);
    ClearProfileBank(1U);
    active_profile_bank = 0U;
    pending_profile_bank = -1;
    collection_paused = 0U;
    previous_theta_valid = 0U;
    abort_collection_requested = 0U;
    return;
  }

  const int8_t pending = pending_profile_bank;
  if (pending < 0) {
    return;
  }
  const uint8_t profile_bank = (uint8_t)pending;

  /* 先将不等间隔采样平均到位置桶，并检查本转的位置覆盖率。 */
  uint32_t covered_bins = 0U;
  for (uint32_t bin = 0U; bin < PAPER_LFF_POSITION_BINS; ++bin) {
    const uint16_t count = profile_sample_count[profile_bank][bin];
    if (count > 0U) {
      current_profile_nm[bin] =
          profile_sum_nm[profile_bank][bin] / (float)count;
      covered_bins++;
    } else {
      current_profile_nm[bin] = 0.0f;
    }
  }

  const float coverage =
      (float)covered_bins / (float)PAPER_LFF_POSITION_BINS;
  g_lff_last_coverage = coverage;
  if (coverage < PAPER_LFF_MIN_COVERAGE) {
    /* 覆盖不足的转不参与频谱和相关性统计，防止缺失区间导致误学习。 */
    FinishPendingProfile(profile_bank);
    return;
  }
  g_lff_covered_revolution_count++;

  /* 对完整轮廓做实数傅里叶投影，a/b分别为余弦/正弦系数。 */
  float current_a[PAPER_LFF_MAX_ORDER + 1U] = {0.0f};
  float current_b[PAPER_LFF_MAX_ORDER + 1U] = {0.0f};

  /* 0阶仅保留为原始轮廓均值诊断量，不参与可信度计算和学习表更新。 */
  float mean = 0.0f;
  for (uint32_t bin = 0U; bin < PAPER_LFF_POSITION_BINS; ++bin) {
    mean += current_profile_nm[bin];
  }
  current_a[0] = mean / (float)PAPER_LFF_POSITION_BINS;

  const float fourier_scale = 2.0f / (float)PAPER_LFF_POSITION_BINS;
  for (uint32_t order = 1U; order <= PAPER_LFF_MAX_ORDER; ++order) {
    const float step =
        LFF_TWO_PI * (float)order / (float)PAPER_LFF_POSITION_BINS;
    const float cosine_step = cosf(step);
    const float sine_step = sinf(step);
    float cosine = 1.0f;
    float sine = 0.0f;
    float a_sum = 0.0f;
    float b_sum = 0.0f;

    for (uint32_t bin = 0U; bin < PAPER_LFF_POSITION_BINS; ++bin) {
      a_sum += current_profile_nm[bin] * cosine;
      b_sum += current_profile_nm[bin] * sine;
      const float next_cosine =
          cosine * cosine_step - sine * sine_step;
      sine = sine * cosine_step + cosine * sine_step;
      cosine = next_cosine;
    }
    current_a[order] = fourier_scale * a_sum;
    current_b[order] = fourier_scale * b_sum;
  }

  /* 比较相邻转的谐波稳定性，筛除随机扰动和未收敛的阶次。 */
  uint8_t correlation_ready = 0U;
  if (have_previous_coefficients != 0U) {
    const float forgetting = PAPER_LFF_RHO_FORGETTING;
    const float new_weight = 1.0f - forgetting;

    for (uint32_t order = PAPER_LFF_MIN_ORDER;
         order <= PAPER_LFF_MAX_ORDER;
         ++order) {
      const float cross = current_a[order] * previous_a[order] +
                          current_b[order] * previous_b[order];
      const float previous_energy =
          previous_a[order] * previous_a[order] +
          previous_b[order] * previous_b[order];
      const float current_energy =
          current_a[order] * current_a[order] +
          current_b[order] * current_b[order];
      const float magnitude = sqrtf(current_energy);

      if (coefficient_pair_count == 0U) {
        cross_average[order] = cross;
        previous_energy_average[order] = previous_energy;
        current_energy_average[order] = current_energy;
        magnitude_average[order] = magnitude;
        magnitude_sq_average[order] = magnitude * magnitude;
      } else {
        cross_average[order] =
            forgetting * cross_average[order] + new_weight * cross;
        previous_energy_average[order] =
            forgetting * previous_energy_average[order] +
            new_weight * previous_energy;
        current_energy_average[order] =
            forgetting * current_energy_average[order] +
            new_weight * current_energy;
        magnitude_average[order] =
            forgetting * magnitude_average[order] + new_weight * magnitude;
        magnitude_sq_average[order] =
            forgetting * magnitude_sq_average[order] +
            new_weight * magnitude * magnitude;
      }
    }

    coefficient_pair_count++;
    correlation_ready =
        (coefficient_pair_count >= PAPER_LFF_RHO_MIN_PAIRS) ? 1U : 0U;
  } else {
    have_previous_coefficients = 1U;
  }

  memcpy(g_lff_last_residual_a_nm, current_a,
         sizeof(g_lff_last_residual_a_nm));
  memcpy(g_lff_last_residual_b_nm, current_b,
         sizeof(g_lff_last_residual_b_nm));

  /*
   * 与仿真保持一致：负相关阶次不取绝对值；幅值变异系数连续衰减学习率，
   * 而不是只做二值开关。rho_mean按轮廓能量统计可信分量占比。
   */
  float total_energy = 0.0f;
  float trusted_energy = 0.0f;
  uint8_t selected_order_count = 0U;
  g_lff_rho_orders[0] = 0.0f;
  for (uint32_t order = PAPER_LFF_MIN_ORDER;
       order <= PAPER_LFF_MAX_ORDER;
       ++order) {
    float rho = 0.0f;
    if (correlation_ready != 0U) {
      const float denominator =
          sqrtf(previous_energy_average[order] *
                    current_energy_average[order] +
                LFF_EPSILON);
      const float raw_rho =
          ClampFloat(cross_average[order] / denominator, 0.0f, 1.0f);
      const float average_magnitude = magnitude_average[order];
      float magnitude_cv = 0.0f;
      if (average_magnitude > LFF_EPSILON) {
        const float variance =
            fmaxf(0.0f, magnitude_sq_average[order] -
                            average_magnitude * average_magnitude);
        magnitude_cv = sqrtf(variance) / average_magnitude;
      }
      const float amplitude_score = ClampFloat(
          1.0f - magnitude_cv / PAPER_LFF_MAGNITUDE_CV_LIMIT,
          0.0f,
          1.0f);

      if ((raw_rho >= PAPER_LFF_RHO_THRESHOLD) &&
          (average_magnitude >= PAPER_LFF_AMPLITUDE_MIN_NM) &&
          (magnitude_cv <= PAPER_LFF_MAGNITUDE_CV_LIMIT)) {
        rho = raw_rho * amplitude_score;
      }
    }
    g_lff_rho_orders[order] = rho;
    const float order_energy = 0.5f * current_energy_average[order];
    total_energy += order_energy;
    if (rho > 0.0f) {
      trusted_energy += order_energy;
      selected_order_count++;
    }
  }
  g_lff_rho_mean =
      (total_energy > LFF_EPSILON) ? trusted_energy / total_energy : 0.0f;

  /* 全局学习率只汇总论文定义的非零阶，不再让直流负载主导相关性。 */
  float global_cross = 0.0f;
  float global_previous_energy = 0.0f;
  float global_current_energy = 0.0f;
  for (uint32_t order = PAPER_LFF_MIN_ORDER;
       order <= PAPER_LFF_MAX_ORDER;
       ++order) {
    global_cross += 0.5f * cross_average[order];
    global_previous_energy += 0.5f * previous_energy_average[order];
    global_current_energy += 0.5f * current_energy_average[order];
  }
  g_lff_global_rho = 0.0f;
  if (correlation_ready != 0U) {
    const float global_denominator =
        sqrtf(global_previous_energy * global_current_energy + LFF_EPSILON);
    g_lff_global_rho =
        ClampFloat(global_cross / global_denominator, 0.0f, 1.0f);
  }

  uint8_t can_update = 0U;
  if (pending_mode == POSITION_LFF_MODE_ORDINARY) {
    /* 普通模式不做筛选；选择性/全局模式需要足够的相邻转统计量。 */
    can_update = 1U;
    g_lff_rho_mean = 1.0f;
    g_lff_global_rho = 1.0f;
    g_lff_rho_orders[0] = 0.0f;
    for (uint32_t order = PAPER_LFF_MIN_ORDER;
         order <= PAPER_LFF_MAX_ORDER;
         ++order) {
      g_lff_rho_orders[order] = 1.0f;
    }
    selected_order_count =
        PAPER_LFF_MAX_ORDER - PAPER_LFF_MIN_ORDER + 1U;
  } else if (correlation_ready != 0U) {
    can_update = 1U;
  }
  g_lff_selected_order_count = selected_order_count;

  if (can_update != 0U) {
    /* 只由论文定义的1~40阶重构学习表，并强制保持零均值。 */
    const uint8_t read_table = g_lff_active_table_index;
    const uint8_t write_table = (uint8_t)(1U - read_table);

    for (uint32_t bin = 0U; bin < PAPER_LFF_POSITION_BINS; ++bin) {
      learning_profile_nm[bin] = 0.0f;
    }
    for (uint32_t order = PAPER_LFF_MIN_ORDER;
         order <= PAPER_LFF_MAX_ORDER;
         ++order) {
      const float weight =
          (pending_mode == POSITION_LFF_MODE_GLOBAL)
              ? g_lff_global_rho
              : g_lff_rho_orders[order];
      const float step =
          LFF_TWO_PI * (float)order / (float)PAPER_LFF_POSITION_BINS;
      const float cosine_step = cosf(step);
      const float sine_step = sinf(step);
      float cosine = 1.0f;
      float sine = 0.0f;

      for (uint32_t bin = 0U; bin < PAPER_LFF_POSITION_BINS; ++bin) {
        learning_profile_nm[bin] +=
            weight * (current_a[order] * cosine +
                      current_b[order] * sine);
        const float next_cosine =
            cosine * cosine_step - sine * sine_step;
        sine = sine * cosine_step + cosine * sine_step;
        cosine = next_cosine;
      }
    }

    float table_mean_nm = 0.0f;
    for (uint32_t bin = 0U; bin < PAPER_LFF_POSITION_BINS; ++bin) {
      const float previous_value = g_lff_table_bank_nm[read_table][bin];
      const float updated =
          PAPER_LFF_LEAKAGE * previous_value +
          PAPER_LFF_GAMMA * (learning_profile_nm[bin] - previous_value);
      g_lff_table_bank_nm[write_table][bin] = updated;
      table_mean_nm += updated;
    }
    table_mean_nm /= (float)PAPER_LFF_POSITION_BINS;

    float unscaled_peak_abs_nm = 0.0f;
    for (uint32_t bin = 0U; bin < PAPER_LFF_POSITION_BINS; ++bin) {
      const float zero_mean =
          g_lff_table_bank_nm[write_table][bin] - table_mean_nm;
      g_lff_table_bank_nm[write_table][bin] = zero_mean;
      unscaled_peak_abs_nm =
          fmaxf(unscaled_peak_abs_nm, fabsf(zero_mean));
    }
    const float table_scale =
        (unscaled_peak_abs_nm > PAPER_LFF_TORQUE_LIMIT_NM)
            ? PAPER_LFF_TORQUE_LIMIT_NM / unscaled_peak_abs_nm
            : 1.0f;

    float published_sum_nm = 0.0f;
    float table_sum_sq_nm2 = 0.0f;
    float table_peak_abs_nm = 0.0f;
    for (uint32_t bin = 0U; bin < PAPER_LFF_POSITION_BINS; ++bin) {
      const float published_value =
          g_lff_table_bank_nm[write_table][bin] * table_scale;
      g_lff_table_bank_nm[write_table][bin] = published_value;
      published_sum_nm += published_value;
      table_sum_sq_nm2 += published_value * published_value;
      table_peak_abs_nm =
          fmaxf(table_peak_abs_nm, fabsf(published_value));
    }

    g_lff_learned_a_nm[0] = 0.0f;
    g_lff_learned_b_nm[0] = 0.0f;
    for (uint32_t order = PAPER_LFF_MIN_ORDER;
         order <= PAPER_LFF_MAX_ORDER;
         ++order) {
      const float weight =
          (pending_mode == POSITION_LFF_MODE_GLOBAL)
              ? g_lff_global_rho
              : g_lff_rho_orders[order];
      const float updated_a =
          PAPER_LFF_LEAKAGE * g_lff_learned_a_nm[order] +
          PAPER_LFF_GAMMA *
              (weight * current_a[order] - g_lff_learned_a_nm[order]);
      const float updated_b =
          PAPER_LFF_LEAKAGE * g_lff_learned_b_nm[order] +
          PAPER_LFF_GAMMA *
              (weight * current_b[order] - g_lff_learned_b_nm[order]);
      g_lff_learned_a_nm[order] = updated_a * table_scale;
      g_lff_learned_b_nm[order] = updated_b * table_scale;
    }

    /* 保留协议字段但固定为零，避免改变现有上下位机通信布局。 */
    g_lff_dc_torque_nm = 0.0f;
    g_lff_table_mean_nm =
        published_sum_nm / (float)PAPER_LFF_POSITION_BINS;
    g_lff_table_rms_nm =
        sqrtf(table_sum_sq_nm2 / (float)PAPER_LFF_POSITION_BINS);
    g_lff_table_peak_abs_nm = table_peak_abs_nm;
    g_lff_active_table_index = write_table;
    g_lff_update_count++;
  }

  memcpy(previous_a, current_a, sizeof(previous_a));
  memcpy(previous_b, current_b, sizeof(previous_b));
  FinishPendingProfile(profile_bank);
}
