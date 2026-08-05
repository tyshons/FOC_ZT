#ifndef FOC_ZT_EXPERIMENT_FIXED_FF_SWEEP_H
#define FOC_ZT_EXPERIMENT_FIXED_FF_SWEEP_H

#include <stdint.h>
#include "Experiment_Config.h"

/* 单个阶次的正、负注入依次经历稳定和测量两个阶段。 */
typedef enum {
  FIXED_FF_SWEEP_IDLE = 0U,
  FIXED_FF_SWEEP_SETTLE_POSITIVE = 1U,
  FIXED_FF_SWEEP_MEASURE_POSITIVE = 2U,
  FIXED_FF_SWEEP_SETTLE_NEGATIVE = 3U,
  FIXED_FF_SWEEP_MEASURE_NEGATIVE = 4U,
  FIXED_FF_SWEEP_COMPLETE = 5U
} Experiment_FixedFF_Sweep_State;

/*
 * 调试器控制接口。
 *
 * 先设置阶次范围和注入幅值，再选择实验模式6并让电机达到目标速度，
 * 最后把g_sweep_enable_request设为1。把g_sweep_reset_request设为1可清除全部结果。
 */
extern volatile uint8_t g_sweep_enable_request;
extern volatile uint8_t g_sweep_reset_request;
extern volatile uint8_t g_sweep_start_order_request;
extern volatile uint8_t g_sweep_end_order_request;
extern volatile float g_sweep_injection_amplitude_request_a;

/* 扫频状态和快速遥测量。 */
extern volatile uint8_t g_sweep_state;
extern volatile uint8_t g_sweep_active_order;
extern volatile uint8_t g_sweep_stage_revolution_count;
extern volatile uint8_t g_sweep_speed_ready;
extern volatile uint8_t g_sweep_current_ready;
extern volatile uint8_t g_sweep_complete;
extern volatile uint8_t g_sweep_completed_order_count;
extern volatile float g_iq_sweep_a;
/* 最近一圈的质量指标，用于判断扫频为何等待。 */
extern volatile float g_sweep_last_revolution_mean_rpm;
extern volatile float g_sweep_last_revolution_coverage;
extern volatile uint16_t g_sweep_rejected_revolution_count;

/*
 * 结果数组使用机械阶次作为索引，范围为1到PAPER_LFF_MAX_ORDER。
 * 系数关系为x(theta)=cos_coefficient*cos(n*theta)
 *                  +sin_coefficient*sin(n*theta)。
 */
extern volatile float
    g_sweep_baseline_cos_rpm[PAPER_LFF_MAX_ORDER + 1U];
extern volatile float
    g_sweep_baseline_sin_rpm[PAPER_LFF_MAX_ORDER + 1U];
extern volatile float
    g_sweep_response_cos_rpm_per_a[PAPER_LFF_MAX_ORDER + 1U];
extern volatile float
    g_sweep_response_sin_rpm_per_a[PAPER_LFF_MAX_ORDER + 1U];
extern volatile float
    g_sweep_response_magnitude_rpm_per_a[PAPER_LFF_MAX_ORDER + 1U];
extern volatile float
    g_sweep_recommended_cos_current_a[PAPER_LFF_MAX_ORDER + 1U];
extern volatile float
    g_sweep_recommended_sin_current_a[PAPER_LFF_MAX_ORDER + 1U];
/* 逐圈统计量；标准差描述单圈离散度，标准误差描述最终均值的不确定度。 */
extern volatile float
    g_sweep_positive_coefficient_std_rpm[PAPER_LFF_MAX_ORDER + 1U];
extern volatile float
    g_sweep_negative_coefficient_std_rpm[PAPER_LFF_MAX_ORDER + 1U];
extern volatile float
    g_sweep_baseline_standard_error_rpm[PAPER_LFF_MAX_ORDER + 1U];
extern volatile float
    g_sweep_response_standard_error_rpm_per_a[PAPER_LFF_MAX_ORDER + 1U];
extern volatile float
    g_sweep_response_snr[PAPER_LFF_MAX_ORDER + 1U];
extern volatile float
    g_sweep_recommended_standard_error_a[PAPER_LFF_MAX_ORDER + 1U];
extern volatile uint8_t
    g_sweep_statistics_revolution_count[PAPER_LFF_MAX_ORDER + 1U];
extern volatile uint8_t
    g_sweep_result_valid[PAPER_LFF_MAX_ORDER + 1U];

/* 初始化或终止扫频状态机；Stop不清除已完成的实验结果。 */
void Experiment_FixedFFSweep_Init(void);
void Experiment_FixedFFSweep_Stop(void);
/* 在后台响应结果清除请求，避免在快速控制回调中写大数组。 */
void Experiment_FixedFFSweep_Background(void);
/*
 * 每个速度环周期调用一次。满足转速和电流裕量条件时，返回当前
 * 机械阶次的正弦注入电流；否则暂停扫频并输出零注入。
 */
float Experiment_FixedFFSweep_Update(float mechanical_angle_rad,
                                     float speed_rpm,
                                     float feedback_iq_a);

#endif /* FOC_ZT_EXPERIMENT_FIXED_FF_SWEEP_H */
