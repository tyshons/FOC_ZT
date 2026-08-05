#ifndef FOC_ZT_EXPERIMENT_CONTROL_H
#define FOC_ZT_EXPERIMENT_CONTROL_H

#include <stdint.h>
#include "Experiment_FixedFF_Sweep.h"
#include "Experiment_LearningFeedforward.h"

/*
 * 可从调试器选择的实验组合。每种模式只叠加所需的补偿环节，
 * 便于将ESO、固定前馈和学习前馈的效果与基线逐项对比。
 */
typedef enum {
  PHYSICAL_EXPERIMENT_BASELINE = 0U,
  PHYSICAL_EXPERIMENT_ESO = 1U,
  PHYSICAL_EXPERIMENT_FIXED_FEEDFORWARD = 2U,
  PHYSICAL_EXPERIMENT_ESO_FIXED_FEEDFORWARD = 3U,
  PHYSICAL_EXPERIMENT_ESO_LEARNING_FEEDFORWARD = 4U,
  PHYSICAL_EXPERIMENT_FULL = 5U,
  PHYSICAL_EXPERIMENT_FIXED_FF_SWEEP = 6U
} Physical_Experiment_Mode;

/* 运行时实验模式选择；安全上电模式固定为基线模式。 */
extern volatile uint8_t g_experiment_mode_request;
extern volatile uint8_t g_experiment_active_mode;
extern volatile uint8_t g_learning_update_request;
extern volatile uint8_t g_learning_reset_request;
extern volatile uint8_t g_learning_mode_request;
extern volatile uint8_t g_training_noise_enable_request;
extern volatile float g_training_noise_amplitude_request_a;
extern volatile uint32_t g_training_noise_seed_request;

/* 所有实验模式共享的速度环遥测量。 */
extern volatile float g_iq_feedback_a;
extern volatile float g_iq_fixed_feedforward_a;
extern volatile float g_iq_learning_feedforward_a;
extern volatile float g_iq_learning_dc_a;
extern volatile float g_iq_learning_ac_a;
extern volatile float g_iq_training_noise_a;
extern volatile float g_iq_eso_raw_a;
extern volatile float g_iq_eso_a;
extern volatile float g_iq_composite_a;
extern volatile float g_fixed_disturbance_nm;
extern volatile float g_learning_disturbance_nm;
extern volatile float g_residual_disturbance_nm;
extern volatile float g_eso_z1_rad_s;
extern volatile float g_eso_z2_rad_s2;
extern volatile float g_eso_speed_error_rad_s;
extern volatile uint32_t g_eso_saturation_count;
extern volatile uint32_t g_eso_update_count;
extern volatile uint8_t g_eso_saturated;
extern volatile uint8_t g_lff_learning_active;

/* 初始化全部实验子模块，并恢复到不叠加补偿的安全基线模式。 */
void Experiment_Control_Init(void);
/* 在快速控制路径中清除补偿状态；用于停机或输入异常时立即回退。 */
void Experiment_Control_FastDisable(void);
/*
 * 速度环更新入口：合成反馈i_q、固定/学习前馈、ESO补偿和扫频注入，
 * 返回已限幅的最终q轴电流指令。
 */
float Experiment_Control_Update(float mechanical_angle_deg,
                                float speed_rpm,
                                float feedback_iq_a);
/* 在低优先级上下文处理学习表更新和扫频复位，避免增加速度环负担。 */
void Experiment_Control_Background(void);

#endif /* FOC_ZT_EXPERIMENT_CONTROL_H */
