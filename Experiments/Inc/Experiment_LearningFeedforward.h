#ifndef FOC_ZT_EXPERIMENT_LEARNING_FEEDFORWARD_H
#define FOC_ZT_EXPERIMENT_LEARNING_FEEDFORWARD_H

#include <stdint.h>
#include "Experiment_Config.h"

/* 学习更新所采用的谐波筛选策略。 */
typedef enum {
  POSITION_LFF_MODE_ORDINARY = 0U,
  POSITION_LFF_MODE_SELECTIVE = 1U,
  POSITION_LFF_MODE_GLOBAL = 2U
} Position_Lff_Mode;

/* 两个表格存储区对调试器可见；只能读取g_lff_active_table_index选中的存储区。 */
extern float
    g_lff_table_bank_nm[2][PAPER_LFF_POSITION_BINS];
extern volatile uint8_t g_lff_active_table_index;
extern volatile uint16_t g_lff_current_bin;
extern volatile float g_lff_rho_mean;
extern volatile float g_lff_global_rho;
extern float g_lff_rho_orders[PAPER_LFF_MAX_ORDER + 1U];
extern float g_lff_last_residual_a_nm[PAPER_LFF_MAX_ORDER + 1U];
extern float g_lff_last_residual_b_nm[PAPER_LFF_MAX_ORDER + 1U];
extern float g_lff_learned_a_nm[PAPER_LFF_MAX_ORDER + 1U];
extern float g_lff_learned_b_nm[PAPER_LFF_MAX_ORDER + 1U];
extern volatile uint32_t g_lff_revolution_count;
extern volatile uint32_t g_lff_covered_revolution_count;
extern volatile uint32_t g_lff_update_count;
extern volatile uint32_t g_lff_dropped_revolution_count;
/* 最近一圈的覆盖率、筛选结果和当前已发布学习表的幅值统计。 */
extern volatile float g_lff_last_coverage;
extern volatile uint8_t g_lff_selected_order_count;
extern volatile float g_lff_table_rms_nm;
extern volatile float g_lff_table_peak_abs_nm;
extern volatile float g_lff_table_mean_nm;
/* 为兼容现有通信协议保留；论文算法不学习0阶，因此运行时恒为0。 */
extern volatile float g_lff_dc_torque_nm;

/* 初始化/清除双缓冲学习表及其相关性统计量。 */
void Position_Learning_Init(void);
void Position_Learning_ResetAll(void);
/* 丢弃当前转的采样数据；不会删除已经生效的前馈表。 */
void Position_Learning_AbortCollection(void);
/* 按当前机械角度读取当前生效表中的学习转矩前馈。 */
float Position_Learning_GetOutput(float theta_rad);
/* 在快速路径中按位置桶累计本转的残余扰动。 */
void Position_Learning_Sample(float theta_rad,
                              float residual_torque_nm,
                              uint8_t learning_enabled,
                              Position_Lff_Mode mode);
/* 在后台完成傅里叶分析、相关性筛选和双缓冲表更新。 */
void Position_Learning_Background(void);

#endif /* FOC_ZT_EXPERIMENT_LEARNING_FEEDFORWARD_H */
