#ifndef FOC_ZT_EXPERIMENT_ESO_H
#define FOC_ZT_EXPERIMENT_ESO_H

#include <stdint.h>

typedef struct {
  float bandwidth_rad_s;
  float compensation_gain;
  float current_limit_a;
} Experiment_Eso_Config;

/* ESO单次更新的补偿结果和便于调试器观察的内部状态。 */
typedef struct {
  float compensation_current_a;
  float raw_compensation_current_a;
  float residual_torque_nm;
  float z1_rad_s;
  float z2_rad_s2;
  float speed_error_rad_s;
  uint32_t saturation_count;
  uint32_t update_count;
  uint8_t compensation_saturated;
} Experiment_Eso_Output;

/* 恢复保守默认参数并清除观测器状态。 */
void Experiment_ESO_Init(void);
/* 清除观测器积分状态；下次更新会以当前实测速度重新初始化。 */
void Experiment_ESO_Reset(void);
/*
 * 设置运行时带宽、补偿增益和独立电流限幅。
 * 参数超出安全范围或包含非有限数时拒绝更新；成功后自动复位观测器。
 */
uint8_t Experiment_ESO_Configure(float bandwidth_rad_s,
                                 float compensation_gain,
                                 float current_limit_a);
Experiment_Eso_Config Experiment_ESO_GetConfig(void);
/*
 * 根据实测速度、上一拍实际施加的i_q和已知前馈扰动估计剩余扰动，
 * 输出对应的限幅补偿电流。
 */
Experiment_Eso_Output Experiment_ESO_Update(float speed_rad_s,
                                            float applied_iq_a,
                                            float disturbance_hat_nm);

#endif /* FOC_ZT_EXPERIMENT_ESO_H */
