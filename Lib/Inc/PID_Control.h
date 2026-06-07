//
// Created by tyshon on 2026/3/18.
//

#ifndef FOC_ZT_SETPARA_H
#define FOC_ZT_SETPARA_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
  float Kp;
  float Ki;
  float Kd;

  // 输出限幅
  float output_min;
  float output_max;

  // 积分限幅（防饱和）
  float integral_limit;

  // 微分滤波器
  float low_pass_filter_time_constant;

  // 内部状态
  float integral;
  float derivative;
  float last_error;
  uint32_t last_update_time_us;

  // 配置标志
  bool enabled;
  bool anti_windup_enabled;
} PID_TypeDef;

extern PID_TypeDef position_pid_inst;
extern PID_TypeDef speed_pid_inst;
extern PID_TypeDef id_pid_inst;
extern PID_TypeDef iq_pid_inst;

// 初始化函数
void Control_Loop_Init(void);
void PID_Init(PID_TypeDef* pid);
void PID_SetOutputLimits(PID_TypeDef* pid, float min, float max);
void PID_SetIntegralLimit(PID_TypeDef* pid, float limit);
void PID_Reset(PID_TypeDef* pid);

// 计算函数
float PID_Update(PID_TypeDef* pid, float error, uint32_t current_time_us);

#endif //FOC_ZT_SETPARA_H