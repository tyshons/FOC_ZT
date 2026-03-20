//
// Created by tyshon on 2026/3/18.
//

#include "PID_Control.h"
#include "adc.h"

float iq_num[4], iq_den[4];
float speed_num[4], speed_den[4];
float pos_num[4], pos_den[4];

pid_state_t pos_pid_sp = {0}; // 水平轴位置环状态
pid_state_t spd_pid_sp = {0}; // 水平轴速度环状态
pid_state_t iq_pid_sp = {0};

// 新增：标准 PID 控制器实例
PID_TypeDef position_pid_inst = {0};
PID_TypeDef speed_pid_inst = {0};
PID_TypeDef iq_pid_inst = {0};

void Control_Loop_Init(void) {
  // 初始化位置环 PID（参数需要根据实际调整）
  PID_Init(&position_pid_inst, 5.0f, 0.5f, 0.1f);
  PID_SetOutputLimits(&position_pid_inst, -1000.0f, 1000.0f); // 速度限幅
  PID_SetIntegralLimit(&position_pid_inst, 100.0f);
  position_pid_inst.low_pass_filter_time_constant = 0.01f; // 10ms 低通滤波

  // 初始化速度环 PID
  PID_Init(&speed_pid_inst, 2.0f, 0.8f, 0.05f);
  PID_SetOutputLimits(&speed_pid_inst, -8400.0f, 8400.0f); // PWM 限幅
  PID_SetIntegralLimit(&speed_pid_inst, 500.0f);
  speed_pid_inst.low_pass_filter_time_constant = 0.005f; // 5ms 低通滤波

  // 初始化电流环 PID
  PID_Init(&iq_pid_inst, 1.5f, 1.0f, 0.01f);
  PID_SetOutputLimits(&iq_pid_inst, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  PID_SetIntegralLimit(&iq_pid_inst, INTEGRAL_LIMIT);
  iq_pid_inst.low_pass_filter_time_constant = 0.001f; // 1ms 低通滤波
}

void PID_Init(PID_TypeDef* pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->output_min = -10.0f;
    pid->output_max = 10.0f;
    pid->integral_limit = 5.0f;

    pid->enabled = true;
    pid->anti_windup_enabled = true;

    PID_Reset(pid);
}

void PID_Reset(PID_TypeDef* pid)
{
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->last_error = 0.0f;
    pid->last_update_time_us = 0;
}

void PID_SetOutputLimits(PID_TypeDef* pid, float min, float max)
{
    if (min >= max) return;
    pid->output_min = min;
    pid->output_max = max;
}

void PID_SetIntegralLimit(PID_TypeDef* pid, float limit)
{
    pid->integral_limit = limit;
}

// 标准位置式 PID
float PID_Update(PID_TypeDef* pid, float error, uint32_t current_time_us)
{
    if (!pid->enabled) return 0.0f;

    // 计算时间间隔 (秒)
    float dt = (current_time_us - pid->last_update_time_us) / 1000000.0f;
    if (dt <= 0.0f || dt > 1.0f) dt = 0.001f; // 默认 1ms

    // 比例项
    float p_term = pid->Kp * error;

    // 积分项（带抗饱和）
    pid->integral += error * dt;

    if (pid->anti_windup_enabled) {
        // 钳位积分
        if (pid->integral > pid->integral_limit) {
            pid->integral = pid->integral_limit;
        } else if (pid->integral < -pid->integral_limit) {
            pid->integral = -pid->integral_limit;
        }
    }

    float i_term = pid->Ki * pid->integral;

    // 微分项（带低通滤波）
    float d_term = 0.0f;
    if (pid->Kd > 0.0f) {
        float derivative_raw = (error - pid->last_error) / dt;

        // 一阶低通滤波
        float alpha = pid->low_pass_filter_time_constant /
                      (pid->low_pass_filter_time_constant + dt);
        pid->derivative = alpha * pid->derivative + (1.0f - alpha) * derivative_raw;

        d_term = pid->Kd * pid->derivative;
    }

    // 保存状态
    pid->last_error = error;
    pid->last_update_time_us = current_time_us;

    // 计算总输出
    float output = p_term + i_term + d_term;

    // 输出限幅
    if (output > pid->output_max) output = pid->output_max;
    else if (output < pid->output_min) output = pid->output_min;

    return output;
}

