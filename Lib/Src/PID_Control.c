//
// 创建于 2026/3/18。
//

#include "PID_Control.h"

PID_TypeDef position_pid_inst = {
  .Kp = 0.25f,
  .Ki = 0.0f,
  .Kd = 0.0f,
  .output_min = -800.0f,
  .output_max = 800.0f,
  .integral_limit = 100.0f,
  .low_pass_filter_time_constant = 0.01f};
PID_TypeDef speed_pid_inst = {
  .Kp = 1.0f,
  .Ki = 0.7f,
  .Kd = 0.0f,
  .output_min = -10.0f,
  .output_max = 10.0f,
  .integral_limit = 18.0f,
  .low_pass_filter_time_constant = 0.005f};
PID_TypeDef id_pid_inst = {
  .Kp = 1.0f,
  .Ki = 35.0f,
  .Kd = 0.0f,
  .output_min = -30.0f,
  .output_max = 30.0f,
  .integral_limit = 25.0f,
  .low_pass_filter_time_constant = 0.001f};
PID_TypeDef iq_pid_inst = {
  .Kp = 1.0f,
  .Ki = 35.0f,
  .Kd = 0.0f,
  .output_min = -30.0f,
  .output_max = 30.0f,
  .integral_limit = 25.0f,
  .low_pass_filter_time_constant = 0.001f};

void Control_Loop_Init(void) {
  // 初始化位置环 PID
  PID_Init(&position_pid_inst);
  PID_Reset(&position_pid_inst);

  // 初始化速度环 PID
  PID_Init(&speed_pid_inst);
  PID_Reset(&speed_pid_inst);

  // 初始化电流环 PID
  PID_Init(&id_pid_inst);
  PID_Reset(&id_pid_inst);

  PID_Init(&iq_pid_inst);
  PID_Reset(&iq_pid_inst);
}

void PID_Init(PID_TypeDef* pid)
{
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
    pid->has_previous_update = false;
}

// 标准位置式 PID
void PID_SetOutputLimits(PID_TypeDef* pid, float min, float max)
{
    if (min > max) {
        float tmp = min;
        min = max;
        max = tmp;
    }

    pid->output_min = min;
    pid->output_max = max;
}

void PID_SetIntegralLimit(PID_TypeDef* pid, float limit)
{
    if (limit < 0.0f) {
        limit = -limit;
    }

    pid->integral_limit = limit;

    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
}

float PID_Update(PID_TypeDef* pid, float error, uint32_t current_time_us)
{
    if (!pid->enabled) return 0.0f;

    // 首次更新不积分，避免复位后把累计运行时间当作采样周期。
    float dt = 0.0f;
    if (pid->has_previous_update) {
        dt = (current_time_us - pid->last_update_time_us) / 1000000.0f;
        if ((dt <= 0.0f) || (dt > 1.0f)) {
            dt = 0.0f;
        }
    } else {
        pid->has_previous_update = true;
    }

    // 比例项
    float p_term = pid->Kp * error;

    // 积分项（带抗饱和）
    const float previous_integral = pid->integral;
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
    if ((pid->Kd > 0.0f) && (dt > 0.0f)) {
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

    // 条件积分抗饱和：若误差继续把输出推向饱和，撤销本次积分。
    if (pid->anti_windup_enabled) {
        const bool saturating_high =
            (output > pid->output_max) && (error > 0.0f);
        const bool saturating_low =
            (output < pid->output_min) && (error < 0.0f);
        if (saturating_high || saturating_low) {
            pid->integral = previous_integral;
            i_term = pid->Ki * pid->integral;
            output = p_term + i_term + d_term;
        }
    }

    // 输出限幅
    if (output > pid->output_max) output = pid->output_max;
    else if (output < pid->output_min) output = pid->output_min;

    return output;
}

