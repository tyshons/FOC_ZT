//
// Created by tyshon on 2026/3/6.
//

#include "tim.h"
#include "FOC_Control.h"
//#include <math.h>
#include "PID_Control.h"
#include "encoder.h"
#include "FOC_Math.h"
//#include "BISS_C.h"
#include "ssi.h"
#include <math.h>
#include <stdint.h>
#include "pos_process.h"

#define PI 3.14159265358979323846f

float open_loop_theta = 0.0f;
float open_loop_speed = 80.0f;  // 期望转动的电角速度 (rad/s)
float open_loop_voltage = 5.0f; // 开环电压 (V)，不要给太大，防止发热

// 水平轴（motor_id = 1）
float current_angle_sp = 0.0f;
float current_speed_sp = 0.0f;
float position_given_sp = 100.0f;
float speed_given_sp = 0.0f;
float id_given_sp = 0.0f;
float iq_given_sp = 0.0f;

float theta;
float i_alpha = 0.0f , i_beta = 0.0f;
float i_d = 0.0f, i_q = 0.0f;
float u_d = 0.0f, u_q = 0.0f;
float u_alpha, u_beta;
uint32_t ccrA, ccrB, ccrC;

static volatile FOC_ControlMode foc_control_mode = FOC_CONTROL_MODE_POSITION;

typedef enum {
  FOC_POWER_DISABLED = 0,
  FOC_POWER_WAIT_ENCODER,
  FOC_POWER_ENABLED
} FOC_PowerState;

#define FOC_ENABLE_MIN_FRESH_SAMPLES 2U

static volatile FOC_PowerState foc_power_state = FOC_POWER_DISABLED;
static volatile uint8_t foc_position_target_valid = 0U;
static uint32_t enable_sample_count = 0U;

static float shortest_angle_error_deg(float target_deg, float actual_deg)
{
  float error = fmodf(target_deg - actual_deg + 180.0f, 360.0f);
  if (error < 0.0f) {
    error += 360.0f;
  }
  return error - 180.0f;
}

static void restore_interrupt_state(uint32_t primask)
{
  if ((primask & 1U) == 0U) {
    __enable_irq();
  }
}

static void reset_control_state(void)
{
  PID_Reset(&position_pid_inst);
  PID_Reset(&speed_pid_inst);
  PID_Reset(&id_pid_inst);
  PID_Reset(&iq_pid_inst);
  speed_given_sp = 0.0f;
  id_given_sp = 0.0f;
  iq_given_sp = 0.0f;
  u_d = 0.0f;
  u_q = 0.0f;
  foc_control_mode = FOC_CONTROL_MODE_POSITION;
}

static void set_pwm_neutral(void)
{
  uint32_t neutral_compare = htim1.Init.Period / 2U;
  ccrA = neutral_compare;
  ccrB = neutral_compare;
  ccrC = neutral_compare;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, neutral_compare);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, neutral_compare);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, neutral_compare);
}

void FOC_SetPositionTarget(float target_deg)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  if (foc_control_mode != FOC_CONTROL_MODE_POSITION) {
    PID_Reset(&position_pid_inst);
    PID_Reset(&speed_pid_inst);
  }
  position_given_sp = target_deg;
  foc_control_mode = FOC_CONTROL_MODE_POSITION;
  foc_position_target_valid = 1U;

  restore_interrupt_state(primask);
}

void FOC_SetSpeedTarget(float target_rpm)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  if (foc_control_mode != FOC_CONTROL_MODE_SPEED) {
    PID_Reset(&speed_pid_inst);
  }
  speed_given_sp = target_rpm;
  foc_control_mode = FOC_CONTROL_MODE_SPEED;
  foc_position_target_valid = 0U;

  restore_interrupt_state(primask);
}

FOC_ControlMode FOC_GetControlMode(void)
{
  return foc_control_mode;
}

uint8_t FOC_IsPositionTargetValid(void)
{
  return (foc_position_target_valid &&
          foc_control_mode == FOC_CONTROL_MODE_POSITION) ? 1U : 0U;
}


static uint32_t Get_Time_Us(void) {
  return __HAL_TIM_GET_COUNTER(&htim1) / 25;
}

void Control_Loop(void) {
  static uint8_t cnt = 0;
  uint32_t current_time = Get_Time_Us();
  //Biss_process(&current_angle_sp);//bissc
  //current_angle_sp = encoder_data.angle;//485
  ssi_process();
  Get_Electrical_Angle(&theta,&current_angle_sp);

  if (foc_power_state == FOC_POWER_WAIT_ENCODER) {
    if ((uint32_t)(SSI_GetValidSampleCount() - enable_sample_count) >=
        FOC_ENABLE_MIN_FRESH_SAMPLES) {
      position_given_sp = current_angle_sp;
      current_speed_sp = 0.0f;
      Encoder_Speed_Reset(current_angle_sp);
      reset_control_state();
      set_pwm_neutral();
      foc_position_target_valid = 1U;
      foc_power_state = FOC_POWER_ENABLED;
      HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin, GPIO_PIN_SET);
    }
    return;
  }

  if (foc_power_state != FOC_POWER_ENABLED) {
    return;
  }

  cnt++;

  if (cnt==5) {
    cnt = 0;
    Encoder_Speed_Update(&current_speed_sp,&current_angle_sp);
    if (foc_control_mode == FOC_CONTROL_MODE_POSITION) {
      float position_error = shortest_angle_error_deg(position_given_sp, current_angle_sp);
      speed_given_sp = PID_Update(&position_pid_inst, position_error, current_time);
    }
    iq_given_sp = PID_Update(&speed_pid_inst,(speed_given_sp - current_speed_sp), current_time);
  }

  float i_a = -g_adc_current[0];
  float i_b = -g_adc_current[1];
  float i_c = -g_adc_current[2];

  clarke_transform(i_a, i_b, i_c, &i_alpha, &i_beta);
  park_transform(i_alpha, i_beta, theta, &i_d, &i_q);

  u_d = PID_Update(&id_pid_inst, (id_given_sp-i_d),current_time);
  u_q = PID_Update(&iq_pid_inst,(iq_given_sp-i_q),current_time);

  ipark_transform(u_d, u_q, theta, &u_alpha, &u_beta);

  svpwm_generate(u_alpha, u_beta, g_adc_vbus, &ccrA, &ccrB, &ccrC);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccrA);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccrB);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccrC);

}

void Control_Loop_test(void) {
  // 1. 获取时间步长 dt (假设此函数每 100us 或 1ms 运行一次)
  // 如果没有精确 dt，可以直接用一个固定增量
  float dt = 0.00005f; // 假设 PWM 频率为 10kHz

  ssi_process();
  Get_Electrical_Angle(&theta,&current_angle_sp);
  // 2. 让电角度自增
  open_loop_theta += open_loop_speed * dt;

  // 3. 角度归一化 [0, 2π]
  if (open_loop_theta > 2.0f * PI) open_loop_theta -= 2.0f * PI;
  if (open_loop_theta < 0.0f)      open_loop_theta += 2.0f * PI;


  float i_a = -g_adc_current[0];
  float i_b = -g_adc_current[1];
  float i_c = -g_adc_current[2];


  clarke_transform(i_a, i_b, i_c, &i_alpha, &i_beta);
  park_transform(i_alpha, i_beta, theta, &i_d, &i_q);
  // 4. 设置固定的开环电压 (此时 Ud=V, Uq=0)
  // 这样磁场会拉着转子同步旋转
  float u_d_test = open_loop_voltage;
  float u_q_test = 0.0f;

  // 5. 坐标变换 (注意这里传入的是我们自增的 open_loop_theta)
  ipark_transform(u_d_test, u_q_test, open_loop_theta, &u_alpha, &u_beta);

  // 6. 输出到 SVPWM
  svpwm_generate(u_alpha, u_beta, g_adc_vbus, &ccrA, &ccrB, &ccrC);

  // 7. 更新硬件占空比
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccrA);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccrB);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccrC);
}

void Motor_Enable(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  if (foc_power_state == FOC_POWER_ENABLED ||
      foc_power_state == FOC_POWER_WAIT_ENCODER) {
    restore_interrupt_state(primask);
    return;
  }

  /* Start timing and encoder sampling first while the power stage stays off. */
  HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin, GPIO_PIN_RESET);
  reset_control_state();
  set_pwm_neutral();
  foc_position_target_valid = 0U;
  SSI_RearmValidation();
  enable_sample_count = SSI_GetValidSampleCount();
  foc_power_state = FOC_POWER_WAIT_ENCODER;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  restore_interrupt_state(primask);
}

void Motor_Disable(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin, GPIO_PIN_RESET);
  foc_power_state = FOC_POWER_DISABLED;
  reset_control_state();
  current_speed_sp = 0.0f;
  position_given_sp = current_angle_sp;
  foc_position_target_valid = (SSI_GetValidSampleCount() > 0U) ? 1U : 0U;
  Encoder_Speed_Reset(current_angle_sp);
  set_pwm_neutral();

  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

  restore_interrupt_state(primask);
}

uint8_t FOC_GetPowerState(void)
{
  return (uint8_t)foc_power_state;
}
