//
// Created by tyshon on 2026/3/6.
//

#include "tim.h"
#include "FOC_Control.h"
#include <math.h>
#include "PID_Control.h"
#include "encoder.h"
#include "FOC_Math.h"
#include <stdint.h>

// 水平轴（motor_id = 1）
float current_angle_sp = 0.0f;
float current_speed_sp = 0.0f;
float position_given_sp = 0.0f;
float speed_given_sp = 0.0f;
float iq_given_sp = 0.0f;

static uint32_t Get_Time_Us(void) {
  return __HAL_TIM_GET_COUNTER(&htim1) * 10; // 假设 TIM1 为 10MHz，1tick=0.1us
}

void Control_Loop(void) {
  static uint8_t cnt = 0;
  cnt++;

  if (encoder_data.is_valid) {
    current_angle_sp = encoder_data.angle;
  }

  if (cnt==20)
  {
    cnt = 0;

    uint32_t current_time = Get_Time_Us();
    float position_error = position_given_sp - current_angle_sp;
    speed_given_sp = PID_Update(&position_pid_inst,position_error, current_time);
    iq_given_sp = PID_Update(&speed_pid_inst,(speed_given_sp - current_speed_sp), current_time);
  }
  float theta ;
  Get_Electrical_Angle(&theta);

  float i_a = g_adc_current[0];
  float i_b = g_adc_current[1];
  float i_c = g_adc_current[2];

  float i_alpha = 0.0f , i_beta = 0.0f;
  clarke_transform(i_a, i_b, i_c, &i_alpha, &i_beta);

  float i_d = 0.0f, i_q = 0.0f;
  park_transform(i_alpha, i_beta, theta, &i_d, &i_q);

  uint32_t current_time = Get_Time_Us();
  float u_d = PID_Update(&iq_pid_inst, (0-i_d), current_time);
  float u_q = PID_Update(&iq_pid_inst,(iq_given_sp-i_q),current_time);

  float mag = sqrtf(u_d * u_d + u_q * u_q);
  if (mag > VOLTAGE_LIMIT) {
    u_d = u_d * VOLTAGE_LIMIT / mag;
    u_q = u_q * VOLTAGE_LIMIT / mag;
  }

  float u_alpha, u_beta;
  ipark_transform(u_d, u_q, theta, &u_alpha, &u_beta);

  uint32_t ccrA, ccrB, ccrC;
  svpwm_generate(u_alpha, u_beta, g_adc_vbus, &ccrA, &ccrB, &ccrC);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccrA);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccrB);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccrC);

}

void Motor_Enable(void) {
  HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin, GPIO_PIN_SET);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

void Motor_Disable(void) {
  HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin, GPIO_PIN_RESET);

  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}
