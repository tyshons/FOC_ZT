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

static uint32_t Get_Time_Us(void) {
  return __HAL_TIM_GET_COUNTER(&htim1) / 25;
}

void Control_Loop(void) {
  static uint8_t cnt = 0;
  cnt++;
  uint32_t current_time = Get_Time_Us();
  //Biss_process(&current_angle_sp);
  ssi_process(&current_angle_sp);
  //current_angle_sp = encoder_data.angle;
  Get_Electrical_Angle(&theta,&current_angle_sp);

  if (cnt==4) {
    cnt = 0;
    Encoder_Speed_Update(&current_speed_sp,&current_angle_sp);
    //speed_given_sp = PID_Update(&position_pid_inst,(position_given_sp-current_angle_sp), current_time);
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

  ssi_process(&current_angle_sp);
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
