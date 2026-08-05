//
// 创建于 2026/3/6。
//

#include "tim.h"
#include "FOC_Control.h"
#include "PID_Control.h"
#include "FOC_Math.h"
#include "ssi.h"
#include <math.h>
#include <stdint.h>
#include "pos_process.h"
#include "Experiment_Config.h"
#include "Experiment_Control.h"

#define PI 3.14159265358979323846f

float open_loop_theta = 0.0f;
float open_loop_speed = 80.0f;  // 期望转动的电角速度 (rad/s)
float open_loop_voltage = 5.0f; // 开环电压 (V)，不要给太大，防止发热

// 方位轴控制变量。
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
static volatile FOC_FaultCode foc_fault_code = FOC_FAULT_NONE;
static volatile uint8_t foc_position_target_valid = 0U;
static uint32_t enable_sample_count = 0U;
static uint32_t enable_start_tick_ms = 0U;
static uint32_t control_time_us = 0U;
static uint16_t speed_loop_count = 0U;
static uint32_t speed_loop_elapsed_us = 0U;
static uint32_t previous_control_cycle_count = 0U;
static uint8_t control_cycle_counter_valid = 0U;

static uint32_t measure_control_elapsed_us(void)
{
  /*
   * 用内核周期计数器测量相邻控制回调的真实间隔。
   * 目标调度仍为20 kHz/4 kHz；这里只修正中断抖动或漏回调造成的时间尺度误差。
   */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    control_cycle_counter_valid = 0U;
  }

  const uint32_t current_cycle_count = DWT->CYCCNT;
  if (control_cycle_counter_valid == 0U) {
    previous_control_cycle_count = current_cycle_count;
    control_cycle_counter_valid = 1U;
    return FOC_CURRENT_LOOP_PERIOD_US;
  }

  const uint32_t elapsed_cycles =
      current_cycle_count - previous_control_cycle_count;
  previous_control_cycle_count = current_cycle_count;
  const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
  if (cycles_per_us == 0U) {
    return FOC_CURRENT_LOOP_PERIOD_US;
  }

  const uint32_t elapsed_us =
      (elapsed_cycles + (cycles_per_us / 2U)) / cycles_per_us;
  /* 调试器暂停或周期计数异常时，不把长停顿灌入PID积分。 */
  if ((elapsed_us < (FOC_CURRENT_LOOP_PERIOD_US / 4U)) ||
      (elapsed_us > 1000U)) {
    return FOC_CURRENT_LOOP_PERIOD_US;
  }
  return elapsed_us;
}

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
  speed_loop_count = 0U;
  speed_loop_elapsed_us = 0U;
  control_cycle_counter_valid = 0U;
  Experiment_Control_FastDisable();
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

static void stop_power_stage(FOC_FaultCode fault_code)
{
  /* 必须先关闭驱动器，再修改控制状态和定时器输出。 */
  HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin, GPIO_PIN_RESET);
  foc_power_state = FOC_POWER_DISABLED;
  foc_fault_code = fault_code;
  reset_control_state();
  current_speed_sp = 0.0f;
  position_given_sp = current_angle_sp;
  foc_position_target_valid =
      SSI_IsFrameFresh(SSI_MAX_FRAME_AGE_MS) ? 1U : 0U;
  Encoder_Speed_Reset(current_angle_sp);
  set_pwm_neutral();

  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
  ADC_ResetCurrentProcessing();
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

float FOC_GetSpeedTarget(void)
{
  return speed_given_sp;
}

float FOC_GetIqTarget(void)
{
  return iq_given_sp;
}

uint8_t FOC_IsPositionTargetValid(void)
{
  return (foc_position_target_valid &&
          foc_control_mode == FOC_CONTROL_MODE_POSITION) ? 1U : 0U;
}


void Control_Loop(void) {
  const uint32_t elapsed_control_us = measure_control_elapsed_us();
  control_time_us += elapsed_control_us;
  const uint32_t current_time = control_time_us;
  ssi_process();
  Get_Electrical_Angle(&theta,&current_angle_sp);

  if (foc_power_state == FOC_POWER_WAIT_ENCODER) {
    if (((uint32_t)(SSI_GetValidSampleCount() - enable_sample_count) >=
         FOC_ENABLE_MIN_FRESH_SAMPLES) &&
        (SSI_IsFrameFresh(SSI_MAX_FRAME_AGE_MS) != 0U)) {
      position_given_sp = current_angle_sp;
      current_speed_sp = 0.0f;
      Encoder_Speed_Reset(current_angle_sp);
      reset_control_state();
      set_pwm_neutral();
      foc_position_target_valid = 1U;
      foc_power_state = FOC_POWER_ENABLED;
      HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin, GPIO_PIN_SET);
    } else if ((uint32_t)(HAL_GetTick() - enable_start_tick_ms) >=
               SSI_ENABLE_ACQUIRE_TIMEOUT_MS) {
      stop_power_stage(FOC_FAULT_ENCODER_START_TIMEOUT);
    }
    return;
  }

  if (foc_power_state != FOC_POWER_ENABLED) {
    return;
  }

  if (SSI_IsFrameFresh(SSI_MAX_FRAME_AGE_MS) == 0U) {
    stop_power_stage(FOC_FAULT_ENCODER_RUNTIME_TIMEOUT);
    return;
  }

  speed_loop_count++;
  speed_loop_elapsed_us += elapsed_control_us;

  if (speed_loop_count >= FOC_SPEED_LOOP_DIVIDER) {
    speed_loop_count = 0U;
    const float measured_speed_period_s =
        (float)speed_loop_elapsed_us / 1000000.0f;
    speed_loop_elapsed_us = 0U;
    Encoder_Speed_Update(&current_speed_sp,
                         &current_angle_sp,
                         measured_speed_period_s);
    if (foc_control_mode == FOC_CONTROL_MODE_POSITION) {
      float position_error = shortest_angle_error_deg(position_given_sp, current_angle_sp);
      speed_given_sp = PID_Update(&position_pid_inst, position_error, current_time);
    }
    const float feedback_iq = PID_Update(&speed_pid_inst,(speed_given_sp - current_speed_sp),current_time);
    iq_given_sp = Experiment_Control_Update(current_angle_sp,current_speed_sp,feedback_iq);
  }

  float i_a = -g_adc_current[0];
  float i_b = -g_adc_current[1];
  float i_c = -g_adc_current[2];

  clarke_transform(i_a, i_b, i_c, &i_alpha, &i_beta);
  park_transform(i_alpha, i_beta, theta, &i_d, &i_q);

  u_d = PID_Update(&id_pid_inst, (id_given_sp-i_d),current_time);
  u_q = PID_Update(&iq_pid_inst,(iq_given_sp-i_q),current_time);

  float voltage_limit = g_adc_vbus * EXPERIMENT_VOLTAGE_UTILIZATION;
  if (voltage_limit > VOLTAGE_LIMIT) {
    voltage_limit = VOLTAGE_LIMIT;
  }

  const float voltage_magnitude_sq = u_d * u_d + u_q * u_q;
  if ((voltage_limit > 0.0f) &&
      (voltage_magnitude_sq > voltage_limit * voltage_limit)) {
    const float scale = voltage_limit / sqrtf(voltage_magnitude_sq);
    u_d *= scale;
    u_q *= scale;
  }

  ipark_transform(u_d, u_q, theta, &u_alpha, &u_beta);

  svpwm_generate(u_alpha, u_beta, g_adc_vbus, &ccrA, &ccrB, &ccrC);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccrA);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccrB);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccrC);

}

void Control_Loop_test(void) {
  const float dt = FOC_CURRENT_LOOP_PERIOD_S;

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
  // 4. 设置固定开环电压，此时Ud=V、Uq=0。
  // 这样磁场会拉着转子同步旋转
  float u_d_test = open_loop_voltage;
  float u_q_test = 0.0f;

  // 5. 坐标变换，此处传入自增的开环角度。
  ipark_transform(u_d_test, u_q_test, open_loop_theta, &u_alpha, &u_beta);

  // 6. 输出到空间矢量脉宽调制模块。
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

  if (g_adc_calibrated == 0U) {
    restore_interrupt_state(primask);
    return;
  }

  /* 功率级保持关闭，先启动控制时基并获取编码器数据。 */
  HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin, GPIO_PIN_RESET);
  reset_control_state();
  ADC_ResetCurrentProcessing();
  set_pwm_neutral();
  foc_position_target_valid = 0U;
  foc_fault_code = FOC_FAULT_NONE;
  SSI_RearmValidation();
  enable_sample_count = SSI_GetValidSampleCount();
  enable_start_tick_ms = HAL_GetTick();
  foc_power_state = FOC_POWER_WAIT_ENCODER;
  HAL_StatusTypeDef pwm_status = HAL_OK;
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) pwm_status = HAL_ERROR;
  if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) pwm_status = HAL_ERROR;
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) pwm_status = HAL_ERROR;
  if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) pwm_status = HAL_ERROR;
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) pwm_status = HAL_ERROR;
  if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) pwm_status = HAL_ERROR;

  if (pwm_status != HAL_OK) {
    stop_power_stage(FOC_FAULT_PWM_START_FAILED);
  }

  restore_interrupt_state(primask);
}

void Motor_Disable(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  stop_power_stage(FOC_FAULT_NONE);

  restore_interrupt_state(primask);
}

uint8_t FOC_GetPowerState(void)
{
  return (uint8_t)foc_power_state;
}

FOC_FaultCode FOC_GetFaultCode(void)
{
  return foc_fault_code;
}
