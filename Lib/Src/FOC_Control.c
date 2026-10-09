//
// 创建于 2026/3/6。
//

#include "tim.h"
#include "FOC_Control.h"
#include "PID_Control.h"
#include "FOC_Math.h"
#include "ssi.h"
#include "pos_process.h"
#include "adc.h"
#include "Experiment_Config.h"
#include "Experiment_Control.h"

#include <math.h>
#include <stdint.h>

#define PI 3.14159265358979323846f
#define FOC_ENABLE_MIN_FRESH_SAMPLES 2U

float open_loop_theta_sp = 0.0f;
float open_loop_theta_fy = 0.0f;
float open_loop_speed_sp = 80.0f;
float open_loop_speed_fy = 80.0f;
float open_loop_voltage_sp = 5.0f;
float open_loop_voltage_fy = 5.0f;

float current_angle_sp = 0.0f;
float current_angle_fy = 0.0f;
float current_speed_sp = 0.0f;
float current_speed_fy = 0.0f;
float position_given_sp = 0.0f;
float position_given_fy = 0.0f;
float speed_given_sp = 0.0f;
float speed_given_fy = 0.0f;
float id_given_sp = 0.0f;
float id_given_fy = 0.0f;
float iq_given_sp = 0.0f;
float iq_given_fy = 0.0f;

float electrical_offset_deg_sp = MOTOR_ELECTRICAL_OFFSET_DEG_SP_DEFAULT;
float electrical_offset_deg_fy = MOTOR_ELECTRICAL_OFFSET_DEG_FY_DEFAULT;

float theta_sp = 0.0f;
float theta_fy = 0.0f;
float i_alpha_sp = 0.0f;
float i_alpha_fy = 0.0f;
float i_beta_sp = 0.0f;
float i_beta_fy = 0.0f;
float i_d_sp = 0.0f;
float i_d_fy = 0.0f;
float i_q_sp = 0.0f;
float i_q_fy = 0.0f;
float u_d_sp = 0.0f;
float u_d_fy = 0.0f;
float u_q_sp = 0.0f;
float u_q_fy = 0.0f;
float u_alpha_sp = 0.0f;
float u_alpha_fy = 0.0f;
float u_beta_sp = 0.0f;
float u_beta_fy = 0.0f;
uint32_t ccrA_sp = 0U;
uint32_t ccrA_fy = 0U;
uint32_t ccrB_sp = 0U;
uint32_t ccrB_fy = 0U;
uint32_t ccrC_sp = 0U;
uint32_t ccrC_fy = 0U;

typedef enum {
  FOC_POWER_DISABLED = 0,
  FOC_POWER_WAIT_ENCODER,
  FOC_POWER_ENABLED
} FOC_PowerState;

static volatile FOC_ControlMode foc_control_mode_sp = FOC_CONTROL_MODE_POSITION;
static volatile FOC_ControlMode foc_control_mode_fy = FOC_CONTROL_MODE_POSITION;
static volatile FOC_PowerState foc_power_state_sp = FOC_POWER_DISABLED;
static volatile FOC_PowerState foc_power_state_fy = FOC_POWER_DISABLED;
static volatile FOC_FaultCode foc_fault_code_sp = FOC_FAULT_NONE;
static volatile FOC_FaultCode foc_fault_code_fy = FOC_FAULT_NONE;
static volatile uint8_t foc_position_target_valid_sp = 0U;
static volatile uint8_t foc_position_target_valid_fy = 0U;

static volatile uint8_t electrical_calibration_active_sp = 0U;
static volatile uint8_t electrical_calibration_active_fy = 0U;
static float electrical_calibration_ud_sp = 0.0f;
static float electrical_calibration_ud_fy = 0.0f;

static uint32_t enable_sample_count_sp = 0U;
static uint32_t enable_sample_count_fy = 0U;
static uint32_t enable_start_tick_ms_sp = 0U;
static uint32_t enable_start_tick_ms_fy = 0U;
static uint32_t control_time_us_sp = 0U;
static uint32_t control_time_us_fy = 0U;
static uint16_t speed_loop_count_sp = 0U;
static uint16_t speed_loop_count_fy = 0U;
static uint8_t ssi_request_count_sp = 0U;
static uint8_t ssi_request_count_fy = 0U;
static uint32_t speed_loop_elapsed_us_sp = 0U;
static uint32_t speed_loop_elapsed_us_fy = 0U;
static uint32_t previous_control_cycle_count_sp = 0U;
static uint32_t previous_control_cycle_count_fy = 0U;
static uint8_t control_cycle_counter_valid_sp = 0U;
static uint8_t control_cycle_counter_valid_fy = 0U;
static uint8_t fy_soft_limit_blocked = 0U;
static int8_t fy_soft_limit_direction = 0;

static uint32_t measure_control_elapsed_us(FOC_Axis axis)
{
  uint32_t *previous_count = (axis == FOC_AXIS_FY)
                                 ? &previous_control_cycle_count_fy
                                 : &previous_control_cycle_count_sp;
  uint8_t *counter_valid = (axis == FOC_AXIS_FY)
                               ? &control_cycle_counter_valid_fy
                               : &control_cycle_counter_valid_sp;

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    control_cycle_counter_valid_sp = 0U;
    control_cycle_counter_valid_fy = 0U;
  }

  const uint32_t current_count = DWT->CYCCNT;
  if (*counter_valid == 0U) {
    *previous_count = current_count;
    *counter_valid = 1U;
    return FOC_CURRENT_LOOP_PERIOD_US;
  }

  const uint32_t elapsed_cycles = current_count - *previous_count;
  *previous_count = current_count;
  const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
  if (cycles_per_us == 0U) {
    return FOC_CURRENT_LOOP_PERIOD_US;
  }

  const uint32_t elapsed_us =
      (elapsed_cycles + (cycles_per_us / 2U)) / cycles_per_us;
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

static float normalize_angle_deg(float angle_deg)
{
  float angle = fmodf(angle_deg, 360.0f);
  if (angle < 0.0f) {
    angle += 360.0f;
  }
  return angle;
}

static uint8_t fy_angle_inside_limit(float angle_deg);

static float clamp_fy_target_deg(float target_deg)
{
  const float target = normalize_angle_deg(target_deg);
  if (fy_angle_inside_limit(target) != 0U) {
    return target;
  }
  const float forward_to_min =
      normalize_angle_deg(FY_SOFT_LIMIT_MIN_DEG - target);
  const float backward_to_max =
      normalize_angle_deg(target - FY_SOFT_LIMIT_MAX_DEG);
  return (forward_to_min <= backward_to_max)
             ? FY_SOFT_LIMIT_MIN_DEG
             : FY_SOFT_LIMIT_MAX_DEG;
}

static uint8_t fy_angle_inside_limit(float angle_deg)
{
  const float angle = normalize_angle_deg(angle_deg);
  if (FY_SOFT_LIMIT_MIN_DEG <= FY_SOFT_LIMIT_MAX_DEG) {
    return ((angle >= FY_SOFT_LIMIT_MIN_DEG) &&
            (angle <= FY_SOFT_LIMIT_MAX_DEG)) ? 1U : 0U;
  }
  return ((angle >= FY_SOFT_LIMIT_MIN_DEG) ||
          (angle <= FY_SOFT_LIMIT_MAX_DEG)) ? 1U : 0U;
}

static float fy_position_error_deg(float target_deg, float actual_deg)
{
  const float actual = normalize_angle_deg(actual_deg);
  if (fy_angle_inside_limit(actual) != 0U) {
    const float target_position = normalize_angle_deg(
        clamp_fy_target_deg(target_deg) - FY_SOFT_LIMIT_MIN_DEG);
    const float actual_position =
        normalize_angle_deg(actual - FY_SOFT_LIMIT_MIN_DEG);
    return target_position - actual_position;
  }

  /* 已在禁区时，先沿较短方向退回最近的限位边界。 */
  const float forward_to_min =
      normalize_angle_deg(FY_SOFT_LIMIT_MIN_DEG - actual);
  const float backward_to_max =
      normalize_angle_deg(actual - FY_SOFT_LIMIT_MAX_DEG);
  return (forward_to_min <= backward_to_max)
             ? forward_to_min
             : -backward_to_max;
}

static uint8_t fy_speed_is_outward(float target_rpm, float actual_deg)
{
  const float actual = normalize_angle_deg(actual_deg);
  if (fy_angle_inside_limit(actual) != 0U) {
    const float limit_length = normalize_angle_deg(
        FY_SOFT_LIMIT_MAX_DEG - FY_SOFT_LIMIT_MIN_DEG);
    const float actual_position =
        normalize_angle_deg(actual - FY_SOFT_LIMIT_MIN_DEG);
    if ((actual_position <= 0.0f) && (target_rpm < 0.0f)) {
      fy_soft_limit_direction = -1;
      return 1U;
    }
    if ((actual_position >= limit_length) && (target_rpm > 0.0f)) {
      fy_soft_limit_direction = 1;
      return 1U;
    }
    fy_soft_limit_direction = 0;
    return 0U;
  }

  if (fy_soft_limit_direction == 0) {
    const float forward_to_min =
        normalize_angle_deg(FY_SOFT_LIMIT_MIN_DEG - actual);
    const float backward_to_max =
        normalize_angle_deg(actual - FY_SOFT_LIMIT_MAX_DEG);
    fy_soft_limit_direction = (forward_to_min <= backward_to_max) ? -1 : 1;
  }
  return (((fy_soft_limit_direction < 0) && (target_rpm < 0.0f)) ||
          ((fy_soft_limit_direction > 0) && (target_rpm > 0.0f)))
             ? 1U
             : 0U;
}

static void restore_interrupt_state(uint32_t primask)
{
  if ((primask & 1U) == 0U) {
    __enable_irq();
  }
}

static uint32_t ssi_valid_sample_count(FOC_Axis axis)
{
  return (axis == FOC_AXIS_FY)
             ? SSI_GetValidSampleCount_fy()
             : SSI_GetValidSampleCount_sp();
}

static uint8_t ssi_frame_is_fresh(FOC_Axis axis)
{
  return (axis == FOC_AXIS_FY)
             ? SSI_IsFrameFresh_fy(SSI_MAX_FRAME_AGE_MS)
             : SSI_IsFrameFresh_sp(SSI_MAX_FRAME_AGE_MS);
}

static void ssi_process_axis(FOC_Axis axis)
{
  if (axis == FOC_AXIS_FY) {
    ssi_process_fy();
  } else {
    ssi_process_sp();
  }
}

static void ssi_rearm_axis(FOC_Axis axis)
{
  if (axis == FOC_AXIS_FY) {
    SSI_RearmValidation_fy();
  } else {
    SSI_RearmValidation_sp();
  }
}

static void encoder_speed_reset_axis(FOC_Axis axis, float angle_deg)
{
  if (axis == FOC_AXIS_FY) {
    Encoder_Speed_Reset_fy(angle_deg);
  } else {
    Encoder_Speed_Reset_sp(angle_deg);
  }
}

static void encoder_speed_update_axis(FOC_Axis axis,
                                      float *speed,
                                      const float *angle,
                                      float period_s)
{
  if (axis == FOC_AXIS_FY) {
    Encoder_Speed_Update_fy(speed, angle, period_s);
  } else {
    Encoder_Speed_Update_sp(speed, angle, period_s);
  }
}

static void adc_reset_axis(FOC_Axis axis)
{
  if (axis == FOC_AXIS_FY) {
    ADC_ResetCurrentProcessing_fy();
  } else {
    ADC_ResetCurrentProcessing_sp();
  }
}

static void reset_control_state(FOC_Axis axis)
{
  if (axis == FOC_AXIS_FY) {
    PID_Reset(&position_pid_inst_fy);
    PID_Reset(&speed_pid_inst_fy);
    PID_Reset(&id_pid_inst_fy);
    PID_Reset(&iq_pid_inst_fy);
    speed_given_fy = 0.0f;
    id_given_fy = 0.0f;
    iq_given_fy = 0.0f;
    u_d_fy = 0.0f;
    u_q_fy = 0.0f;
    foc_control_mode_fy = FOC_CONTROL_MODE_POSITION;
    speed_loop_count_fy = 0U;
    ssi_request_count_fy = 0U;
    speed_loop_elapsed_us_fy = 0U;
    control_cycle_counter_valid_fy = 0U;
    fy_soft_limit_blocked = 0U;
    fy_soft_limit_direction = 0;
  } else {
    PID_Reset(&position_pid_inst_sp);
    PID_Reset(&speed_pid_inst_sp);
    PID_Reset(&id_pid_inst_sp);
    PID_Reset(&iq_pid_inst_sp);
    speed_given_sp = 0.0f;
    id_given_sp = 0.0f;
    iq_given_sp = 0.0f;
    u_d_sp = 0.0f;
    u_q_sp = 0.0f;
    foc_control_mode_sp = FOC_CONTROL_MODE_POSITION;
    speed_loop_count_sp = 0U;
    ssi_request_count_sp = 0U;
    speed_loop_elapsed_us_sp = 0U;
    control_cycle_counter_valid_sp = 0U;
    Experiment_Control_FastDisable();
  }
}

static void set_pwm_neutral(FOC_Axis axis)
{
  TIM_HandleTypeDef *timer = (axis == FOC_AXIS_FY) ? &htim8 : &htim1;
  uint32_t *ccr_a = (axis == FOC_AXIS_FY) ? &ccrA_fy : &ccrA_sp;
  uint32_t *ccr_b = (axis == FOC_AXIS_FY) ? &ccrB_fy : &ccrB_sp;
  uint32_t *ccr_c = (axis == FOC_AXIS_FY) ? &ccrC_fy : &ccrC_sp;
  const uint32_t neutral = timer->Init.Period / 2U;

  *ccr_a = neutral;
  *ccr_b = neutral;
  *ccr_c = neutral;
  __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_1, neutral);
  __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_2, neutral);
  __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_3, neutral);
}

static void stop_pwm(FOC_Axis axis)
{
  TIM_HandleTypeDef *timer = (axis == FOC_AXIS_FY) ? &htim8 : &htim1;
  (void)HAL_TIM_PWM_Stop(timer, TIM_CHANNEL_1);
  (void)HAL_TIMEx_PWMN_Stop(timer, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Stop(timer, TIM_CHANNEL_2);
  (void)HAL_TIMEx_PWMN_Stop(timer, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Stop(timer, TIM_CHANNEL_3);
  (void)HAL_TIMEx_PWMN_Stop(timer, TIM_CHANNEL_3);
}

static void stop_power_stage(FOC_Axis axis, FOC_FaultCode fault_code)
{
  volatile FOC_PowerState *power_state = (axis == FOC_AXIS_FY)
                                               ? &foc_power_state_fy
                                               : &foc_power_state_sp;
  volatile FOC_FaultCode *fault = (axis == FOC_AXIS_FY)
                                      ? &foc_fault_code_fy
                                      : &foc_fault_code_sp;
  volatile uint8_t *target_valid = (axis == FOC_AXIS_FY)
                                       ? &foc_position_target_valid_fy
                                       : &foc_position_target_valid_sp;
  volatile uint8_t *calibration_active = (axis == FOC_AXIS_FY)
                                             ? &electrical_calibration_active_fy
                                             : &electrical_calibration_active_sp;
  float *calibration_ud = (axis == FOC_AXIS_FY)
                              ? &electrical_calibration_ud_fy
                              : &electrical_calibration_ud_sp;
  float *current_angle = (axis == FOC_AXIS_FY)
                             ? &current_angle_fy
                             : &current_angle_sp;
  float *current_speed = (axis == FOC_AXIS_FY)
                             ? &current_speed_fy
                             : &current_speed_sp;
  float *position_given = (axis == FOC_AXIS_FY)
                              ? &position_given_fy
                              : &position_given_sp;

  *power_state = FOC_POWER_DISABLED;
  *fault = fault_code;
  *calibration_active = 0U;
  *calibration_ud = 0.0f;
  reset_control_state(axis);
  *current_speed = 0.0f;
  *position_given = *current_angle;
  *target_valid = ssi_frame_is_fresh(axis) ? 1U : 0U;
  encoder_speed_reset_axis(axis, *current_angle);
  set_pwm_neutral(axis);
  stop_pwm(axis);
  adc_reset_axis(axis);
}

void FOC_SetPositionTarget(FOC_Axis axis, float target_deg)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();

  if (axis == FOC_AXIS_FY) {
    if (foc_control_mode_fy != FOC_CONTROL_MODE_POSITION) {
      PID_Reset(&position_pid_inst_fy);
      PID_Reset(&speed_pid_inst_fy);
    }
    electrical_calibration_active_fy = 0U;
    position_given_fy = clamp_fy_target_deg(target_deg);
    foc_control_mode_fy = FOC_CONTROL_MODE_POSITION;
    foc_position_target_valid_fy = 1U;
  } else {
    if (foc_control_mode_sp != FOC_CONTROL_MODE_POSITION) {
      PID_Reset(&position_pid_inst_sp);
      PID_Reset(&speed_pid_inst_sp);
    }
    electrical_calibration_active_sp = 0U;
    position_given_sp = target_deg;
    foc_control_mode_sp = FOC_CONTROL_MODE_POSITION;
    foc_position_target_valid_sp = 1U;
  }

  restore_interrupt_state(primask);
}

void FOC_SetSpeedTarget(FOC_Axis axis, float target_rpm)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();

  if (axis == FOC_AXIS_FY) {
    if (foc_control_mode_fy != FOC_CONTROL_MODE_SPEED) {
      PID_Reset(&speed_pid_inst_fy);
    }
    electrical_calibration_active_fy = 0U;
    speed_given_fy = target_rpm;
    foc_control_mode_fy = FOC_CONTROL_MODE_SPEED;
    foc_position_target_valid_fy = 0U;
  } else {
    if (foc_control_mode_sp != FOC_CONTROL_MODE_SPEED) {
      PID_Reset(&speed_pid_inst_sp);
    }
    electrical_calibration_active_sp = 0U;
    speed_given_sp = target_rpm;
    foc_control_mode_sp = FOC_CONTROL_MODE_SPEED;
    foc_position_target_valid_sp = 0U;
  }

  restore_interrupt_state(primask);
}

void Control_Loop(FOC_Axis axis)
{
  const uint8_t is_fy = (axis == FOC_AXIS_FY) ? 1U : 0U;
  TIM_HandleTypeDef *timer = is_fy ? &htim8 : &htim1;
  volatile FOC_ControlMode *control_mode = is_fy
                                               ? &foc_control_mode_fy
                                               : &foc_control_mode_sp;
  volatile FOC_PowerState *power_state = is_fy
                                             ? &foc_power_state_fy
                                             : &foc_power_state_sp;
  volatile uint8_t *target_valid = is_fy
                                       ? &foc_position_target_valid_fy
                                       : &foc_position_target_valid_sp;
  volatile uint8_t *calibration_active = is_fy
                                             ? &electrical_calibration_active_fy
                                             : &electrical_calibration_active_sp;
  uint32_t *enable_sample_count = is_fy
                                      ? &enable_sample_count_fy
                                      : &enable_sample_count_sp;
  uint32_t *enable_start_tick_ms = is_fy
                                       ? &enable_start_tick_ms_fy
                                       : &enable_start_tick_ms_sp;
  uint32_t *control_time_us = is_fy
                                  ? &control_time_us_fy
                                  : &control_time_us_sp;
  uint16_t *speed_loop_count = is_fy
                                   ? &speed_loop_count_fy
                                   : &speed_loop_count_sp;
  uint8_t *ssi_request_count = is_fy
                                   ? &ssi_request_count_fy
                                   : &ssi_request_count_sp;
  uint32_t *speed_loop_elapsed_us = is_fy
                                        ? &speed_loop_elapsed_us_fy
                                        : &speed_loop_elapsed_us_sp;
  PID_TypeDef *position_pid = is_fy
                                  ? &position_pid_inst_fy
                                  : &position_pid_inst_sp;
  PID_TypeDef *speed_pid = is_fy ? &speed_pid_inst_fy : &speed_pid_inst_sp;
  PID_TypeDef *id_pid = is_fy ? &id_pid_inst_fy : &id_pid_inst_sp;
  PID_TypeDef *iq_pid = is_fy ? &iq_pid_inst_fy : &iq_pid_inst_sp;
  float *adc_current = is_fy ? g_adc_current_fy : g_adc_current_sp;
  float *adc_vbus = is_fy ? &g_adc_vbus_fy : &g_adc_vbus_sp;
  float *current_angle = is_fy ? &current_angle_fy : &current_angle_sp;
  float *current_speed = is_fy ? &current_speed_fy : &current_speed_sp;
  float *position_given = is_fy ? &position_given_fy : &position_given_sp;
  float *speed_given = is_fy ? &speed_given_fy : &speed_given_sp;
  float *id_given = is_fy ? &id_given_fy : &id_given_sp;
  float *iq_given = is_fy ? &iq_given_fy : &iq_given_sp;
  float *calibration_ud = is_fy
                              ? &electrical_calibration_ud_fy
                              : &electrical_calibration_ud_sp;
  float *electrical_offset = is_fy
                                 ? &electrical_offset_deg_fy
                                 : &electrical_offset_deg_sp;
  float *theta = is_fy ? &theta_fy : &theta_sp;
  float *i_alpha = is_fy ? &i_alpha_fy : &i_alpha_sp;
  float *i_beta = is_fy ? &i_beta_fy : &i_beta_sp;
  float *i_d = is_fy ? &i_d_fy : &i_d_sp;
  float *i_q = is_fy ? &i_q_fy : &i_q_sp;
  float *u_d = is_fy ? &u_d_fy : &u_d_sp;
  float *u_q = is_fy ? &u_q_fy : &u_q_sp;
  float *u_alpha = is_fy ? &u_alpha_fy : &u_alpha_sp;
  float *u_beta = is_fy ? &u_beta_fy : &u_beta_sp;
  uint32_t *ccr_a = is_fy ? &ccrA_fy : &ccrA_sp;
  uint32_t *ccr_b = is_fy ? &ccrB_fy : &ccrB_sp;
  uint32_t *ccr_c = is_fy ? &ccrC_fy : &ccrC_sp;

  const uint32_t elapsed_control_us = measure_control_elapsed_us(axis);
  *control_time_us += elapsed_control_us;
  const uint32_t current_time = *control_time_us;

  (*ssi_request_count)++;
  if (*ssi_request_count >= FOC_SPEED_LOOP_DIVIDER) {
    *ssi_request_count = 0U;
    ssi_process_axis(axis);
  }
  Get_Electrical_Angle(theta, current_angle, *electrical_offset);

  if (*power_state == FOC_POWER_WAIT_ENCODER) {
    if (((uint32_t)(ssi_valid_sample_count(axis) - *enable_sample_count) >=
         FOC_ENABLE_MIN_FRESH_SAMPLES) &&
        (ssi_frame_is_fresh(axis) != 0U)) {
      *position_given = *current_angle;
      *current_speed = 0.0f;
      encoder_speed_reset_axis(axis, *current_angle);
      reset_control_state(axis);
      set_pwm_neutral(axis);
      *target_valid = (*calibration_active != 0U) ? 0U : 1U;
      *power_state = FOC_POWER_ENABLED;
    } else if ((uint32_t)(HAL_GetTick() - *enable_start_tick_ms) >=
               SSI_ENABLE_ACQUIRE_TIMEOUT_MS) {
      stop_power_stage(axis, FOC_FAULT_ENCODER_START_TIMEOUT);
    }
    return;
  }

  if (*power_state != FOC_POWER_ENABLED) {
    return;
  }
  if (ssi_frame_is_fresh(axis) == 0U) {
    stop_power_stage(axis, FOC_FAULT_ENCODER_RUNTIME_TIMEOUT);
    return;
  }

  (*speed_loop_count)++;
  *speed_loop_elapsed_us += elapsed_control_us;
  if (*speed_loop_count >= FOC_SPEED_LOOP_DIVIDER) {
    *speed_loop_count = 0U;
    const float speed_period_s =
        (float)(*speed_loop_elapsed_us) / 1000000.0f;
    *speed_loop_elapsed_us = 0U;
    encoder_speed_update_axis(axis, current_speed, current_angle,
                              speed_period_s);

    if (*calibration_active == 0U) {
      if (*control_mode == FOC_CONTROL_MODE_POSITION) {
        const float position_error = is_fy
                                         ? fy_position_error_deg(
                                               *position_given,
                                               *current_angle)
                                         : shortest_angle_error_deg(
                                               *position_given,
                                               *current_angle);
        *speed_given = PID_Update(position_pid, position_error, current_time);
      }
      float limited_speed_given = *speed_given;
      if (is_fy &&
          (fy_speed_is_outward(limited_speed_given, *current_angle) != 0U)) {
        limited_speed_given = 0.0f;
        if (fy_soft_limit_blocked == 0U) {
          PID_Reset(speed_pid);
          fy_soft_limit_blocked = 1U;
        }
      } else if (is_fy) {
        fy_soft_limit_blocked = 0U;
      }
      const float feedback_iq = PID_Update(
          speed_pid, limited_speed_given - *current_speed, current_time);
      *iq_given = is_fy
                      ? feedback_iq
                      : Experiment_Control_Update(*current_angle,
                                                  *current_speed,
                                                  feedback_iq);
    }
  }

  const float i_a = -adc_current[0];
  const float i_b = -adc_current[1];
  const float i_c = -adc_current[2];
  clarke_transform(i_a, i_b, i_c, i_alpha, i_beta);
  const float theta_sin = sinf(*theta);
  const float theta_cos = cosf(*theta);
  *i_d = *i_alpha * theta_cos + *i_beta * theta_sin;
  *i_q = -*i_alpha * theta_sin + *i_beta * theta_cos;

  if (*calibration_active != 0U) {
    /* 标定时固定电角度0°、Ud为给定值、Uq为0，不运行电流PI。 */
    *u_d = *calibration_ud;
    *u_q = 0.0f;
  } else {
    *u_d = PID_Update(id_pid, *id_given - *i_d, current_time);
    *u_q = PID_Update(iq_pid, *iq_given - *i_q, current_time);
  }

  float voltage_limit = *adc_vbus * EXPERIMENT_VOLTAGE_UTILIZATION;
  if (voltage_limit > VOLTAGE_LIMIT) {
    voltage_limit = VOLTAGE_LIMIT;
  }
  const float voltage_magnitude_sq = *u_d * *u_d + *u_q * *u_q;
  if ((voltage_limit > 0.0f) &&
      (voltage_magnitude_sq > voltage_limit * voltage_limit)) {
    const float scale = voltage_limit / sqrtf(voltage_magnitude_sq);
    *u_d *= scale;
    *u_q *= scale;
  }

  if (*calibration_active != 0U) {
    *u_alpha = *u_d;
    *u_beta = *u_q;
  } else {
    *u_alpha = *u_d * theta_cos - *u_q * theta_sin;
    *u_beta = *u_d * theta_sin + *u_q * theta_cos;
  }
  svpwm_generate(*u_alpha, *u_beta, *adc_vbus, ccr_a, ccr_b, ccr_c);
  __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_1, *ccr_a);
  __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_2, *ccr_b);
  __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_3, *ccr_c);
}

void Control_Loop_test(FOC_Axis axis)
{
  const uint8_t is_fy = (axis == FOC_AXIS_FY) ? 1U : 0U;
  TIM_HandleTypeDef *timer = is_fy ? &htim8 : &htim1;
  float *adc_current = is_fy ? g_adc_current_fy : g_adc_current_sp;
  float *adc_vbus = is_fy ? &g_adc_vbus_fy : &g_adc_vbus_sp;
  float *current_angle = is_fy ? &current_angle_fy : &current_angle_sp;
  float *electrical_offset = is_fy
                                 ? &electrical_offset_deg_fy
                                 : &electrical_offset_deg_sp;
  float *theta = is_fy ? &theta_fy : &theta_sp;
  float *i_alpha = is_fy ? &i_alpha_fy : &i_alpha_sp;
  float *i_beta = is_fy ? &i_beta_fy : &i_beta_sp;
  float *i_d = is_fy ? &i_d_fy : &i_d_sp;
  float *i_q = is_fy ? &i_q_fy : &i_q_sp;
  float *u_alpha = is_fy ? &u_alpha_fy : &u_alpha_sp;
  float *u_beta = is_fy ? &u_beta_fy : &u_beta_sp;
  float *open_loop_theta = is_fy
                               ? &open_loop_theta_fy
                               : &open_loop_theta_sp;
  float *open_loop_speed = is_fy
                               ? &open_loop_speed_fy
                               : &open_loop_speed_sp;
  float *open_loop_voltage = is_fy
                                 ? &open_loop_voltage_fy
                                 : &open_loop_voltage_sp;
  uint32_t *ccr_a = is_fy ? &ccrA_fy : &ccrA_sp;
  uint32_t *ccr_b = is_fy ? &ccrB_fy : &ccrB_sp;
  uint32_t *ccr_c = is_fy ? &ccrC_fy : &ccrC_sp;

  ssi_process_axis(axis);
  Get_Electrical_Angle(theta, current_angle, *electrical_offset);
  *open_loop_theta += *open_loop_speed * FOC_CURRENT_LOOP_PERIOD_S;
  if (*open_loop_theta > 2.0f * PI) {
    *open_loop_theta -= 2.0f * PI;
  }
  if (*open_loop_theta < 0.0f) {
    *open_loop_theta += 2.0f * PI;
  }

  clarke_transform(-adc_current[0], -adc_current[1], -adc_current[2],
                   i_alpha, i_beta);
  park_transform(*i_alpha, *i_beta, *theta, i_d, i_q);
  ipark_transform(*open_loop_voltage, 0.0f, *open_loop_theta,
                  u_alpha, u_beta);
  svpwm_generate(*u_alpha, *u_beta, *adc_vbus, ccr_a, ccr_b, ccr_c);
  __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_1, *ccr_a);
  __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_2, *ccr_b);
  __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_3, *ccr_c);
}

static uint8_t start_power_stage(FOC_Axis axis)
{
  volatile FOC_PowerState *power_state = (axis == FOC_AXIS_FY)
                                               ? &foc_power_state_fy
                                               : &foc_power_state_sp;
  volatile FOC_FaultCode *fault = (axis == FOC_AXIS_FY)
                                      ? &foc_fault_code_fy
                                      : &foc_fault_code_sp;
  volatile uint8_t *target_valid = (axis == FOC_AXIS_FY)
                                       ? &foc_position_target_valid_fy
                                       : &foc_position_target_valid_sp;
  uint32_t *enable_sample_count = (axis == FOC_AXIS_FY)
                                      ? &enable_sample_count_fy
                                      : &enable_sample_count_sp;
  uint32_t *enable_start_tick_ms = (axis == FOC_AXIS_FY)
                                       ? &enable_start_tick_ms_fy
                                       : &enable_start_tick_ms_sp;
  TIM_HandleTypeDef *timer = (axis == FOC_AXIS_FY) ? &htim8 : &htim1;
  const uint8_t adc_calibrated = (axis == FOC_AXIS_FY)
                                     ? g_adc_calibrated_fy
                                     : g_adc_calibrated_sp;

  if ((*power_state != FOC_POWER_DISABLED) || (adc_calibrated == 0U)) {
    return 0U;
  }

  reset_control_state(axis);
  adc_reset_axis(axis);
  set_pwm_neutral(axis);
  *target_valid = 0U;
  *fault = FOC_FAULT_NONE;
  ssi_rearm_axis(axis);
  *enable_sample_count = ssi_valid_sample_count(axis);
  *enable_start_tick_ms = HAL_GetTick();
  *power_state = FOC_POWER_WAIT_ENCODER;

  /* 第二轴启动时只错开载波，不增加额外的轴间互锁。 */
  if (((axis == FOC_AXIS_SP) &&
       (foc_power_state_fy != FOC_POWER_DISABLED)) ||
      ((axis == FOC_AXIS_FY) &&
       (foc_power_state_sp != FOC_POWER_DISABLED))) {
    __HAL_TIM_SET_COUNTER(timer, timer->Init.Period / 2U);
  } else {
    __HAL_TIM_SET_COUNTER(timer, 0U);
  }

  HAL_StatusTypeDef pwm_status = HAL_OK;
  if (HAL_TIM_PWM_Start(timer, TIM_CHANNEL_1) != HAL_OK) pwm_status = HAL_ERROR;
  if (HAL_TIMEx_PWMN_Start(timer, TIM_CHANNEL_1) != HAL_OK) pwm_status = HAL_ERROR;
  if (HAL_TIM_PWM_Start(timer, TIM_CHANNEL_2) != HAL_OK) pwm_status = HAL_ERROR;
  if (HAL_TIMEx_PWMN_Start(timer, TIM_CHANNEL_2) != HAL_OK) pwm_status = HAL_ERROR;
  if (HAL_TIM_PWM_Start(timer, TIM_CHANNEL_3) != HAL_OK) pwm_status = HAL_ERROR;
  if (HAL_TIMEx_PWMN_Start(timer, TIM_CHANNEL_3) != HAL_OK) pwm_status = HAL_ERROR;

  if (pwm_status != HAL_OK) {
    stop_power_stage(axis, FOC_FAULT_PWM_START_FAILED);
    return 0U;
  }
  return 1U;
}

void Motor_Enable(FOC_Axis axis)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (axis == FOC_AXIS_FY) {
    electrical_calibration_active_fy = 0U;
    electrical_calibration_ud_fy = 0.0f;
  } else {
    electrical_calibration_active_sp = 0U;
    electrical_calibration_ud_sp = 0.0f;
  }
  (void)start_power_stage(axis);
  restore_interrupt_state(primask);
}

void Motor_Disable(FOC_Axis axis)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  stop_power_stage(axis, FOC_FAULT_NONE);
  restore_interrupt_state(primask);
}

uint8_t FOC_StartElectricalCalibration(FOC_Axis axis, float ud_voltage_v)
{
  if (!isfinite(ud_voltage_v) || (ud_voltage_v <= 0.0f)) {
    return 0U;
  }

  Motor_Disable(axis);
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (axis == FOC_AXIS_FY) {
    electrical_calibration_active_fy = 1U;
    electrical_calibration_ud_fy = ud_voltage_v;
  } else {
    electrical_calibration_active_sp = 1U;
    electrical_calibration_ud_sp = ud_voltage_v;
  }
  const uint8_t started = start_power_stage(axis);
  if (started == 0U) {
    if (axis == FOC_AXIS_FY) {
      electrical_calibration_active_fy = 0U;
      electrical_calibration_ud_fy = 0.0f;
    } else {
      electrical_calibration_active_sp = 0U;
      electrical_calibration_ud_sp = 0.0f;
    }
  }
  restore_interrupt_state(primask);
  return started;
}

void FOC_StopElectricalCalibration(FOC_Axis axis)
{
  Motor_Disable(axis);
}

uint8_t FOC_SetElectricalOffset(FOC_Axis axis, float offset_deg)
{
  if (!isfinite(offset_deg)) {
    return 0U;
  }
  float offset = fmodf(offset_deg, 360.0f);
  if (offset < 0.0f) {
    offset += 360.0f;
  }
  if (axis == FOC_AXIS_FY) {
    electrical_offset_deg_fy = offset;
  } else {
    electrical_offset_deg_sp = offset;
  }
  return 1U;
}

uint8_t FOC_CaptureElectricalOffset(FOC_Axis axis)
{
  const float offset = (axis == FOC_AXIS_FY)
                           ? current_angle_fy
                           : current_angle_sp;
  Motor_Disable(axis);
  return FOC_SetElectricalOffset(axis, offset);
}

FOC_ControlMode FOC_GetControlMode(FOC_Axis axis)
{
  return (axis == FOC_AXIS_FY) ? foc_control_mode_fy : foc_control_mode_sp;
}

float FOC_GetSpeedTarget(FOC_Axis axis)
{
  return (axis == FOC_AXIS_FY) ? speed_given_fy : speed_given_sp;
}

float FOC_GetIdTarget(FOC_Axis axis)
{
  return (axis == FOC_AXIS_FY) ? id_given_fy : id_given_sp;
}

float FOC_GetIqTarget(FOC_Axis axis)
{
  return (axis == FOC_AXIS_FY) ? iq_given_fy : iq_given_sp;
}

float FOC_GetElectricalOffset(FOC_Axis axis)
{
  return (axis == FOC_AXIS_FY)
             ? electrical_offset_deg_fy
             : electrical_offset_deg_sp;
}

float FOC_GetElectricalCalibrationUd(FOC_Axis axis)
{
  return (axis == FOC_AXIS_FY)
             ? electrical_calibration_ud_fy
             : electrical_calibration_ud_sp;
}

uint8_t FOC_IsElectricalCalibrationActive(FOC_Axis axis)
{
  return (axis == FOC_AXIS_FY)
             ? electrical_calibration_active_fy
             : electrical_calibration_active_sp;
}

uint8_t FOC_IsPositionTargetValid(FOC_Axis axis)
{
  if (axis == FOC_AXIS_FY) {
    return (foc_position_target_valid_fy &&
            (foc_control_mode_fy == FOC_CONTROL_MODE_POSITION)) ? 1U : 0U;
  }
  return (foc_position_target_valid_sp &&
          (foc_control_mode_sp == FOC_CONTROL_MODE_POSITION)) ? 1U : 0U;
}

uint8_t FOC_GetPowerState(FOC_Axis axis)
{
  return (uint8_t)((axis == FOC_AXIS_FY)
                       ? foc_power_state_fy
                       : foc_power_state_sp);
}

FOC_FaultCode FOC_GetFaultCode(FOC_Axis axis)
{
  return (axis == FOC_AXIS_FY) ? foc_fault_code_fy : foc_fault_code_sp;
}
