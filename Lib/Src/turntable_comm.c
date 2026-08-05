#include "turntable_comm.h"

#include "FOC_Control.h"
#include "PID_Control.h"
#include "Experiment_Config.h"
#include "Experiment_Control.h"
#include "Experiment_ESO.h"
#include "Experiment_LearningFeedforward.h"
#include "ssi.h"
#include "usart.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define TT_HEAD0 0xA5U
#define TT_HEAD1 0x5AU
#define TT_TAIL0 0x0DU
#define TT_TAIL1 0x0AU

#define TT_RX_MAX_FRAME_SIZE 32U
#define TT_RX_DMA_BUFFER_SIZE 64U
#define TT_RX_QUEUE_DEPTH 8U
#define TT_STATUS_PERIOD_MS 50U

#define TT_MODE_DISABLE 0x00U
#define TT_MODE_ENABLE  0x01U
#define TT_MODE_SERVO   0x02U
#define TT_MODE_TRACK   0x03U
#define TT_MODE_EXT     0x05U

#define TT_FUNC_PID     0x00U
#define TT_FUNC_ANGLE   0x01U
#define TT_FUNC_SPEED   0x02U

#define TT_EXT_PID_QUERY  0x10U
#define TT_EXT_PID_REPORT 0x11U
#define TT_EXT_TELEMETRY_REPORT 0x12U
#define TT_EXT_TARGET_QUERY  0x13U
#define TT_EXT_TARGET_REPORT 0x14U
#define TT_EXT_EXPERIMENT_CONFIG 0x15U
#define TT_EXT_EXPERIMENT_REPORT 0x16U
#define TT_EXT_SPEED_REPORT 0x17U
#define TT_EXT_HEALTH_REPORT 0x18U
#define TT_EXT_ADC_DIAGNOSTIC_REPORT 0x19U
#define TT_EXT_SWEEP_RESULT_REPORT 0x1AU
#define TT_EXT_ESO_CONFIG 0x1BU
#define TT_EXT_ESO_REPORT 0x1CU
#define TT_EXT_LFF_REPORT 0x1DU
#define TT_EXT_LFF_ORDER_REPORT 0x1EU
#define TT_EXT_LFF_TABLE_REPORT 0x1FU

#define TT_AXIS_AZ      0x00U
#define TT_UART_HANDLE  huart1
#define TT_UART_INSTANCE USART1

extern float current_angle_sp;
/* FOC运行时只读测量量，不改变控制层和驱动层。 */
extern float current_speed_sp;
extern float position_given_sp;
extern float i_d;
extern float i_q;

static void send_telemetry_frame(void);
static void send_health_report(void);
static void send_adc_diagnostic_report(void);
static void send_speed_report(uint8_t accepted);
static void send_sweep_result_report(uint8_t order);
static void send_eso_report(void);
static void send_lff_report(void);
static void send_lff_order_report(uint8_t order);
static void send_lff_table_report(uint16_t start_bin);

typedef enum {
  TT_RX_WAIT_HEAD0 = 0,
  TT_RX_WAIT_HEAD1,
  TT_RX_PAYLOAD
} TT_RxState;

typedef struct {
  uint8_t visible_light;
  uint8_t infrared;
  uint8_t laser;
} TT_FeatureState;

static uint8_t tt_rx_byte;
static uint8_t tt_rx_dma_buf[TT_RX_DMA_BUFFER_SIZE];
static volatile uint8_t tt_rx_uses_dma;
static volatile TT_RxState tt_rx_state;
static volatile uint8_t tt_rx_len;
static uint8_t tt_rx_frame[TT_RX_MAX_FRAME_SIZE];

static volatile uint8_t tt_rx_queue_head;
static volatile uint8_t tt_rx_queue_tail;
static volatile uint8_t tt_rx_queue_count;
static uint8_t tt_rx_queue_len[TT_RX_QUEUE_DEPTH];
static uint8_t tt_rx_queue[TT_RX_QUEUE_DEPTH][TT_RX_MAX_FRAME_SIZE];

static volatile uint8_t tt_tx_busy;
static uint8_t tt_tx_buf[128];

volatile uint32_t tt_status_tx_count;
volatile uint32_t tt_status_tx_error_count;
volatile uint32_t tt_rx_dropped_frame_count;
volatile uint32_t tt_rx_uart_error_count;
volatile uint32_t tt_rx_valid_frame_count;
volatile uint32_t tt_rx_invalid_frame_count;

static TT_FeatureState tt_servo_features;
static TT_FeatureState tt_tracking_features;
static volatile uint8_t tt_eso_config_accepted = 1U;

uint8_t Turntable_Comm_IsEnabled(void)
{
  return TURNTABLE_COMM_ENABLE ? 1U : 0U;
}

static uint8_t checksum_sum(const uint8_t *data, uint16_t start, uint16_t end_exclusive)
{
  uint16_t sum = 0U;
  for (uint16_t i = start; i < end_exclusive; i++) {
    sum = (uint16_t)(sum + data[i]);
  }
  return (uint8_t)(sum & 0xFFU);
}

static uint8_t frame_is_valid(const uint8_t *frame, uint8_t len)
{
  if (len < 7U) {
    return 0U;
  }
  if (frame[0] != TT_HEAD0 || frame[1] != TT_HEAD1) {
    return 0U;
  }
  if (frame[len - 2U] != TT_TAIL0 || frame[len - 1U] != TT_TAIL1) {
    return 0U;
  }

  uint8_t expected = checksum_sum(frame, 2U, (uint16_t)(len - 3U));
  return expected == frame[len - 3U];
}

static float read_le_float(const uint8_t *p)
{
  float value;
  uint8_t bytes[4] = {p[0], p[1], p[2], p[3]};
  memcpy(&value, bytes, sizeof(value));
  return value;
}

static void write_le_float(uint8_t *p, float value)
{
  memcpy(p, &value, sizeof(value));
}

static uint16_t read_le_u16(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t read_le_u32(const uint8_t *p)
{
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static void write_le_u16(uint8_t *p, uint16_t value)
{
  p[0] = (uint8_t)(value & 0xFFU);
  p[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void write_le_u32(uint8_t *p, uint32_t value)
{
  p[0] = (uint8_t)(value & 0xFFU);
  p[1] = (uint8_t)((value >> 8) & 0xFFU);
  p[2] = (uint8_t)((value >> 16) & 0xFFU);
  p[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static PID_TypeDef *pid_from_loop(uint8_t loop)
{
  switch (loop) {
    case 0x00U:
      return &iq_pid_inst;
    case 0x03U:
      return &id_pid_inst;
    case 0x01U:
      return &speed_pid_inst;
    case 0x02U:
      return &position_pid_inst;
    default:
      return NULL;
  }
}

static void apply_axis_angle(uint8_t axis, float target_deg)
{
  if ((axis == TT_AXIS_AZ) && isfinite(target_deg)) {
    FOC_SetPositionTarget(target_deg);
  }
}

static void apply_pid(uint8_t axis, uint8_t loop, float kp, float ki, float kd)
{
  if ((axis != TT_AXIS_AZ) ||
      !isfinite(kp) || !isfinite(ki) || !isfinite(kd)) {
    return;
  }
  PID_TypeDef *pid = pid_from_loop(loop);
  if (pid == NULL) {
    return;
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;
  PID_Reset(pid);
  if ((primask & 1U) == 0U) {
    __enable_irq();
  }
}

static void apply_feature(TT_FeatureState *features, uint8_t feature_code, uint8_t enabled)
{
  uint8_t value = enabled ? 1U : 0U;

  switch (feature_code) {
    case 0x00U:
    case 0x03U:
      features->visible_light = value;
      break;
    case 0x01U:
    case 0x04U:
      features->infrared = value;
      break;
    case 0x02U:
    case 0x05U:
      features->laser = value;
      break;
    default:
      break;
  }
}

static void handle_servo_frame(const uint8_t *frame, uint8_t len)
{
  if (len < 8U) {
    return;
  }

  uint8_t func = frame[3];
  if (func == TT_FUNC_PID && len == 21U) {
    uint8_t axis = frame[4];
    uint8_t loop = frame[5];
    float kp = read_le_float(&frame[6]);
    float ki = read_le_float(&frame[10]);
    float kd = read_le_float(&frame[14]);
    apply_pid(axis, loop, kp, ki, kd);
  } else if (func == TT_FUNC_ANGLE) {
    if (len == 15U) {
      float az_target = read_le_float(&frame[4]);
      apply_axis_angle(TT_AXIS_AZ, az_target);
    } else if (len == 12U) {
      uint8_t axis = frame[4];
      float target = read_le_float(&frame[5]);
      apply_axis_angle(axis, target);
    }
  } else if (func == TT_FUNC_SPEED) {
    if (len == 12U && frame[4] == TT_AXIS_AZ) {
      float requested_rpm = read_le_float(&frame[5]);
      uint8_t accepted = 0U;

      if (isfinite(requested_rpm)) {
        FOC_SetSpeedTarget(requested_rpm);
        accepted = 1U;
      }
      send_speed_report(accepted);
    }
  } else if ((func >= 0x03U && func <= 0x05U) && len == 8U) {
    apply_feature(&tt_servo_features, func, frame[4]);
  }
}

static void handle_tracking_frame(const uint8_t *frame, uint8_t len)
{
  if (len < 8U) {
    return;
  }

  uint8_t func = frame[3];
  if (func == 0x03U && len == 21U) {
    uint8_t axis = frame[4];
    uint8_t loop = frame[5];
    float kp = read_le_float(&frame[6]);
    float ki = read_le_float(&frame[10]);
    float kd = read_le_float(&frame[14]);
    apply_pid(axis, loop, kp, ki, kd);
  } else if (func <= 0x02U && len == 8U) {
    apply_feature(&tt_tracking_features, func, frame[4]);
  }
}

static void send_pid_report(uint8_t pid_mode, uint8_t axis, uint8_t loop)
{
  PID_TypeDef *pid = pid_from_loop(loop);
  if ((axis != TT_AXIS_AZ) || (pid == NULL) || tt_tx_busy) {
    return;
  }

  tt_tx_buf[0] = TT_HEAD0;
  tt_tx_buf[1] = TT_HEAD1;
  tt_tx_buf[2] = TT_MODE_EXT;
  tt_tx_buf[3] = TT_EXT_PID_REPORT;
  tt_tx_buf[4] = pid_mode;
  tt_tx_buf[5] = axis;
  tt_tx_buf[6] = loop;
  write_le_float(&tt_tx_buf[7], pid->Kp);
  write_le_float(&tt_tx_buf[11], pid->Ki);
  write_le_float(&tt_tx_buf[15], pid->Kd);
  tt_tx_buf[19] = checksum_sum(tt_tx_buf, 2U, 19U);
  tt_tx_buf[20] = TT_TAIL0;
  tt_tx_buf[21] = TT_TAIL1;

  tt_tx_busy = 1U;
  if (HAL_UART_Transmit(&TT_UART_HANDLE, tt_tx_buf, 22U, 5U) != HAL_OK) {
    tt_status_tx_error_count++;
  }
  tt_tx_busy = 0U;
}

/* 扩展目标回报帧，共16字节：
 * A5 5A 05 14 有效标志 方位目标 俯仰占位值 校验和 0D 0A。 */
static void send_target_report(void)
{
  if (tt_tx_busy) {
    return;
  }

  tt_tx_buf[0] = TT_HEAD0;
  tt_tx_buf[1] = TT_HEAD1;
  tt_tx_buf[2] = TT_MODE_EXT;
  tt_tx_buf[3] = TT_EXT_TARGET_REPORT;
  tt_tx_buf[4] = FOC_IsPositionTargetValid();
  write_le_float(&tt_tx_buf[5], position_given_sp);
  write_le_float(&tt_tx_buf[9], 0.0f);
  tt_tx_buf[13] = checksum_sum(tt_tx_buf, 2U, 13U);
  tt_tx_buf[14] = TT_TAIL0;
  tt_tx_buf[15] = TT_TAIL1;

  tt_tx_busy = 1U;
  if (HAL_UART_Transmit(&TT_UART_HANDLE, tt_tx_buf, 16U, 5U) != HAL_OK) {
    tt_status_tx_error_count++;
  }
  tt_tx_busy = 0U;
}

/*
 * 扩展速度确认帧，共23字节：
 *   A5 5A 05 17 接受标志 轴号 控制模式 功率状态
 *   速度目标 母线电压 q轴电流目标 校验和 0D 0A
 *
 * 收到结构正确的方位速度命令后立即返回本帧。帧内数据从控制层回读，
 * 上位机可据此区分“数据已写入串口”和“控制器已接受目标”，并判断暂未转动的原因。
 */
static void send_speed_report(uint8_t accepted)
{
  if (tt_tx_busy) {
    return;
  }

  tt_tx_buf[0] = TT_HEAD0;
  tt_tx_buf[1] = TT_HEAD1;
  tt_tx_buf[2] = TT_MODE_EXT;
  tt_tx_buf[3] = TT_EXT_SPEED_REPORT;
  tt_tx_buf[4] = accepted;
  tt_tx_buf[5] = TT_AXIS_AZ;
  tt_tx_buf[6] = (uint8_t)FOC_GetControlMode();
  tt_tx_buf[7] = FOC_GetPowerState();
  write_le_float(&tt_tx_buf[8], FOC_GetSpeedTarget());
  write_le_float(&tt_tx_buf[12], g_adc_vbus);
  write_le_float(&tt_tx_buf[16], FOC_GetIqTarget());
  tt_tx_buf[20] = checksum_sum(tt_tx_buf, 2U, 20U);
  tt_tx_buf[21] = TT_TAIL0;
  tt_tx_buf[22] = TT_TAIL1;

  tt_tx_busy = 1U;
  if (HAL_UART_Transmit(&TT_UART_HANDLE, tt_tx_buf, 23U, 5U) != HAL_OK) {
    tt_status_tx_error_count++;
  }
  tt_tx_busy = 0U;
}

/*
 * 实验状态回报帧，共33字节：
 *   A5 5A 05 16
 *   当前模式 请求模式 运行时模式开关
 *   学习状态 学习请求 学习模式
 *   扫频状态 当前阶次 完成标志 已完成阶次数
 *   反馈电流 固定前馈电流 学习前馈电流 ESO电流
 *   校验和 0D 0A
 */
static void send_experiment_report(void)
{
  if (tt_tx_busy) {
    return;
  }

  tt_tx_buf[0] = TT_HEAD0;
  tt_tx_buf[1] = TT_HEAD1;
  tt_tx_buf[2] = TT_MODE_EXT;
  tt_tx_buf[3] = TT_EXT_EXPERIMENT_REPORT;
  tt_tx_buf[4] = g_experiment_active_mode;
  tt_tx_buf[5] = g_experiment_mode_request;
  tt_tx_buf[6] = EXPERIMENT_RUNTIME_MODES_ENABLED ? 1U : 0U;
  tt_tx_buf[7] = g_lff_learning_active;
  tt_tx_buf[8] = g_learning_update_request;
  tt_tx_buf[9] = g_learning_mode_request;
  tt_tx_buf[10] = g_sweep_state;
  tt_tx_buf[11] = g_sweep_active_order;
  tt_tx_buf[12] = g_sweep_complete;
  tt_tx_buf[13] = g_sweep_completed_order_count;
  write_le_float(&tt_tx_buf[14], g_iq_feedback_a);
  write_le_float(&tt_tx_buf[18], g_iq_fixed_feedforward_a);
  write_le_float(&tt_tx_buf[22], g_iq_learning_feedforward_a);
  write_le_float(&tt_tx_buf[26], g_iq_eso_a);
  tt_tx_buf[30] = checksum_sum(tt_tx_buf, 2U, 30U);
  tt_tx_buf[31] = TT_TAIL0;
  tt_tx_buf[32] = TT_TAIL1;

  tt_tx_busy = 1U;
  if (HAL_UART_Transmit(&TT_UART_HANDLE, tt_tx_buf, 33U, 5U) != HAL_OK) {
    tt_status_tx_error_count++;
  }
  tt_tx_busy = 0U;
}

/*
 * 实验配置帧，新版20字节并兼容旧版16字节：
 *   A5 5A 05 15 模式 标志 学习模式 起始阶次 结束阶次
 *   注入幅值 随机种子 校验和 0D 0A
 * 模式6中注入幅值表示扫频电流；模式4/5中表示训练随机扰动幅值。
 *
 * 标志位：
 *   位0：使能学习
 *   位1：清空学习表
 *   位2：使能固定前馈扫频
 *   位3：清空扫频结果
 *   位4：允许训练随机扰动（仅学习正在运行时实际注入）
 *
 * 功能码为0x15的7字节帧表示查询实验状态。
 */
/*
 * 单阶扫频结果回报，共37字节：
 *   A5 5A 05 1A 阶次 有效标志
 *   基线余弦 基线正弦
 *   响应余弦 响应正弦 响应幅值
 *   建议前馈余弦电流 建议前馈正弦电流
 *   校验和 0D 0A
 *
 * 上位机发送A5 5A 05 1A 阶次 校验和 0D 0A逐阶读取。
 * 本接口只复制结果数组，不改变实验、控制或驱动状态。
 */
static void send_sweep_result_report(uint8_t order)
{
  if ((order < 1U) || (order > PAPER_LFF_MAX_ORDER) || tt_tx_busy) {
    return;
  }

  float baseline_cos_rpm;
  float baseline_sin_rpm;
  float response_cos_rpm_per_a;
  float response_sin_rpm_per_a;
  float response_magnitude_rpm_per_a;
  float recommended_cos_current_a;
  float recommended_sin_current_a;
  uint8_t result_valid;

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  baseline_cos_rpm = g_sweep_baseline_cos_rpm[order];
  baseline_sin_rpm = g_sweep_baseline_sin_rpm[order];
  response_cos_rpm_per_a = g_sweep_response_cos_rpm_per_a[order];
  response_sin_rpm_per_a = g_sweep_response_sin_rpm_per_a[order];
  response_magnitude_rpm_per_a =
      g_sweep_response_magnitude_rpm_per_a[order];
  recommended_cos_current_a = g_sweep_recommended_cos_current_a[order];
  recommended_sin_current_a = g_sweep_recommended_sin_current_a[order];
  result_valid = g_sweep_result_valid[order];
  if ((primask & 1U) == 0U) {
    __enable_irq();
  }

  tt_tx_buf[0] = TT_HEAD0;
  tt_tx_buf[1] = TT_HEAD1;
  tt_tx_buf[2] = TT_MODE_EXT;
  tt_tx_buf[3] = TT_EXT_SWEEP_RESULT_REPORT;
  tt_tx_buf[4] = order;
  tt_tx_buf[5] = result_valid;
  write_le_float(&tt_tx_buf[6], baseline_cos_rpm);
  write_le_float(&tt_tx_buf[10], baseline_sin_rpm);
  write_le_float(&tt_tx_buf[14], response_cos_rpm_per_a);
  write_le_float(&tt_tx_buf[18], response_sin_rpm_per_a);
  write_le_float(&tt_tx_buf[22], response_magnitude_rpm_per_a);
  write_le_float(&tt_tx_buf[26], recommended_cos_current_a);
  write_le_float(&tt_tx_buf[30], recommended_sin_current_a);
  tt_tx_buf[34] = checksum_sum(tt_tx_buf, 2U, 34U);
  tt_tx_buf[35] = TT_TAIL0;
  tt_tx_buf[36] = TT_TAIL1;

  tt_tx_busy = 1U;
  if (HAL_UART_Transmit(&TT_UART_HANDLE, tt_tx_buf, 37U, 5U) != HAL_OK) {
    tt_status_tx_error_count++;
  }
  tt_tx_busy = 0U;
}

static uint8_t experiment_mode_uses_eso(uint8_t mode)
{
  return ((mode == PHYSICAL_EXPERIMENT_ESO) ||
          (mode == PHYSICAL_EXPERIMENT_ESO_FIXED_FEEDFORWARD) ||
          (mode == PHYSICAL_EXPERIMENT_ESO_LEARNING_FEEDFORWARD) ||
          (mode == PHYSICAL_EXPERIMENT_FULL))
             ? 1U
             : 0U;
}

/*
 * ESO状态回报帧，共67字节：
 *   A5 5A 05 1C 当前模式 ESO启用 限幅标志 参数接受标志
 *   带宽 补偿增益 独立限幅 原始补偿 补偿输出 合成Iq
 *   速度PI 固定前馈 z1 z2 速度观测误差 剩余扰动转矩
 *   限幅更新次数 总更新次数 校验和 0D 0A
 */
static void send_eso_report(void)
{
  if (tt_tx_busy) {
    return;
  }

  Experiment_Eso_Config config;
  uint8_t active_mode;
  uint8_t saturated;
  uint8_t config_accepted;
  float iq_eso_raw_a;
  float iq_eso_a;
  float iq_composite_a;
  float iq_feedback_a;
  float iq_fixed_a;
  float z1_rad_s;
  float z2_rad_s2;
  float speed_error_rad_s;
  float residual_torque_nm;
  uint32_t saturation_count;
  uint32_t update_count;

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  config = Experiment_ESO_GetConfig();
  active_mode = g_experiment_active_mode;
  saturated = g_eso_saturated;
  config_accepted = tt_eso_config_accepted;
  iq_eso_raw_a = g_iq_eso_raw_a;
  iq_eso_a = g_iq_eso_a;
  iq_composite_a = g_iq_composite_a;
  iq_feedback_a = g_iq_feedback_a;
  iq_fixed_a = g_iq_fixed_feedforward_a;
  z1_rad_s = g_eso_z1_rad_s;
  z2_rad_s2 = g_eso_z2_rad_s2;
  speed_error_rad_s = g_eso_speed_error_rad_s;
  residual_torque_nm = g_residual_disturbance_nm;
  saturation_count = g_eso_saturation_count;
  update_count = g_eso_update_count;
  if ((primask & 1U) == 0U) {
    __enable_irq();
  }

  tt_tx_buf[0] = TT_HEAD0;
  tt_tx_buf[1] = TT_HEAD1;
  tt_tx_buf[2] = TT_MODE_EXT;
  tt_tx_buf[3] = TT_EXT_ESO_REPORT;
  tt_tx_buf[4] = active_mode;
  tt_tx_buf[5] = experiment_mode_uses_eso(active_mode);
  tt_tx_buf[6] = saturated;
  tt_tx_buf[7] = config_accepted;
  write_le_float(&tt_tx_buf[8], config.bandwidth_rad_s);
  write_le_float(&tt_tx_buf[12], config.compensation_gain);
  write_le_float(&tt_tx_buf[16], config.current_limit_a);
  write_le_float(&tt_tx_buf[20], iq_eso_raw_a);
  write_le_float(&tt_tx_buf[24], iq_eso_a);
  write_le_float(&tt_tx_buf[28], iq_composite_a);
  write_le_float(&tt_tx_buf[32], iq_feedback_a);
  write_le_float(&tt_tx_buf[36], iq_fixed_a);
  write_le_float(&tt_tx_buf[40], z1_rad_s);
  write_le_float(&tt_tx_buf[44], z2_rad_s2);
  write_le_float(&tt_tx_buf[48], speed_error_rad_s);
  write_le_float(&tt_tx_buf[52], residual_torque_nm);
  write_le_u32(&tt_tx_buf[56], saturation_count);
  write_le_u32(&tt_tx_buf[60], update_count);
  tt_tx_buf[64] = checksum_sum(tt_tx_buf, 2U, 64U);
  tt_tx_buf[65] = TT_TAIL0;
  tt_tx_buf[66] = TT_TAIL1;

  tt_tx_busy = 1U;
  if (HAL_UART_Transmit(&TT_UART_HANDLE, tt_tx_buf, 67U, 10U) != HAL_OK) {
    tt_status_tx_error_count++;
  }
  tt_tx_busy = 0U;
}

/*
 * 学习前馈状态回报帧，共80字节。前48字节保持旧字段顺序，新增：
 * 全局rho、直流/交流学习电流、交流表均值、实际训练扰动、配置幅值、
 * 随机种子，以及扰动使能/冻结标志。
 */
static void send_lff_report(void)
{
  if (tt_tx_busy) {
    return;
  }

  uint8_t active_mode;
  uint8_t learning_active;
  uint8_t learning_requested;
  uint8_t learning_mode;
  uint8_t active_table_index;
  uint8_t selected_order_count;
  uint16_t current_bin;
  uint32_t revolution_count;
  uint32_t covered_revolution_count;
  uint32_t update_count;
  uint32_t dropped_revolution_count;
  float last_coverage;
  float rho_mean;
  float table_rms_nm;
  float table_peak_abs_nm;
  float table_mean_nm;
  float dc_torque_nm;
  float global_rho;
  float iq_learning_a;
  float iq_learning_dc_a;
  float iq_learning_ac_a;
  float iq_training_noise_a;
  float training_noise_amplitude_a;
  uint32_t training_noise_seed;
  uint8_t state_flags;

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  active_mode = g_experiment_active_mode;
  learning_active = g_lff_learning_active;
  learning_requested = g_learning_update_request;
  learning_mode = g_learning_mode_request;
  active_table_index = g_lff_active_table_index;
  selected_order_count = g_lff_selected_order_count;
  current_bin = g_lff_current_bin;
  revolution_count = g_lff_revolution_count;
  covered_revolution_count = g_lff_covered_revolution_count;
  update_count = g_lff_update_count;
  dropped_revolution_count = g_lff_dropped_revolution_count;
  last_coverage = g_lff_last_coverage;
  rho_mean = g_lff_rho_mean;
  table_rms_nm = g_lff_table_rms_nm;
  table_peak_abs_nm = g_lff_table_peak_abs_nm;
  table_mean_nm = g_lff_table_mean_nm;
  dc_torque_nm = g_lff_dc_torque_nm;
  global_rho = g_lff_global_rho;
  iq_learning_a = g_iq_learning_feedforward_a;
  iq_learning_dc_a = g_iq_learning_dc_a;
  iq_learning_ac_a = g_iq_learning_ac_a;
  iq_training_noise_a = g_iq_training_noise_a;
  training_noise_amplitude_a = g_training_noise_amplitude_request_a;
  training_noise_seed = g_training_noise_seed_request;
  state_flags =
      (g_training_noise_enable_request != 0U ? 0x01U : 0x00U) |
      (((learning_requested == 0U) && (update_count > 0U))
           ? 0x02U
           : 0x00U);
  if ((primask & 1U) == 0U) {
    __enable_irq();
  }

  float table_rms_a = 0.0f;
  float table_peak_abs_a = 0.0f;
  float table_mean_a = 0.0f;
  float dc_current_a = iq_learning_dc_a;
  if (MOTOR_TORQUE_CONSTANT_NM_PER_A > 0.0f) {
    table_rms_a = table_rms_nm / MOTOR_TORQUE_CONSTANT_NM_PER_A;
    table_peak_abs_a =
        table_peak_abs_nm / MOTOR_TORQUE_CONSTANT_NM_PER_A;
    table_mean_a = table_mean_nm / MOTOR_TORQUE_CONSTANT_NM_PER_A;
    dc_current_a = dc_torque_nm / MOTOR_TORQUE_CONSTANT_NM_PER_A;
  }

  tt_tx_buf[0] = TT_HEAD0;
  tt_tx_buf[1] = TT_HEAD1;
  tt_tx_buf[2] = TT_MODE_EXT;
  tt_tx_buf[3] = TT_EXT_LFF_REPORT;
  tt_tx_buf[4] = active_mode;
  tt_tx_buf[5] = learning_active;
  tt_tx_buf[6] = learning_requested;
  tt_tx_buf[7] = learning_mode;
  tt_tx_buf[8] = active_table_index;
  tt_tx_buf[9] = selected_order_count;
  write_le_u16(&tt_tx_buf[10], current_bin);
  write_le_u32(&tt_tx_buf[12], revolution_count);
  write_le_u32(&tt_tx_buf[16], covered_revolution_count);
  write_le_u32(&tt_tx_buf[20], update_count);
  write_le_u32(&tt_tx_buf[24], dropped_revolution_count);
  write_le_float(&tt_tx_buf[28], last_coverage);
  write_le_float(&tt_tx_buf[32], rho_mean);
  write_le_float(&tt_tx_buf[36], table_rms_a);
  write_le_float(&tt_tx_buf[40], table_peak_abs_a);
  write_le_float(&tt_tx_buf[44], iq_learning_a);
  write_le_float(&tt_tx_buf[48], global_rho);
  write_le_float(&tt_tx_buf[52], dc_current_a);
  write_le_float(&tt_tx_buf[56], iq_learning_ac_a);
  write_le_float(&tt_tx_buf[60], table_mean_a);
  write_le_float(&tt_tx_buf[64], iq_training_noise_a);
  write_le_float(&tt_tx_buf[68], training_noise_amplitude_a);
  write_le_u32(&tt_tx_buf[72], training_noise_seed);
  tt_tx_buf[76] = state_flags;
  tt_tx_buf[77] = checksum_sum(tt_tx_buf, 2U, 77U);
  tt_tx_buf[78] = TT_TAIL0;
  tt_tx_buf[79] = TT_TAIL1;

  tt_tx_busy = 1U;
  if (HAL_UART_Transmit(&TT_UART_HANDLE, tt_tx_buf, 80U, 10U) != HAL_OK) {
    tt_status_tx_error_count++;
  }
  tt_tx_busy = 0U;
}

/* 单阶学习诊断回报，共35字节，所有傅里叶系数均换算为q轴电流。 */
static void send_lff_order_report(uint8_t order)
{
  if ((order > PAPER_LFF_MAX_ORDER) || tt_tx_busy) {
    return;
  }

  float rho;
  float residual_a_nm;
  float residual_b_nm;
  float learned_a_nm;
  float learned_b_nm;
  uint32_t update_count;
  uint8_t learning_mode;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  rho = g_lff_rho_orders[order];
  residual_a_nm = g_lff_last_residual_a_nm[order];
  residual_b_nm = g_lff_last_residual_b_nm[order];
  learned_a_nm = g_lff_learned_a_nm[order];
  learned_b_nm = g_lff_learned_b_nm[order];
  update_count = g_lff_update_count;
  learning_mode = g_learning_mode_request;
  if ((primask & 1U) == 0U) {
    __enable_irq();
  }

  const float torque_to_current =
      (MOTOR_TORQUE_CONSTANT_NM_PER_A > 0.0f)
          ? 1.0f / MOTOR_TORQUE_CONSTANT_NM_PER_A
          : 0.0f;
  tt_tx_buf[0] = TT_HEAD0;
  tt_tx_buf[1] = TT_HEAD1;
  tt_tx_buf[2] = TT_MODE_EXT;
  tt_tx_buf[3] = TT_EXT_LFF_ORDER_REPORT;
  tt_tx_buf[4] = order;
  tt_tx_buf[5] = (rho > 0.0f) ? 1U : 0U;
  tt_tx_buf[6] = learning_mode;
  tt_tx_buf[7] = 0U;
  write_le_float(&tt_tx_buf[8], rho);
  write_le_float(&tt_tx_buf[12], residual_a_nm * torque_to_current);
  write_le_float(&tt_tx_buf[16], residual_b_nm * torque_to_current);
  write_le_float(&tt_tx_buf[20], learned_a_nm * torque_to_current);
  write_le_float(&tt_tx_buf[24], learned_b_nm * torque_to_current);
  write_le_u32(&tt_tx_buf[28], update_count);
  tt_tx_buf[32] = checksum_sum(tt_tx_buf, 2U, 32U);
  tt_tx_buf[33] = TT_TAIL0;
  tt_tx_buf[34] = TT_TAIL1;

  tt_tx_busy = 1U;
  if (HAL_UART_Transmit(&TT_UART_HANDLE, tt_tx_buf, 35U, 10U) != HAL_OK) {
    tt_status_tx_error_count++;
  }
  tt_tx_busy = 0U;
}

/* 学习交流表分块回报，共79字节：每帧固定返回16个位置槽。 */
static void send_lff_table_report(uint16_t start_bin)
{
  if (tt_tx_busy) {
    return;
  }
  if (start_bin >= PAPER_LFF_POSITION_BINS) {
    start_bin = PAPER_LFF_POSITION_BINS - 1U;
  }

  const uint8_t table_index = g_lff_active_table_index;
  uint8_t count = 0U;
  float table_current_a[16] = {0.0f};
  const float torque_to_current =
      (MOTOR_TORQUE_CONSTANT_NM_PER_A > 0.0f)
          ? 1.0f / MOTOR_TORQUE_CONSTANT_NM_PER_A
          : 0.0f;
  for (uint32_t index = 0U; index < 16U; ++index) {
    const uint32_t bin = (uint32_t)start_bin + index;
    if (bin < PAPER_LFF_POSITION_BINS) {
      table_current_a[index] =
          g_lff_table_bank_nm[table_index][bin] * torque_to_current;
      count++;
    }
  }

  tt_tx_buf[0] = TT_HEAD0;
  tt_tx_buf[1] = TT_HEAD1;
  tt_tx_buf[2] = TT_MODE_EXT;
  tt_tx_buf[3] = TT_EXT_LFF_TABLE_REPORT;
  write_le_u16(&tt_tx_buf[4], start_bin);
  tt_tx_buf[6] = count;
  tt_tx_buf[7] = table_index;
  write_le_float(&tt_tx_buf[8], g_lff_dc_torque_nm * torque_to_current);
  for (uint32_t index = 0U; index < 16U; ++index) {
    write_le_float(&tt_tx_buf[12U + 4U * index], table_current_a[index]);
  }
  tt_tx_buf[76] = checksum_sum(tt_tx_buf, 2U, 76U);
  tt_tx_buf[77] = TT_TAIL0;
  tt_tx_buf[78] = TT_TAIL1;

  tt_tx_busy = 1U;
  if (HAL_UART_Transmit(&TT_UART_HANDLE, tt_tx_buf, 79U, 10U) != HAL_OK) {
    tt_status_tx_error_count++;
  }
  tt_tx_busy = 0U;
}

/*
 * ESO配置帧，共20字节：
 *   A5 5A 05 1B 标志 带宽 补偿增益 独立限幅 校验和 0D 0A
 * 配置成功后在临界区内复位观测器，避免4 kHz更新读到半组参数。
 */
static void apply_eso_config(const uint8_t *frame)
{
  const float bandwidth_rad_s = read_le_float(&frame[5]);
  const float compensation_gain = read_le_float(&frame[9]);
  const float current_limit_a = read_le_float(&frame[13]);
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  tt_eso_config_accepted =
      Experiment_ESO_Configure(bandwidth_rad_s,
                               compensation_gain,
                               current_limit_a);
  if ((primask & 1U) == 0U) {
    __enable_irq();
  }
}

static void apply_experiment_config(const uint8_t *frame, uint8_t len)
{
  uint8_t mode = frame[4];
  uint8_t flags = frame[5];
  uint8_t learning_mode = frame[6];
  uint8_t start_order = frame[7];
  uint8_t end_order = frame[8];
  float injection_a = read_le_float(&frame[9]);

  if (mode <= PHYSICAL_EXPERIMENT_FIXED_FF_SWEEP) {
    g_experiment_mode_request = mode;
  }
  if (learning_mode <= POSITION_LFF_MODE_GLOBAL) {
    g_learning_mode_request = learning_mode;
  }

  if (start_order < 1U) {
    start_order = 1U;
  } else if (start_order > PAPER_LFF_MAX_ORDER) {
    start_order = PAPER_LFF_MAX_ORDER;
  }
  if (end_order < start_order) {
    end_order = start_order;
  } else if (end_order > PAPER_LFF_MAX_ORDER) {
    end_order = PAPER_LFF_MAX_ORDER;
  }
  g_sweep_start_order_request = start_order;
  g_sweep_end_order_request = end_order;

  if (isfinite(injection_a)) {
    if (injection_a < 0.0f) {
      injection_a = -injection_a;
    }
    if (mode == PHYSICAL_EXPERIMENT_FIXED_FF_SWEEP) {
      if (injection_a > EXPERIMENT_SWEEP_MAX_INJECTION_A) {
        injection_a = EXPERIMENT_SWEEP_MAX_INJECTION_A;
      }
      g_sweep_injection_amplitude_request_a = injection_a;
    } else {
      if (injection_a > PAPER_LFF_NOISE_MAX_CURRENT_A) {
        injection_a = PAPER_LFF_NOISE_MAX_CURRENT_A;
      }
      g_training_noise_amplitude_request_a = injection_a;
    }
  }
  if (len == 20U) {
    uint32_t seed = read_le_u32(&frame[13]);
    if (seed == 0U) {
      seed = PAPER_LFF_NOISE_DEFAULT_SEED;
    }
    g_training_noise_seed_request = seed;
  }

  g_learning_update_request = (flags & 0x01U) ? 1U : 0U;
  if ((flags & 0x02U) != 0U) {
    g_learning_reset_request = 1U;
  }
  g_sweep_enable_request = (flags & 0x04U) ? 1U : 0U;
  if ((flags & 0x08U) != 0U) {
    g_sweep_reset_request = 1U;
  }
  g_training_noise_enable_request =
      (((flags & 0x10U) != 0U) &&
       ((mode == PHYSICAL_EXPERIMENT_ESO_LEARNING_FEEDFORWARD) ||
        (mode == PHYSICAL_EXPERIMENT_FULL)))
          ? 1U
          : 0U;

  if (g_experiment_mode_request == PHYSICAL_EXPERIMENT_BASELINE) {
    g_learning_update_request = 0U;
    g_sweep_enable_request = 0U;
    g_training_noise_enable_request = 0U;
  }
}

static void handle_ext_frame(const uint8_t *frame, uint8_t len)
{
  if (len == 7U && frame[3] == TT_EXT_TARGET_QUERY) {
    send_target_report();
    return;
  }

  if (len == 7U && frame[3] == TT_EXT_ADC_DIAGNOSTIC_REPORT) {
    send_adc_diagnostic_report();
    return;
  }

  if (len == 8U && frame[3] == TT_EXT_SWEEP_RESULT_REPORT) {
    send_sweep_result_report(frame[4]);
    return;
  }

  if (frame[3] == TT_EXT_ESO_CONFIG) {
    if (len == 20U) {
      apply_eso_config(frame);
      send_eso_report();
    } else if (len == 7U) {
      send_eso_report();
    }
    return;
  }

  if (len == 7U && frame[3] == TT_EXT_LFF_REPORT) {
    send_lff_report();
    return;
  }

  if (len == 8U && frame[3] == TT_EXT_LFF_ORDER_REPORT) {
    send_lff_order_report(frame[4]);
    return;
  }

  if (len == 9U && frame[3] == TT_EXT_LFF_TABLE_REPORT) {
    send_lff_table_report(read_le_u16(&frame[4]));
    return;
  }

  if (frame[3] == TT_EXT_EXPERIMENT_CONFIG) {
    if ((len == 16U) || (len == 20U)) {
      apply_experiment_config(frame, len);
      send_experiment_report();
    } else if (len == 7U) {
      send_experiment_report();
    }
    return;
  }

  if (len != 10U || frame[3] != TT_EXT_PID_QUERY) {
    return;
  }

  uint8_t pid_mode = frame[4];
  uint8_t axis = frame[5];
  uint8_t loop = frame[6];

  if (loop == 0xFFU) {
    send_pid_report(pid_mode, axis, 0x00U);
    send_pid_report(pid_mode, axis, 0x03U);
    send_pid_report(pid_mode, axis, 0x01U);
    send_pid_report(pid_mode, axis, 0x02U);
  } else {
    send_pid_report(pid_mode, axis, loop);
  }
}

static void handle_frame(const uint8_t *frame, uint8_t len)
{
  if (!frame_is_valid(frame, len)) {
    tt_rx_invalid_frame_count++;
    return;
  }
  tt_rx_valid_frame_count++;

  uint8_t mode = frame[2];
  if (mode == TT_MODE_DISABLE && len == 7U && frame[3] == TT_AXIS_AZ) {
    Motor_Disable();
  } else if (mode == TT_MODE_ENABLE && len == 7U && frame[3] == TT_AXIS_AZ) {
    Motor_Enable();
  } else if (mode == TT_MODE_SERVO) {
    handle_servo_frame(frame, len);
  } else if (mode == TT_MODE_TRACK) {
    handle_tracking_frame(frame, len);
  } else if (mode == TT_MODE_EXT) {
    handle_ext_frame(frame, len);
  }
}

static void send_status_frame(void)
{
  HAL_StatusTypeDef tx_status;

  if (!Turntable_Comm_IsEnabled() || tt_tx_busy) {
    return;
  }

  tt_tx_buf[0] = TT_HEAD0;
  tt_tx_buf[1] = TT_HEAD1;
  write_le_float(&tt_tx_buf[2], current_angle_sp);
  write_le_float(&tt_tx_buf[6], 0.0f);
  tt_tx_buf[10] = TT_TAIL0;
  tt_tx_buf[11] = TT_TAIL1;

  tt_tx_busy = 1U;
  tx_status = HAL_UART_Transmit(&TT_UART_HANDLE, tt_tx_buf, 12U, 2U);
  if (tx_status != HAL_OK) {
    tt_status_tx_error_count++;
  } else {
    tt_status_tx_count++;
  }
  tt_tx_busy = 0U;

  /* 兼容位置帧发送成功后，再发送扩展遥测帧。 */
  if (tx_status == HAL_OK) {
    send_telemetry_frame();
    send_health_report();
  }
}

/*
 * 扩展遥测帧，共24字节：
 *   A5 5A 05 12 方位速度 俯仰占位速度 q轴电流 d轴电流 功率状态 校验和 0D 0A
 *
 * 为兼容现有上位机，前面仍保留12字节位置状态帧。current_speed_sp单位为RPM，
 * 上位机曲线使用度每秒，因此发送前乘以6。功率状态：0为关闭，
 * 1为等待新编码器样本，2为已使能。
 */
static void send_telemetry_frame(void)
{
  if (!Turntable_Comm_IsEnabled() || tt_tx_busy) {
    return;
  }

  tt_tx_buf[0] = TT_HEAD0;
  tt_tx_buf[1] = TT_HEAD1;
  tt_tx_buf[2] = TT_MODE_EXT;
  tt_tx_buf[3] = TT_EXT_TELEMETRY_REPORT;
  write_le_float(&tt_tx_buf[4], current_speed_sp * 6.0f);
  write_le_float(&tt_tx_buf[8], 0.0f);   /* 当前没有实体俯仰速度通道。 */
  write_le_float(&tt_tx_buf[12], i_q);   /* 转矩电流，单位：安。 */
  write_le_float(&tt_tx_buf[16], i_d);   /* 励磁电流，单位：安。 */
  tt_tx_buf[20] = FOC_GetPowerState();
  tt_tx_buf[21] = checksum_sum(tt_tx_buf, 2U, 21U);
  tt_tx_buf[22] = TT_TAIL0;
  tt_tx_buf[23] = TT_TAIL1;

  tt_tx_busy = 1U;
  if (HAL_UART_Transmit(&TT_UART_HANDLE, tt_tx_buf, 24U, 5U) != HAL_OK) {
    tt_status_tx_error_count++;
  }
  tt_tx_busy = 0U;
}

/*
 * 控制健康状态帧，共40字节：
 *   A5 5A 05 18 故障码 SSI帧龄 启动错误数 拒绝样本数 SPI错误数
 *   有效命令帧数 无效命令帧数 丢弃命令帧数 UART错误数 校验和 0D 0A
 */
static void send_health_report(void)
{
  if (!Turntable_Comm_IsEnabled() || tt_tx_busy) {
    return;
  }

  tt_tx_buf[0] = TT_HEAD0;
  tt_tx_buf[1] = TT_HEAD1;
  tt_tx_buf[2] = TT_MODE_EXT;
  tt_tx_buf[3] = TT_EXT_HEALTH_REPORT;
  tt_tx_buf[4] = (uint8_t)FOC_GetFaultCode();
  write_le_u32(&tt_tx_buf[5], SSI_GetFrameAgeMs());
  write_le_u32(&tt_tx_buf[9], SSI_GetTransferStartErrorCount());
  write_le_u32(&tt_tx_buf[13], SSI_GetRejectedSampleCount());
  write_le_u32(&tt_tx_buf[17], SSI_GetSpiErrorCount());
  write_le_u32(&tt_tx_buf[21], tt_rx_valid_frame_count);
  write_le_u32(&tt_tx_buf[25], tt_rx_invalid_frame_count);
  write_le_u32(&tt_tx_buf[29], tt_rx_dropped_frame_count);
  write_le_u32(&tt_tx_buf[33], tt_rx_uart_error_count);
  tt_tx_buf[37] = checksum_sum(tt_tx_buf, 2U, 37U);
  tt_tx_buf[38] = TT_TAIL0;
  tt_tx_buf[39] = TT_TAIL1;

  tt_tx_busy = 1U;
  if (HAL_UART_Transmit(&TT_UART_HANDLE, tt_tx_buf, 40U, 5U) != HAL_OK) {
    tt_status_tx_error_count++;
  }
  tt_tx_busy = 0U;
}

/*
 * ADC诊断查询回报帧，共40字节：
 *   A5 5A 05 19 标志
 *   U/V/W零点计数
 *   第二端点减第一端点的U/V/W电流
 *   ADC完整序列计数 FOC更新计数
 *   校验和 0D 0A
 *
 * 标志位0表示零点校准有效，位1表示正在等待第二端点样本。
 * 本帧只在收到A5 5A 05 19 1E 0D 0A查询时返回，不增加周期遥测负载。
 */
static void send_adc_diagnostic_report(void)
{
  if (!Turntable_Comm_IsEnabled() || tt_tx_busy) {
    return;
  }

  float offset[3];
  float endpoint_delta[3];
  uint32_t sequence_count;
  uint32_t control_update_count;
  uint8_t flags;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  for (uint32_t phase = 0U; phase < 3U; phase++) {
    offset[phase] = g_adc_offset[phase];
    endpoint_delta[phase] = g_adc_endpoint_delta_current[phase];
  }
  sequence_count = g_adc_sequence_count;
  control_update_count = g_adc_control_update_count;
  flags = (g_adc_calibrated != 0U ? 0x01U : 0x00U) |
          (g_adc_pair_pending != 0U ? 0x02U : 0x00U);
  if ((primask & 1U) == 0U) {
    __enable_irq();
  }

  tt_tx_buf[0] = TT_HEAD0;
  tt_tx_buf[1] = TT_HEAD1;
  tt_tx_buf[2] = TT_MODE_EXT;
  tt_tx_buf[3] = TT_EXT_ADC_DIAGNOSTIC_REPORT;
  tt_tx_buf[4] = flags;
  write_le_float(&tt_tx_buf[5], offset[0]);
  write_le_float(&tt_tx_buf[9], offset[1]);
  write_le_float(&tt_tx_buf[13], offset[2]);
  write_le_float(&tt_tx_buf[17], endpoint_delta[0]);
  write_le_float(&tt_tx_buf[21], endpoint_delta[1]);
  write_le_float(&tt_tx_buf[25], endpoint_delta[2]);
  write_le_u32(&tt_tx_buf[29], sequence_count);
  write_le_u32(&tt_tx_buf[33], control_update_count);
  tt_tx_buf[37] = checksum_sum(tt_tx_buf, 2U, 37U);
  tt_tx_buf[38] = TT_TAIL0;
  tt_tx_buf[39] = TT_TAIL1;

  tt_tx_busy = 1U;
  if (HAL_UART_Transmit(&TT_UART_HANDLE, tt_tx_buf, 40U, 5U) != HAL_OK) {
    tt_status_tx_error_count++;
  }
  tt_tx_busy = 0U;
}

static void restart_uart_receive(void)
{
  if (HAL_UARTEx_ReceiveToIdle_DMA(&TT_UART_HANDLE,
                                   tt_rx_dma_buf,
                                   TT_RX_DMA_BUFFER_SIZE) == HAL_OK) {
    tt_rx_uses_dma = 1U;
    /* 空闲线或缓冲区满时再处理，避免半传输回调拆散协议帧。 */
    __HAL_DMA_DISABLE_IT(TT_UART_HANDLE.hdmarx, DMA_IT_HT);
  } else {
    /* DMA异常时保留单字节中断兜底，通信不会完全失效。 */
    tt_rx_uses_dma = 0U;
    (void)HAL_UART_Receive_IT(&TT_UART_HANDLE, &tt_rx_byte, 1U);
  }
}

static void enqueue_current_rx_frame(void)
{
  if (tt_rx_queue_count >= TT_RX_QUEUE_DEPTH) {
    tt_rx_dropped_frame_count++;
    return;
  }

  const uint8_t head = tt_rx_queue_head;
  memcpy(tt_rx_queue[head], tt_rx_frame, tt_rx_len);
  tt_rx_queue_len[head] = tt_rx_len;
  tt_rx_queue_head = (uint8_t)((head + 1U) % TT_RX_QUEUE_DEPTH);
  tt_rx_queue_count++;
}

static void receive_protocol_byte(uint8_t byte)
{
  switch (tt_rx_state) {
    case TT_RX_WAIT_HEAD0:
      if (byte == TT_HEAD0) {
        tt_rx_frame[0] = byte;
        tt_rx_len = 1U;
        tt_rx_state = TT_RX_WAIT_HEAD1;
      }
      break;

    case TT_RX_WAIT_HEAD1:
      if (byte == TT_HEAD1) {
        tt_rx_frame[1] = byte;
        tt_rx_len = 2U;
        tt_rx_state = TT_RX_PAYLOAD;
      } else if (byte == TT_HEAD0) {
        tt_rx_frame[0] = byte;
        tt_rx_len = 1U;
      } else {
        tt_rx_len = 0U;
        tt_rx_state = TT_RX_WAIT_HEAD0;
      }
      break;

    case TT_RX_PAYLOAD:
      if (tt_rx_len < TT_RX_MAX_FRAME_SIZE) {
        tt_rx_frame[tt_rx_len++] = byte;
        if ((tt_rx_len >= 7U) &&
            (tt_rx_frame[tt_rx_len - 2U] == TT_TAIL0) &&
            (tt_rx_frame[tt_rx_len - 1U] == TT_TAIL1)) {
          enqueue_current_rx_frame();
          tt_rx_len = 0U;
          tt_rx_state = TT_RX_WAIT_HEAD0;
        }
      } else {
        tt_rx_len = 0U;
        tt_rx_state = TT_RX_WAIT_HEAD0;
      }
      break;

    default:
      tt_rx_len = 0U;
      tt_rx_state = TT_RX_WAIT_HEAD0;
      break;
  }
}

void Turntable_Comm_Init(void)
{
  if (!Turntable_Comm_IsEnabled()) {
    return;
  }

  tt_rx_state = TT_RX_WAIT_HEAD0;
  tt_rx_len = 0U;
  tt_rx_queue_head = 0U;
  tt_rx_queue_tail = 0U;
  tt_rx_queue_count = 0U;
  tt_rx_uses_dma = 0U;
  tt_tx_busy = 0U;
  tt_status_tx_count = 0U;
  tt_status_tx_error_count = 0U;
  tt_rx_dropped_frame_count = 0U;
  tt_rx_uart_error_count = 0U;
  tt_rx_valid_frame_count = 0U;
  tt_rx_invalid_frame_count = 0U;
  memset(&tt_servo_features, 0, sizeof(tt_servo_features));
  memset(&tt_tracking_features, 0, sizeof(tt_tracking_features));

  restart_uart_receive();
}

void Turntable_Comm_Task(void)
{
  static uint32_t last_status_ms = 0U;
  uint8_t local_frame[TT_RX_MAX_FRAME_SIZE];

  if (!Turntable_Comm_IsEnabled()) {
    return;
  }

  while (tt_rx_queue_count != 0U) {
    uint8_t local_len;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint8_t tail = tt_rx_queue_tail;
    local_len = tt_rx_queue_len[tail];
    memcpy(local_frame, tt_rx_queue[tail], local_len);
    tt_rx_queue_tail = (uint8_t)((tail + 1U) % TT_RX_QUEUE_DEPTH);
    tt_rx_queue_count--;
    if ((primask & 1U) == 0U) {
      __enable_irq();
    }

    handle_frame(local_frame, local_len);
  }

  const uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - last_status_ms) >= TT_STATUS_PERIOD_MS) {
    last_status_ms = now;
    send_status_frame();
  }
}

void Turntable_Comm_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
  if (!Turntable_Comm_IsEnabled() || huart->Instance != TT_UART_INSTANCE ||
      tt_rx_uses_dma != 0U) {
    return;
  }

  receive_protocol_byte(tt_rx_byte);
  (void)HAL_UART_Receive_IT(&TT_UART_HANDLE, &tt_rx_byte, 1U);
}

void Turntable_Comm_UartRxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if (!Turntable_Comm_IsEnabled() || huart->Instance != TT_UART_INSTANCE ||
      tt_rx_uses_dma == 0U) {
    return;
  }

  if (size > TT_RX_DMA_BUFFER_SIZE) {
    size = TT_RX_DMA_BUFFER_SIZE;
  }
  for (uint16_t i = 0U; i < size; i++) {
    receive_protocol_byte(tt_rx_dma_buf[i]);
  }
  restart_uart_receive();
}

void Turntable_Comm_UartTxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == TT_UART_INSTANCE) {
    tt_tx_busy = 0U;
  }
}

void Turntable_Comm_UartErrorCallback(UART_HandleTypeDef *huart)
{
  if (Turntable_Comm_IsEnabled() && huart->Instance == TT_UART_INSTANCE) {
    tt_rx_uart_error_count++;
    tt_rx_len = 0U;
    tt_rx_state = TT_RX_WAIT_HEAD0;
    tt_tx_busy = 0U;
    restart_uart_receive();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  Turntable_Comm_UartRxCpltCallback(huart);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  Turntable_Comm_UartRxEventCallback(huart, size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  Turntable_Comm_UartTxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  Turntable_Comm_UartErrorCallback(huart);
}
