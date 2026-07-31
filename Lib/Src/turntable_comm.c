#include "turntable_comm.h"

#include "FOC_Control.h"
#include "PID_Control.h"
#include "usart.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define TT_HEAD0 0xA5U
#define TT_HEAD1 0x5AU
#define TT_TAIL0 0x0DU
#define TT_TAIL1 0x0AU

#define TT_RX_MAX_FRAME_SIZE 32U
#define TT_STATUS_PERIOD_MS 50U

#define TT_MODE_DISABLE 0x00U
#define TT_MODE_ENABLE  0x01U
#define TT_MODE_SERVO   0x02U
#define TT_MODE_TRACK   0x03U
#define TT_MODE_SWEEP   0x04U
#define TT_MODE_EXT     0x05U

#define TT_FUNC_PID     0x00U
#define TT_FUNC_ANGLE   0x01U
#define TT_FUNC_SPEED   0x02U

#define TT_EXT_PID_QUERY  0x10U
#define TT_EXT_PID_REPORT 0x11U
#define TT_EXT_TELEMETRY_REPORT 0x12U
#define TT_EXT_TARGET_QUERY  0x13U
#define TT_EXT_TARGET_REPORT 0x14U

#define TT_AXIS_AZ      0x00U
#define TT_AXIS_EL      0x01U

#define TT_UART_HANDLE  huart1
#define TT_UART_INSTANCE USART1

extern float current_angle_sp;
/* Read-only FOC runtime measurements.  The control/driver layer is unchanged. */
extern float current_speed_sp;
extern float position_given_sp;
extern float i_d;
extern float i_q;

static void send_telemetry_frame(void);

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

typedef struct {
  uint8_t active;
  uint8_t axis;
  float amplitude;
  float start_hz;
  float end_hz;
  float period_count;
  float step_count;
} TT_SweepState;

static uint8_t tt_rx_byte;
static volatile TT_RxState tt_rx_state;
static volatile uint8_t tt_rx_len;
static uint8_t tt_rx_frame[TT_RX_MAX_FRAME_SIZE];

static volatile uint8_t tt_frame_ready;
static volatile uint8_t tt_pending_len;
static uint8_t tt_pending_frame[TT_RX_MAX_FRAME_SIZE];

static volatile uint8_t tt_tx_busy;
static uint8_t tt_tx_buf[32];

volatile uint32_t tt_status_tx_count;
volatile uint32_t tt_status_tx_error_count;

static float tt_el_target_deg;
static float tt_el_actual_deg;
static TT_FeatureState tt_servo_features;
static TT_FeatureState tt_tracking_features;
static TT_SweepState tt_sweep_state;

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
  if (axis == TT_AXIS_AZ) {
    FOC_SetPositionTarget(target_deg);
  } else if (axis == TT_AXIS_EL) {
    tt_el_target_deg = target_deg;
    tt_el_actual_deg = target_deg;
  }
}

static void apply_pid(uint8_t axis, uint8_t loop, float kp, float ki, float kd)
{
  (void)axis;
  PID_TypeDef *pid = pid_from_loop(loop);
  if (pid == NULL) {
    return;
  }

  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;
  PID_Reset(pid);
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
      float el_target = read_le_float(&frame[8]);
      apply_axis_angle(TT_AXIS_AZ, az_target);
      apply_axis_angle(TT_AXIS_EL, el_target);
    } else if (len == 12U) {
      uint8_t axis = frame[4];
      float target = read_le_float(&frame[5]);
      apply_axis_angle(axis, target);
    }
  } else if (func == TT_FUNC_SPEED) {
    if (len == 12U && frame[4] == TT_AXIS_AZ) {
      FOC_SetSpeedTarget(read_le_float(&frame[5]));
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

static void handle_sweep_frame(const uint8_t *frame, uint8_t len)
{
  if (len != 27U) {
    return;
  }

  tt_sweep_state.active = 1U;
  tt_sweep_state.axis = frame[3];
  tt_sweep_state.amplitude = read_le_float(&frame[4]);
  tt_sweep_state.start_hz = read_le_float(&frame[8]);
  tt_sweep_state.end_hz = read_le_float(&frame[12]);
  tt_sweep_state.period_count = read_le_float(&frame[16]);
  tt_sweep_state.step_count = read_le_float(&frame[20]);
}

static void send_pid_report(uint8_t pid_mode, uint8_t axis, uint8_t loop)
{
  PID_TypeDef *pid = pid_from_loop(loop);
  if (pid == NULL || tt_tx_busy) {
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
    Turntable_Comm_UartErrorCallback(&TT_UART_HANDLE);
  }
  tt_tx_busy = 0U;
}

/* Extension target report (16 bytes):
 * A5 5A 05 14 valid az_target el_target checksum 0D 0A */
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
  write_le_float(&tt_tx_buf[9], tt_el_target_deg);
  tt_tx_buf[13] = checksum_sum(tt_tx_buf, 2U, 13U);
  tt_tx_buf[14] = TT_TAIL0;
  tt_tx_buf[15] = TT_TAIL1;

  tt_tx_busy = 1U;
  if (HAL_UART_Transmit(&TT_UART_HANDLE, tt_tx_buf, 16U, 5U) != HAL_OK) {
    tt_status_tx_error_count++;
    Turntable_Comm_UartErrorCallback(&TT_UART_HANDLE);
  }
  tt_tx_busy = 0U;
}

static void handle_ext_frame(const uint8_t *frame, uint8_t len)
{
  if (len == 7U && frame[3] == TT_EXT_TARGET_QUERY) {
    send_target_report();
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
    return;
  }

  uint8_t mode = frame[2];
  if (mode == TT_MODE_DISABLE && len == 7U) {
    Motor_Disable();
  } else if (mode == TT_MODE_ENABLE && len == 7U) {
    Motor_Enable();
  } else if (mode == TT_MODE_SERVO) {
    handle_servo_frame(frame, len);
  } else if (mode == TT_MODE_TRACK) {
    handle_tracking_frame(frame, len);
  } else if (mode == TT_MODE_SWEEP) {
    handle_sweep_frame(frame, len);
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
  write_le_float(&tt_tx_buf[6], tt_el_actual_deg);
  tt_tx_buf[10] = TT_TAIL0;
  tt_tx_buf[11] = TT_TAIL1;

  tt_tx_busy = 1U;
  tx_status = HAL_UART_Transmit(&TT_UART_HANDLE, tt_tx_buf, 12U, 2U);
  if (tx_status != HAL_OK) {
    tt_status_tx_error_count++;
    Turntable_Comm_UartErrorCallback(&TT_UART_HANDLE);
  } else {
    tt_status_tx_count++;
  }
  tt_tx_busy = 0U;

  /* Send the optional telemetry only after the legacy position frame. */
  if (tx_status == HAL_OK) {
    send_telemetry_frame();
  }
}

/*
 * Extension telemetry frame (24 bytes):
 *   A5 5A 05 12 az_speed el_speed iq id power_state checksum 0D 0A
 *
 * The original 12-byte position status frame is deliberately retained above
 * for compatibility with existing host software.  current_speed_sp is in RPM;
 * the host chart uses degrees/second, hence the factor of six. power_state is
 * 0 = disabled, 1 = waiting for fresh encoder samples, 2 = enabled.
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
  write_le_float(&tt_tx_buf[8], 0.0f);   /* No physical elevation speed channel. */
  write_le_float(&tt_tx_buf[12], i_q);   /* Torque-producing current, A. */
  write_le_float(&tt_tx_buf[16], i_d);   /* Flux current, A. */
  tt_tx_buf[20] = FOC_GetPowerState();
  tt_tx_buf[21] = checksum_sum(tt_tx_buf, 2U, 21U);
  tt_tx_buf[22] = TT_TAIL0;
  tt_tx_buf[23] = TT_TAIL1;

  tt_tx_busy = 1U;
  if (HAL_UART_Transmit(&TT_UART_HANDLE, tt_tx_buf, 24U, 5U) != HAL_OK) {
    tt_status_tx_error_count++;
    Turntable_Comm_UartErrorCallback(&TT_UART_HANDLE);
  }
  tt_tx_busy = 0U;
}

void Turntable_Comm_Init(void)
{
  if (!Turntable_Comm_IsEnabled()) {
    return;
  }

  tt_rx_state = TT_RX_WAIT_HEAD0;
  tt_rx_len = 0U;
  tt_frame_ready = 0U;
  tt_pending_len = 0U;
  tt_tx_busy = 0U;
  tt_status_tx_count = 0U;
  tt_status_tx_error_count = 0U;
  tt_el_target_deg = 0.0f;
  tt_el_actual_deg = 0.0f;
  memset(&tt_servo_features, 0, sizeof(tt_servo_features));
  memset(&tt_tracking_features, 0, sizeof(tt_tracking_features));
  memset(&tt_sweep_state, 0, sizeof(tt_sweep_state));

  HAL_UART_Receive_IT(&TT_UART_HANDLE, &tt_rx_byte, 1U);
}

void Turntable_Comm_Task(void)
{
  static uint32_t last_status_ms = 0U;
  uint8_t local_frame[TT_RX_MAX_FRAME_SIZE];
  uint8_t local_len = 0U;

  if (!Turntable_Comm_IsEnabled()) {
    return;
  }

  if (tt_frame_ready) {
    __disable_irq();
    local_len = tt_pending_len;
    memcpy(local_frame, tt_pending_frame, local_len);
    tt_frame_ready = 0U;
    __enable_irq();

    handle_frame(local_frame, local_len);
  }

  uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - last_status_ms) >= TT_STATUS_PERIOD_MS) {
    last_status_ms = now;
    send_status_frame();
  }
}

void Turntable_Comm_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
  if (!Turntable_Comm_IsEnabled() || huart->Instance != TT_UART_INSTANCE) {
    return;
  }

  switch (tt_rx_state) {
    case TT_RX_WAIT_HEAD0:
      if (tt_rx_byte == TT_HEAD0) {
        tt_rx_frame[0] = tt_rx_byte;
        tt_rx_len = 1U;
        tt_rx_state = TT_RX_WAIT_HEAD1;
      }
      break;

    case TT_RX_WAIT_HEAD1:
      if (tt_rx_byte == TT_HEAD1) {
        tt_rx_frame[1] = tt_rx_byte;
        tt_rx_len = 2U;
        tt_rx_state = TT_RX_PAYLOAD;
      } else if (tt_rx_byte == TT_HEAD0) {
        tt_rx_frame[0] = tt_rx_byte;
        tt_rx_len = 1U;
      } else {
        tt_rx_len = 0U;
        tt_rx_state = TT_RX_WAIT_HEAD0;
      }
      break;

    case TT_RX_PAYLOAD:
      if (tt_rx_len < TT_RX_MAX_FRAME_SIZE) {
        tt_rx_frame[tt_rx_len++] = tt_rx_byte;
        if (tt_rx_len >= 7U &&
            tt_rx_frame[tt_rx_len - 2U] == TT_TAIL0 &&
            tt_rx_frame[tt_rx_len - 1U] == TT_TAIL1) {
          if (!tt_frame_ready) {
            memcpy(tt_pending_frame, tt_rx_frame, tt_rx_len);
            tt_pending_len = tt_rx_len;
            tt_frame_ready = 1U;
          }
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

  HAL_UART_Receive_IT(&TT_UART_HANDLE, &tt_rx_byte, 1U);
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
    tt_rx_len = 0U;
    tt_rx_state = TT_RX_WAIT_HEAD0;
    tt_tx_busy = 0U;
    HAL_UART_Receive_IT(&TT_UART_HANDLE, &tt_rx_byte, 1U);
  }
}
