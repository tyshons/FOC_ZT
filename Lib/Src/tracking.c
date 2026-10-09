#include "tracking.h"

#include "usart.h"

#include <math.h>

#define TRACKING_UART_HANDLE huart3
#define TRACKING_UART_INSTANCE USART3
#define TRACKING_RX_DMA_SIZE 64U
#define TRACKING_RX_QUEUE_SIZE 256U
#define TRACKING_FRAME_HEAD0 0xA5U
#define TRACKING_FRAME_HEAD1 0x5AU
#define TRACKING_FRAME_MODE  0x03U
#define TRACKING_FRAME_TAIL0 0x0DU
#define TRACKING_FRAME_TAIL1 0x0AU
#define TRACKING_FRAME_SIZE  10U
#define TRACKING_SP_ERROR_SIGN  1.0f
#define TRACKING_FY_ERROR_SIGN  1.0f

volatile uint8_t tracking_enabled = 0U;
volatile uint8_t tracking_target_valid = 0U;
volatile float tracking_error_sp = 0.0f;
volatile float tracking_error_fy = 0.0f;
volatile float tracking_speed_command_sp = 0.0f;
volatile float tracking_speed_command_fy = 0.0f;
volatile uint32_t tracking_last_update_ms = 0U;
volatile uint32_t tracking_rx_byte_count = 0U;
volatile uint32_t tracking_rx_dropped_byte_count = 0U;
volatile uint32_t tracking_input_frame_count = 0U;

static PID_TypeDef tracking_pid_inst_sp = {
  .Kp = 0.10f,
  .Ki = 0.0f,
  .Kd = 0.0f,
  .output_min = -TRACKING_SPEED_LIMIT_RPM,
  .output_max = TRACKING_SPEED_LIMIT_RPM,
  .integral_limit = TRACKING_SPEED_LIMIT_RPM,
  .low_pass_filter_time_constant = 0.01f};
static PID_TypeDef tracking_pid_inst_fy = {
  .Kp = 0.10f,
  .Ki = 0.0f,
  .Kd = 0.0f,
  .output_min = -TRACKING_SPEED_LIMIT_RPM,
  .output_max = TRACKING_SPEED_LIMIT_RPM,
  .integral_limit = TRACKING_SPEED_LIMIT_RPM,
  .low_pass_filter_time_constant = 0.01f};

static uint8_t tracking_rx_byte;
static uint8_t tracking_rx_dma[TRACKING_RX_DMA_SIZE];
static uint8_t tracking_rx_queue[TRACKING_RX_QUEUE_SIZE];
static volatile uint16_t tracking_rx_head;
static volatile uint16_t tracking_rx_tail;
static volatile uint8_t tracking_rx_uses_dma;
static uint32_t tracking_processed_frame_count;
static uint8_t tracking_output_active;
static uint8_t tracking_protocol_frame[TRACKING_FRAME_SIZE];
static uint8_t tracking_protocol_index;

static void tracking_restart_receive(void)
{
  if (HAL_UARTEx_ReceiveToIdle_DMA(&TRACKING_UART_HANDLE,
                                   tracking_rx_dma,
                                   TRACKING_RX_DMA_SIZE) == HAL_OK) {
    tracking_rx_uses_dma = 1U;
    __HAL_DMA_DISABLE_IT(TRACKING_UART_HANDLE.hdmarx, DMA_IT_HT);
  } else {
    tracking_rx_uses_dma = 0U;
    (void)HAL_UART_Receive_IT(&TRACKING_UART_HANDLE, &tracking_rx_byte, 1U);
  }
}

static void tracking_enqueue_byte(uint8_t byte)
{
  const uint16_t next =
      (uint16_t)((tracking_rx_head + 1U) % TRACKING_RX_QUEUE_SIZE);
  tracking_rx_byte_count++;
  if (next == tracking_rx_tail) {
    tracking_rx_dropped_byte_count++;
    return;
  }
  tracking_rx_queue[tracking_rx_head] = byte;
  tracking_rx_head = next;
}

static void tracking_protocol_restart_from_byte(uint8_t byte)
{
  tracking_protocol_index = 0U;
  if (byte == TRACKING_FRAME_HEAD0) {
    tracking_protocol_frame[0] = byte;
    tracking_protocol_index = 1U;
  }
}

static int16_t tracking_read_le_i16(const uint8_t *data)
{
  const uint16_t raw =
      (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
  if ((raw & 0x8000U) != 0U) {
    return (int16_t)((int32_t)raw - 65536L);
  }
  return (int16_t)raw;
}

static void tracking_protocol_parse_byte(uint8_t byte)
{
  if (tracking_protocol_index == 0U) {
    tracking_protocol_restart_from_byte(byte);
    return;
  }

  if (tracking_protocol_index == 1U) {
    if (byte == TRACKING_FRAME_HEAD1) {
      tracking_protocol_frame[1] = byte;
      tracking_protocol_index = 2U;
    } else {
      tracking_protocol_restart_from_byte(byte);
    }
    return;
  }

  tracking_protocol_frame[tracking_protocol_index++] = byte;

  if ((tracking_protocol_index == 3U) &&
      (tracking_protocol_frame[2] != TRACKING_FRAME_MODE)) {
    tracking_protocol_restart_from_byte(byte);
    return;
  }
  if ((tracking_protocol_index == 9U) &&
      (tracking_protocol_frame[8] != TRACKING_FRAME_TAIL0)) {
    tracking_protocol_restart_from_byte(byte);
    return;
  }
  if (tracking_protocol_index < TRACKING_FRAME_SIZE) {
    return;
  }

  const uint8_t checksum =
      (uint8_t)((uint16_t)tracking_protocol_frame[2] +
                (uint16_t)tracking_protocol_frame[3] +
                (uint16_t)tracking_protocol_frame[4] +
                (uint16_t)tracking_protocol_frame[5] +
                (uint16_t)tracking_protocol_frame[6]);
  if ((tracking_protocol_frame[9] == TRACKING_FRAME_TAIL1) &&
      (tracking_protocol_frame[7] == checksum)) {
    const int16_t error_x =
        tracking_read_le_i16(&tracking_protocol_frame[3]);
    const int16_t error_y =
        tracking_read_le_i16(&tracking_protocol_frame[5]);
    Tracking_InputUpdate(TRACKING_SP_ERROR_SIGN * (float)error_x,
      TRACKING_FY_ERROR_SIGN * (float)error_y, 1U);
  }

  tracking_protocol_restart_from_byte(byte);
}

static void tracking_stop_motion(void)
{
  if (tracking_output_active != 0U) {
    FOC_SetSpeedTarget(FOC_AXIS_SP, 0.0f);
    FOC_SetSpeedTarget(FOC_AXIS_FY, 0.0f);
  }
  tracking_speed_command_sp = 0.0f;
  tracking_speed_command_fy = 0.0f;
  tracking_output_active = 0U;
  PID_Reset(&tracking_pid_inst_sp);
  PID_Reset(&tracking_pid_inst_fy);
}

void Tracking_Init(void)
{
  PID_Init(&tracking_pid_inst_sp);
  PID_Init(&tracking_pid_inst_fy);
  tracking_rx_head = 0U;
  tracking_rx_tail = 0U;
  tracking_rx_uses_dma = 0U;
  tracking_protocol_index = 0U;
  tracking_processed_frame_count = 0U;
  tracking_output_active = 0U;
  tracking_restart_receive();
}

void Tracking_Task(void)
{
  while (tracking_rx_tail != tracking_rx_head) {
    const uint8_t byte = tracking_rx_queue[tracking_rx_tail];
    tracking_rx_tail =
        (uint16_t)((tracking_rx_tail + 1U) % TRACKING_RX_QUEUE_SIZE);
    tracking_protocol_parse_byte(byte);
  }

  if (tracking_enabled == 0U) {
    return;
  }

  const uint32_t now = HAL_GetTick();
  if ((tracking_target_valid == 0U) ||
      ((uint32_t)(now - tracking_last_update_ms) > TRACKING_INPUT_TIMEOUT_MS)) {
    tracking_stop_motion();
    return;
  }
  if (tracking_processed_frame_count == tracking_input_frame_count) {
    return;
  }

  tracking_processed_frame_count = tracking_input_frame_count;
  const uint32_t now_us = now * 1000U;
  tracking_speed_command_sp =
      PID_Update(&tracking_pid_inst_sp, tracking_error_sp, now_us);
  tracking_speed_command_fy =
      PID_Update(&tracking_pid_inst_fy, tracking_error_fy, now_us);
  FOC_SetSpeedTarget(FOC_AXIS_SP, tracking_speed_command_sp);
  FOC_SetSpeedTarget(FOC_AXIS_FY, tracking_speed_command_fy);
  tracking_output_active = 1U;
}

void Tracking_SetEnabled(uint8_t enabled)
{
  tracking_enabled = enabled ? 1U : 0U;
  tracking_target_valid = 0U;
  tracking_processed_frame_count = tracking_input_frame_count;
  tracking_stop_motion();
  if (tracking_enabled != 0U) {
    FOC_SetSpeedTarget(FOC_AXIS_SP, 0.0f);
    FOC_SetSpeedTarget(FOC_AXIS_FY, 0.0f);
  } else {
    FOC_SetPositionTarget(FOC_AXIS_SP, current_angle_sp);
    FOC_SetPositionTarget(FOC_AXIS_FY, current_angle_fy);
  }
}

uint8_t Tracking_IsEnabled(void)
{
  return tracking_enabled;
}

void Tracking_InputUpdate(float error_sp, float error_fy, uint8_t target_valid)
{
  if (!isfinite(error_sp) || !isfinite(error_fy)) {
    target_valid = 0U;
    error_sp = 0.0f;
    error_fy = 0.0f;
  }
  tracking_error_sp = error_sp;
  tracking_error_fy = error_fy;
  tracking_target_valid = target_valid ? 1U : 0U;
  tracking_last_update_ms = HAL_GetTick();
  tracking_input_frame_count++;
}

PID_TypeDef *Tracking_GetPid(FOC_Axis axis)
{
  return (axis == FOC_AXIS_FY)
             ? &tracking_pid_inst_fy
             : &tracking_pid_inst_sp;
}

void Tracking_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart->Instance != TRACKING_UART_INSTANCE) ||
      (tracking_rx_uses_dma != 0U)) {
    return;
  }
  tracking_enqueue_byte(tracking_rx_byte);
  (void)HAL_UART_Receive_IT(&TRACKING_UART_HANDLE, &tracking_rx_byte, 1U);
}

void Tracking_UartRxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if ((huart->Instance != TRACKING_UART_INSTANCE) ||
      (tracking_rx_uses_dma == 0U)) {
    return;
  }
  if (size > TRACKING_RX_DMA_SIZE) {
    size = TRACKING_RX_DMA_SIZE;
  }
  for (uint16_t index = 0U; index < size; index++) {
    tracking_enqueue_byte(tracking_rx_dma[index]);
  }
  tracking_restart_receive();
}

void Tracking_UartErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == TRACKING_UART_INSTANCE) {
    tracking_protocol_index = 0U;
    tracking_restart_receive();
  }
}
