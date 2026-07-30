#include "vofa.h"

#include "FOC_Control.h"
#include "PID_Control.h"
#include "adc.h"
#include "usart.h"

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define VOFA_RX_LINE_SIZE 96U
#define VOFA_TX_BUF_SIZE 384U
#define VOFA_TELEMETRY_PERIOD_MS 20U

extern float current_angle_sp;
extern float current_speed_sp;
extern float position_given_sp;
extern float speed_given_sp;
extern float id_given_sp;
extern float iq_given_sp;
extern float theta;
extern float i_d;
extern float i_q;
extern float u_d;
extern float u_q;

static uint8_t vofa_rx_byte;
static volatile uint16_t vofa_rx_len;
static volatile uint8_t vofa_line_ready;
static char vofa_rx_line[VOFA_RX_LINE_SIZE];
static char vofa_cmd_line[VOFA_RX_LINE_SIZE];

static volatile uint8_t vofa_tx_busy;
static char vofa_tx_buf[VOFA_TX_BUF_SIZE];

uint8_t Vofa_IsEnabled(void)
{
  return VOFA_ENABLE ? 1U : 0U;
}

static void to_lower_string(char *s)
{
  while (*s != '\0') {
    *s = (char)tolower((unsigned char)*s);
    s++;
  }
}

static PID_TypeDef *pid_from_name(const char *name)
{
  if (strcmp(name, "pos") == 0 || strcmp(name, "position") == 0) {
    return &position_pid_inst;
  }
  if (strcmp(name, "speed") == 0 || strcmp(name, "spd") == 0) {
    return &speed_pid_inst;
  }
  if (strcmp(name, "id") == 0) {
    return &id_pid_inst;
  }
  if (strcmp(name, "iq") == 0) {
    return &iq_pid_inst;
  }
  return NULL;
}

static char *append_str(char *p, const char *end, const char *s)
{
  while (*s != '\0' && p < end) {
    *p++ = *s++;
  }
  return p;
}

static char *append_uint(char *p, const char *end, uint32_t value, uint8_t min_width)
{
  char tmp[11];
  uint8_t len = 0;

  do {
    tmp[len++] = (char)('0' + (value % 10U));
    value /= 10U;
  } while (value > 0U && len < sizeof(tmp));

  while (len < min_width && p < end) {
    *p++ = '0';
    min_width--;
  }

  while (len > 0U && p < end) {
    *p++ = tmp[--len];
  }

  return p;
}

static char *append_float(char *p, const char *end, float value, uint8_t decimals)
{
  if (isnan(value)) {
    return append_str(p, end, "nan");
  }
  if (isinf(value)) {
    return append_str(p, end, value < 0.0f ? "-inf" : "inf");
  }

  if (value < 0.0f) {
    if (p < end) {
      *p++ = '-';
    }
    value = -value;
  }

  uint32_t scale = 1U;
  for (uint8_t i = 0; i < decimals; i++) {
    scale *= 10U;
  }

  uint32_t whole = (uint32_t)value;
  uint32_t frac = (uint32_t)((value - (float)whole) * (float)scale + 0.5f);
  if (frac >= scale) {
    whole++;
    frac -= scale;
  }

  p = append_uint(p, end, whole, 0U);
  if (decimals > 0U && p < end) {
    *p++ = '.';
    p = append_uint(p, end, frac, decimals);
  }

  return p;
}

static void vofa_send_buffer(const char *buf, size_t len)
{
  if (!Vofa_IsEnabled() || vofa_tx_busy || len == 0U) {
    return;
  }

  vofa_tx_busy = 1U;
  if (HAL_UART_Transmit_IT(&huart3, (uint8_t *)buf, (uint16_t)len) != HAL_OK) {
    vofa_tx_busy = 0U;
  }
}

static void vofa_send_text(const char *text)
{
  vofa_send_buffer(text, strlen(text));
}

static char *append_pid_line(char *p, const char *end, const char *name, const PID_TypeDef *pid)
{
  p = append_str(p, end, "#pid ");
  p = append_str(p, end, name);
  p = append_str(p, end, " kp=");
  p = append_float(p, end, pid->Kp, 4U);
  p = append_str(p, end, " ki=");
  p = append_float(p, end, pid->Ki, 4U);
  p = append_str(p, end, " kd=");
  p = append_float(p, end, pid->Kd, 4U);
  p = append_str(p, end, " out=");
  p = append_float(p, end, pid->output_min, 2U);
  p = append_str(p, end, ",");
  p = append_float(p, end, pid->output_max, 2U);
  p = append_str(p, end, " ilim=");
  p = append_float(p, end, pid->integral_limit, 2U);
  p = append_str(p, end, "\r\n");
  return p;
}

static void vofa_send_all_pid(void)
{
  char *p = vofa_tx_buf;
  const char *end = vofa_tx_buf + sizeof(vofa_tx_buf) - 1U;

  p = append_pid_line(p, end, "pos", &position_pid_inst);
  p = append_pid_line(p, end, "speed", &speed_pid_inst);
  p = append_pid_line(p, end, "id", &id_pid_inst);
  p = append_pid_line(p, end, "iq", &iq_pid_inst);

  vofa_send_buffer(vofa_tx_buf, (size_t)(p - vofa_tx_buf));
}

static bool parse_float_arg(const char *text, float *out)
{
  char *end = NULL;
  float value = strtof(text, &end);
  if (end == text) {
    return false;
  }
  *out = value;
  return true;
}

static void handle_pid_command(int argc, char **argv)
{
  if (argc < 3) {
    vofa_send_text("#usage: pid <pos|speed|id|iq> <kp|ki|kd|ilim|out|reset> <value>\r\n");
    return;
  }

  PID_TypeDef *pid = pid_from_name(argv[1]);
  if (pid == NULL) {
    vofa_send_text("#err pid name\r\n");
    return;
  }

  if (strcmp(argv[2], "reset") == 0) {
    PID_Reset(pid);
    vofa_send_text("#ok pid reset\r\n");
    return;
  }

  if (argc < 4) {
    vofa_send_text("#usage: pid <pos|speed|id|iq> <kp|ki|kd|ilim|out|reset> <value>\r\n");
    return;
  }

  float value = 0.0f;
  if (!parse_float_arg(argv[3], &value)) {
    vofa_send_text("#err value\r\n");
    return;
  }

  if (strcmp(argv[2], "kp") == 0) {
    pid->Kp = value;
  } else if (strcmp(argv[2], "ki") == 0) {
    pid->Ki = value;
  } else if (strcmp(argv[2], "kd") == 0) {
    pid->Kd = value;
  } else if (strcmp(argv[2], "ilim") == 0) {
    PID_SetIntegralLimit(pid, value);
  } else if (strcmp(argv[2], "out") == 0 && argc >= 5) {
    float max_value = 0.0f;
    if (!parse_float_arg(argv[4], &max_value)) {
      vofa_send_text("#err out max\r\n");
      return;
    }
    PID_SetOutputLimits(pid, value, max_value);
  } else {
    vofa_send_text("#err pid field\r\n");
    return;
  }

  vofa_send_text("#ok pid\r\n");
}

static void handle_setpoint_command(int argc, char **argv)
{
  if (argc < 3) {
    vofa_send_text("#usage: sp <pos|speed|id|iq> <value>\r\n");
    return;
  }

  float value = 0.0f;
  if (!parse_float_arg(argv[2], &value)) {
    vofa_send_text("#err value\r\n");
    return;
  }

  if (strcmp(argv[1], "pos") == 0 || strcmp(argv[1], "position") == 0) {
    position_given_sp = value;
  } else if (strcmp(argv[1], "speed") == 0 || strcmp(argv[1], "spd") == 0) {
    speed_given_sp = value;
  } else if (strcmp(argv[1], "id") == 0) {
    id_given_sp = value;
  } else if (strcmp(argv[1], "iq") == 0) {
    iq_given_sp = value;
  } else {
    vofa_send_text("#err sp name\r\n");
    return;
  }

  vofa_send_text("#ok sp\r\n");
}

static void handle_command(char *line)
{
  char *argv[6];
  int argc = 0;

  to_lower_string(line);

  char *tok = strtok(line, " ,=\t\r\n");
  while (tok != NULL && argc < (int)(sizeof(argv) / sizeof(argv[0]))) {
    argv[argc++] = tok;
    tok = strtok(NULL, " ,=\t\r\n");
  }

  if (argc == 0) {
    return;
  }

  if (strcmp(argv[0], "pid") == 0) {
    handle_pid_command(argc, argv);
  } else if (strcmp(argv[0], "sp") == 0 || strcmp(argv[0], "set") == 0) {
    handle_setpoint_command(argc, argv);
  } else if (strcmp(argv[0], "enable") == 0 && argc >= 2) {
    if (strcmp(argv[1], "0") == 0 || strcmp(argv[1], "off") == 0) {
      Motor_Disable();
      vofa_send_text("#ok motor off\r\n");
    } else {
      Motor_Enable();
      vofa_send_text("#ok motor on\r\n");
    }
  } else if (strcmp(argv[0], "reset") == 0) {
    PID_Reset(&position_pid_inst);
    PID_Reset(&speed_pid_inst);
    PID_Reset(&id_pid_inst);
    PID_Reset(&iq_pid_inst);
    vofa_send_text("#ok reset all pid\r\n");
  } else if (strcmp(argv[0], "show") == 0) {
    vofa_send_all_pid();
  } else {
    vofa_send_text("#cmd: pid/sp/enable/reset/show\r\n");
  }
}

static void send_telemetry(void)
{
  char *p = vofa_tx_buf;
  const char *end = vofa_tx_buf + sizeof(vofa_tx_buf) - 1U;

  p = append_float(p, end, current_angle_sp, 3U);
  p = append_str(p, end, ",");
  p = append_float(p, end, position_given_sp, 3U);
  p = append_str(p, end, ",");
  p = append_float(p, end, current_speed_sp, 3U);
  p = append_str(p, end, ",");
  p = append_float(p, end, speed_given_sp, 3U);
  p = append_str(p, end, ",");
  p = append_float(p, end, iq_given_sp, 3U);
  p = append_str(p, end, ",");
  p = append_float(p, end, i_d, 3U);
  p = append_str(p, end, ",");
  p = append_float(p, end, i_q, 3U);
  p = append_str(p, end, ",");
  p = append_float(p, end, u_d, 3U);
  p = append_str(p, end, ",");
  p = append_float(p, end, u_q, 3U);
  p = append_str(p, end, ",");
  p = append_float(p, end, g_adc_vbus, 3U);
  p = append_str(p, end, ",");
  p = append_float(p, end, theta, 3U);
  p = append_str(p, end, "\r\n");

  vofa_send_buffer(vofa_tx_buf, (size_t)(p - vofa_tx_buf));
}

void Vofa_Init(void)
{
  if (!Vofa_IsEnabled()) {
    return;
  }

  vofa_rx_len = 0U;
  vofa_line_ready = 0U;
  vofa_tx_busy = 0U;
  HAL_UART_Receive_IT(&huart3, &vofa_rx_byte, 1U);
  vofa_send_text("#vofa ready\r\n");
}

void Vofa_Task(void)
{
  static uint32_t last_telemetry_ms = 0U;

  if (!Vofa_IsEnabled()) {
    return;
  }

  if (vofa_line_ready) {
    __disable_irq();
    strncpy(vofa_cmd_line, vofa_rx_line, sizeof(vofa_cmd_line));
    vofa_cmd_line[sizeof(vofa_cmd_line) - 1U] = '\0';
    vofa_line_ready = 0U;
    __enable_irq();
    handle_command(vofa_cmd_line);
  }

  uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - last_telemetry_ms) >= VOFA_TELEMETRY_PERIOD_MS) {
    last_telemetry_ms = now;
    send_telemetry();
  }
}

void Vofa_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
  if (!Vofa_IsEnabled() || huart->Instance != USART3) {
    return;
  }

  char ch = (char)vofa_rx_byte;
  if (ch == '\r' || ch == '\n') {
    if (vofa_rx_len > 0U && !vofa_line_ready) {
      vofa_rx_line[vofa_rx_len] = '\0';
      vofa_line_ready = 1U;
    }
    vofa_rx_len = 0U;
  } else if (ch == '\b' || ch == 0x7f) {
    if (vofa_rx_len > 0U) {
      vofa_rx_len--;
    }
  } else if (vofa_rx_len < (VOFA_RX_LINE_SIZE - 1U)) {
    vofa_rx_line[vofa_rx_len++] = ch;
  } else {
    vofa_rx_len = 0U;
  }

  HAL_UART_Receive_IT(&huart3, &vofa_rx_byte, 1U);
}

void Vofa_UartTxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3) {
    vofa_tx_busy = 0U;
  }
}

void Vofa_UartErrorCallback(UART_HandleTypeDef *huart)
{
  if (Vofa_IsEnabled() && huart->Instance == USART3) {
    vofa_tx_busy = 0U;
    HAL_UART_Receive_IT(&huart3, &vofa_rx_byte, 1U);
  }
}
