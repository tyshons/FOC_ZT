#ifndef FOC_ZT_TRACKING_H
#define FOC_ZT_TRACKING_H

#include "FOC_Control.h"
#include "PID_Control.h"
#include "main.h"

#include <stdint.h>

#define TRACKING_INPUT_TIMEOUT_MS 150U
#define TRACKING_SPEED_LIMIT_RPM  5.0f

extern volatile uint8_t tracking_enabled;
extern volatile uint8_t tracking_target_valid;
extern volatile float tracking_error_sp;
extern volatile float tracking_error_fy;
extern volatile float tracking_speed_command_sp;
extern volatile float tracking_speed_command_fy;
extern volatile uint32_t tracking_last_update_ms;
extern volatile uint32_t tracking_rx_byte_count;
extern volatile uint32_t tracking_rx_dropped_byte_count;
extern volatile uint32_t tracking_input_frame_count;

void Tracking_Init(void);
void Tracking_Task(void);
void Tracking_SetEnabled(uint8_t enabled);
uint8_t Tracking_IsEnabled(void);
void Tracking_InputUpdate(float error_sp, float error_fy, uint8_t target_valid);
PID_TypeDef *Tracking_GetPid(FOC_Axis axis);

void Tracking_UartRxCpltCallback(UART_HandleTypeDef *huart);
void Tracking_UartRxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
void Tracking_UartErrorCallback(UART_HandleTypeDef *huart);

#endif
