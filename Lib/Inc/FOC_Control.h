//
// 创建于 2026/3/6。
//

#ifndef FOC_CONTROLLER_H
#define FOC_CONTROLLER_H

#include "adc.h"

#define UDC_VOLTAGE      48.0f
#define VOLTAGE_LIMIT    30.0f
#define INTEGRAL_LIMIT   20.0f

typedef enum {
  FOC_CONTROL_MODE_POSITION = 0,
  FOC_CONTROL_MODE_SPEED
} FOC_ControlMode;

typedef enum {
  FOC_FAULT_NONE = 0,
  FOC_FAULT_ENCODER_START_TIMEOUT,
  FOC_FAULT_ENCODER_RUNTIME_TIMEOUT,
  FOC_FAULT_PWM_START_FAILED
} FOC_FaultCode;

void Motor_Enable(void);
void Motor_Disable(void);
void Control_Loop(void);
void Control_Loop_test(void);
void FOC_SetPositionTarget(float target_deg);
void FOC_SetSpeedTarget(float target_rpm);
FOC_ControlMode FOC_GetControlMode(void);
float FOC_GetSpeedTarget(void);
float FOC_GetIqTarget(void);
uint8_t FOC_IsPositionTargetValid(void);
uint8_t FOC_GetPowerState(void);
FOC_FaultCode FOC_GetFaultCode(void);

#endif
