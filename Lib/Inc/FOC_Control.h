//
// Created by tyshon on 2026/3/6.
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


void Motor_Enable();
void Motor_Disable();
void Control_Loop(void);
void Control_Loop_test(void);
void FOC_SetPositionTarget(float target_deg);
void FOC_SetSpeedTarget(float target_rpm);
FOC_ControlMode FOC_GetControlMode(void);
uint8_t FOC_IsPositionTargetValid(void);
uint8_t FOC_GetPowerState(void);

#endif //FOC_CONTROLLER_H
