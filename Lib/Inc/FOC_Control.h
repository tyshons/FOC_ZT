//
// Created by tyshon on 2026/3/6.
//

#ifndef FOC_CONTROLLER_H
#define FOC_CONTROLLER_H

#include "adc.h"

#define UDC_VOLTAGE      48.0f
#define VOLTAGE_LIMIT    ((0.9f * UDC_VOLTAGE) / 1.732f)
#define INTEGRAL_LIMIT   2.85f


void Motor_Enable();
void Motor_Disable();
void Control_Loop(void);

#endif //FOC_CONTROLLER_H