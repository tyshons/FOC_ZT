//
// Created by tyshon on 2026/3/6.
//

#ifndef FOC_CONTROLLER_H
#define FOC_CONTROLLER_H

#include "adc.h"

#define UDC_VOLTAGE      48.0f
#define VOLTAGE_LIMIT    30.0f
#define INTEGRAL_LIMIT   20.0f


void Motor_Enable();
void Motor_Disable();
void Control_Loop(void);
void Control_Loop_test(void);

#endif //FOC_CONTROLLER_H