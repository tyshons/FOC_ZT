//
// 创建于 2026/3/6。
//

#ifndef FOC_CONTROLLER_H
#define FOC_CONTROLLER_H

#include <stdint.h>

#define UDC_VOLTAGE      48.0f
#define VOLTAGE_LIMIT    30.0f
#define INTEGRAL_LIMIT   20.0f

/* 电角度标定后，把上位机读到的机械角分别写到这里。 */
#define MOTOR_ELECTRICAL_OFFSET_DEG_SP_DEFAULT 96.0f
#define MOTOR_ELECTRICAL_OFFSET_DEG_FY_DEFAULT 142.85f

/* 俯仰轴沿角度增大方向的有效区间：300° -> 0° -> 150°。 */
#define FY_SOFT_LIMIT_MIN_DEG 300.0f
#define FY_SOFT_LIMIT_MAX_DEG 150.0f

typedef enum {
  FOC_CONTROL_MODE_POSITION = 0,
  FOC_CONTROL_MODE_SPEED
} FOC_ControlMode;

typedef enum {
  FOC_AXIS_SP = 0,
  FOC_AXIS_FY = 1
} FOC_Axis;

/* 与原单轴代码一致，只保留原来的三种故障。 */
typedef enum {
  FOC_FAULT_NONE = 0,
  FOC_FAULT_ENCODER_START_TIMEOUT,
  FOC_FAULT_ENCODER_RUNTIME_TIMEOUT,
  FOC_FAULT_PWM_START_FAILED
} FOC_FaultCode;

extern float current_angle_sp;
extern float current_angle_fy;
extern float current_speed_sp;
extern float current_speed_fy;
extern float position_given_sp;
extern float position_given_fy;
extern float speed_given_sp;
extern float speed_given_fy;
extern float id_given_sp;
extern float id_given_fy;
extern float iq_given_sp;
extern float iq_given_fy;
extern float i_d_sp;
extern float i_d_fy;
extern float i_q_sp;
extern float i_q_fy;
extern float electrical_offset_deg_sp;
extern float electrical_offset_deg_fy;

void Motor_Enable(FOC_Axis axis);
void Motor_Disable(FOC_Axis axis);
void Control_Loop(FOC_Axis axis);
void Control_Loop_test(FOC_Axis axis);
void FOC_SetPositionTarget(FOC_Axis axis, float target_deg);
void FOC_SetSpeedTarget(FOC_Axis axis, float target_rpm);
FOC_ControlMode FOC_GetControlMode(FOC_Axis axis);
float FOC_GetSpeedTarget(FOC_Axis axis);
float FOC_GetIdTarget(FOC_Axis axis);
float FOC_GetIqTarget(FOC_Axis axis);
uint8_t FOC_StartElectricalCalibration(FOC_Axis axis, float ud_voltage_v);
void FOC_StopElectricalCalibration(FOC_Axis axis);
uint8_t FOC_CaptureElectricalOffset(FOC_Axis axis);
uint8_t FOC_SetElectricalOffset(FOC_Axis axis, float offset_deg);
float FOC_GetElectricalOffset(FOC_Axis axis);
float FOC_GetElectricalCalibrationUd(FOC_Axis axis);
uint8_t FOC_IsElectricalCalibrationActive(FOC_Axis axis);
uint8_t FOC_IsPositionTargetValid(FOC_Axis axis);
uint8_t FOC_GetPowerState(FOC_Axis axis);
FOC_FaultCode FOC_GetFaultCode(FOC_Axis axis);

#endif
