//
// Created by tyshon on 2026/3/6.
//

#ifndef FOC_CONTROLLER_H
#define FOC_CONTROLLER_H

typedef struct {
  float in_prev[3];
  float out_prev[3];
} pid_state_t;

extern pid_state_t pos_pid_sp;
extern pid_state_t spd_pid_sp;
extern pid_state_t iq_pid_sp;

float position_pid(float error, pid_state_t* state, const float num[4], const float den[4]);
float speed_pid(float speed_error, pid_state_t* state, const float num[4], const float den[4]);
float iq_pid(float iq_error, pid_state_t* state, const float num[4], const float den[4]);

#endif //FOC_CONTROLLER_H