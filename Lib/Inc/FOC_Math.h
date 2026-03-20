//
// Created by tyshon on 2025/12/6.
//

#ifndef FOC_FOC_MATH_H
#define FOC_FOC_MATH_H

#include <stdint.h>

typedef struct
{
  float u_d;
  float u_q;
  float theta; // 电角度

  float u_alpha;
  float u_beta;

  float t_a; // 电压矢量
  float t_b;
  float t_c;

  float i_a;
  float i_b;
  float i_c;

  float i_alpha;
  float i_beta;

  float i_d;
  float i_q;
} FocVariable;

extern FocVariable foc_math;

void clarke_transform(float i_a, float i_b, float i_c, float* i_alpha, float* i_beta);
void park_transform(float i_alpha, float i_beta, float theta, float* i_d, float* i_q);
void ipark_transform(float u_d, float u_q, float theta, float* u_alpha, float* u_beta);
void svpwm_generate(float u_alpha, float u_beta, float u_dc, uint32_t* CCRA, uint32_t* CCRB, uint32_t* CCRC);

#endif //FOC_FOC_MATH_H