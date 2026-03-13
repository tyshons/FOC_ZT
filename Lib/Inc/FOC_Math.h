//
// Created by tyshon on 2025/12/6.
//

#ifndef FOC_FOC_MATH_H
#define FOC_FOC_MATH_H

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

void ipark(void);
void clarke(void);
void park(void);
void svpwm(void);

#endif //FOC_FOC_MATH_H