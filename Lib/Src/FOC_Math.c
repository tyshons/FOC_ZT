//
// Created by tyshon on 2025/12/6.
//

#include "FOC_Math.h"
#include <math.h>
#include <stdint.h>

#define oneOverRootThree 0.5773503f
#define rootThreeOverTwo 0.8660254f
#define rootThree 1.7320508f
#define u_dc 48.0f
#define ARR 1000

FocVariable foc_math;

void clarke(void)
{

  foc_math.i_alpha = foc_math.i_a;
  foc_math.i_beta = (foc_math.i_a + 2.0f * foc_math.i_b) * oneOverRootThree; //  1/根号3

}

void park(void)
{

  float sine = sinf(foc_math.theta);
  float cosine = cosf(foc_math.theta);
  foc_math.i_d = foc_math.i_alpha * cosine + foc_math.i_beta * sine;
  foc_math.i_q = foc_math.i_beta * cosine - foc_math.i_alpha * sine;

}

void ipark(void)
{

  float sine = sinf(foc_math.theta);
  float cosine = cosf(foc_math.theta);
  foc_math.u_alpha = foc_math.u_d * cosine - foc_math.u_q * sine;
  foc_math.u_beta = foc_math.u_q * cosine + foc_math.u_d * sine;

}

void svpwm(void)
{
  //0表示下桥臂导通，1表示上桥臂导通。a,b,c三个H桥一共8种状态，有6种非零状态，对应6个扇区。
  const float ts = 1.0f;  //  SVPWM的采样周期

  float K = rootThree / u_dc;
  float u1 = foc_math.u_beta;
  float u2 = rootThreeOverTwo * foc_math.u_alpha - 0.5f * foc_math.u_beta;  //  根号3/2
  float u3 = -rootThreeOverTwo * foc_math.u_alpha - 0.5f * foc_math.u_beta;

  uint8_t sector = (u1 > 0.0) + ((u2 > 0.0) << 1) + ((u3 > 0.0) << 2);  // 根据u1、u2和u3的正负情况确定所处的扇区

  // 根据扇区的不同，计算对应的t_a、t_b和t_c的值，表示生成的三相电压的时间
  float Tx = 0.0f;
  float Ty = 0.0f;
  switch (sector) {
    case 1: Tx =  K * u3; Ty =  K * u2; break;
    case 2: Tx =  K * u2; Ty = -K * u1; break;
    case 3: Tx = -K * u3; Ty =  K * u1; break;
    case 4: Tx = -K * u1; Ty =  K * u3; break;
    case 5: Tx =  K * u1; Ty = -K * u2; break;
    case 6: Tx = -K * u2; Ty = -K * u3; break;
    default:Tx = 0.0f; Ty = 0.0f; break;
  }

  if (Tx+Ty > ts) {
    float k_svpwm = ts / (Tx + Ty);
    Tx *= k_svpwm;
    Ty *= k_svpwm;
  }

  float Ta = (ts - Tx - Ty) * 0.25f;
  float Tb = Ta + Tx * 0.5f;
  float Tc = Tb + Ty * 0.5f;

  float Tcmp1=0.0f, Tcmp2=0.0f, Tcmp3=0.0f;
  switch (sector) {
    case 1: Tcmp1 = Tb; Tcmp2 = Ta; Tcmp3 = Tc; break;
    case 2: Tcmp1 = Ta; Tcmp2 = Tc; Tcmp3 = Tb; break;
    case 3: Tcmp1 = Ta; Tcmp2 = Tb; Tcmp3 = Tc; break;
    case 4: Tcmp1 = Tc; Tcmp2 = Tb; Tcmp3 = Ta; break;
    case 5: Tcmp1 = Tc; Tcmp2 = Ta; Tcmp3 = Tb; break;
    case 6: Tcmp1 = Tb; Tcmp2 = Ta; Tcmp3 = Ta; break;
    default:Tcmp1 = 0.5f; Tcmp2 = 0.5f; Tcmp3 = 0.5f; break;
  }
  uint32_t ccrA = (uint32_t)(Tcmp1 * ARR);
  uint32_t ccrB = (uint32_t)(Tcmp2 * ARR);
  uint32_t ccrC = (uint32_t)(Tcmp3 * ARR);

  if (ccrA > ARR) ccrA = ARR;
  if (ccrB > ARR) ccrB = ARR;
  if (ccrC > ARR) ccrC = ARR;
}

