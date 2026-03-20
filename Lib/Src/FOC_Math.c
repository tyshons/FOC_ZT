//
// Created by tyshon on 2025/12/6.
//

#include "FOC_Math.h"
#include <math.h>

#define oneOverRootThree 0.5773503f
#define rootThreeOverTwo 0.8660254f
#define rootThree 1.7320508f
#define ARR 4250

FocVariable foc_math;

void clarke_transform(float i_a, float i_b, float ic, float* i_alpha, float* i_beta)
{

  *i_alpha = i_a;
  *i_beta = (i_a + 2.0f * i_b) * oneOverRootThree; //  1/根号3

}

void park_transform(float i_alpha, float i_beta, float theta, float* i_d, float* i_q)
{

  float sine = sinf(theta);
  float cosine = cosf(theta);
  *i_d = i_alpha * cosine + i_beta * sine;
  *i_q = -i_alpha * sine + i_beta * cosine;

}

void ipark_transform(float u_d, float u_q, float theta, float* u_alpha, float* u_beta)
{

  float sine = sinf(theta);
  float cosine = cosf(theta);
  *u_alpha = u_d * cosine - u_q * sine;
  *u_beta = u_d * sine + u_q * cosine;

}

void svpwm_generate(float u_alpha, float u_beta, float u_dc, uint32_t* CCRA, uint32_t* CCRB, uint32_t* CCRC)
{
  //0表示下桥臂导通，1表示上桥臂导通。a,b,c三个H桥一共8种状态，有6种非零状态，对应6个扇区。
  const float ts = 1.0f;  //  SVPWM的采样周期

  float K = rootThree / u_dc;
  float u1 = u_beta;
  float u2 = rootThreeOverTwo * u_alpha - 0.5f * u_beta;  //  根号3/2
  float u3 = -rootThreeOverTwo * u_alpha - 0.5f * u_beta;

  uint8_t sector = (u1 > 0.0) + ((u2 > 0.0) << 1) + ((u3 > 0.0) << 2);  // 根据u1、u2和u3的正负情况确定所处的扇区

  //归一化
  float X = u_beta * K;
  float Y = (rootThreeOverTwo * u_alpha + 0.5f * u_beta)* K;
  float Z = ( -rootThreeOverTwo * u_alpha + 0.5f * u_beta)* K;

  // 根据扇区的不同，计算对应的t_a、t_b和t_c的值，表示生成的三相电压的时间
  float T1 = 0.0f, T2=0.0f;
  switch (sector) {
    case 1: T1 =  Y; T2 =  Z; break;
    case 2: T1 =  Y; T2 = -X; break;
    case 3: T1 = -Z; T2 =  X; break;
    case 4: T1 =  Z; T2 =  -X; break;
    case 5: T1 =  X; T2 = -Y; break;
    case 6: T1 = -Y; T2 = -Z; break;
    default:T1 = 0.0f; T2 = 0.0f; break;
  }

  if (T1+T2 > ts) {
    float k_svpwm = ts / (T1 + T2);
    T1 *= k_svpwm;
    T2 *= k_svpwm;
  }

  float Tp1 = (ts - T1 - T2) * 0.25f;
  float Tp2 = Tp1 + T1 * 0.5f;
  float Tp3 = Tp2 + T2 * 0.5f;

  float Tcmp1=0.0f, Tcmp2=0.0f, Tcmp3=0.0f;
  switch (sector) {
    case 1: Tcmp1 = Tp2; Tcmp2 = Tp1; Tcmp3 = Tp3; break;
    case 2: Tcmp1 = Tp1; Tcmp2 = Tp3; Tcmp3 = Tp2; break;
    case 3: Tcmp1 = Tp1; Tcmp2 = Tp2; Tcmp3 = Tp3; break;
    case 4: Tcmp1 = Tp3; Tcmp2 = Tp2; Tcmp3 = Tp1; break;
    case 5: Tcmp1 = Tp3; Tcmp2 = Tp1; Tcmp3 = Tp2; break;
    case 6: Tcmp1 = Tp2; Tcmp2 = Tp3; Tcmp3 = Tp1; break;
    default:Tcmp1 = 0.5f; Tcmp2 = 0.5f; Tcmp3 = 0.5f; break;
  }
  uint32_t ccrA = (uint32_t)(Tcmp1 * ARR);
  uint32_t ccrB = (uint32_t)(Tcmp2 * ARR);
  uint32_t ccrC = (uint32_t)(Tcmp3 * ARR);

  if (ccrA > ARR) ccrA = ARR;
  if (ccrB > ARR) ccrB = ARR;
  if (ccrC > ARR) ccrC = ARR;

  *CCRA = ccrA;
  *CCRB = ccrB;
  *CCRC = ccrC;

}

