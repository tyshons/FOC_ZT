//
// 创建于 2026/5/25。
//

#ifndef FOC_ZT_POS_PROCESS_H
#define FOC_ZT_POS_PROCESS_H

void Encoder_Speed_Update(float *speed_out,
                          const float *current_angle_sp,
                          float sample_period_s);
void Encoder_Speed_Reset(float current_angle_deg);
void Get_Electrical_Angle(float *theta_out,const float *current_angle_sp);

#endif
