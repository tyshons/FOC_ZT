//
// 创建于 2026/5/25。
//

#ifndef FOC_ZT_POS_PROCESS_H
#define FOC_ZT_POS_PROCESS_H

void Encoder_Speed_Update_sp(float *speed_out,
                             const float *current_angle_sp,
                             float sample_period_s);
void Encoder_Speed_Update_fy(float *speed_out,
                             const float *current_angle_fy,
                             float sample_period_s);
void Encoder_Speed_Reset_sp(float current_angle_deg);
void Encoder_Speed_Reset_fy(float current_angle_deg);
void Get_Electrical_Angle(float *theta_out,
                          const float *current_angle,
                          float electrical_offset_deg);

#endif
