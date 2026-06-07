//
// Created by tyshon on 2026/4/15.
//

#ifndef FOC_ZT_BISS_C_H
#define FOC_ZT_BISS_C_H

#define BISS_SYNC_PATTERN_BITS      3   // Biss—c包头 010
#define BISS_DATA_BITS             35   //

#define  ENCODER_COUNTS  1048576           		//20位

void Biss_process(float *pos_out);
//void Encoder_Speed_Update(float *speed_out);
void Biss_start_transfer(void);
//void Get_Electrical_Angle(float *theta_out);

#endif //FOC_ZT_BISS_C_H
