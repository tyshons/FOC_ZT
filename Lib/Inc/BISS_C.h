//
// Created by tyshon on 2026/4/15.
//

#ifndef FOC_ZT_BISS_C_H
#define FOC_ZT_BISS_C_H

#define BISS_SYNC_PATTERN_BITS      3   // "010"
#define BISS_DATA_BITS             35   // ×ÜÊý¾ÝÎ»£ºMT + ST
#define BISS_MULTI_TURN_BITS       12
#define BISS_SPI_TIMEOUT_MS        10   // ms
#define BISS_DEBUG                  0

/*ËÙ¶È²ÎÊý*/
#define  ENCODER_COUNTS  1048576           		//»úÐµÒ»È¦·Ö¶È 2^20
#define  S_SPEED         0.05722045f          //ËÙ¶È×ª»»ÏµÊý£¨µ¥Î»£ºrpm£© £¨60/0.001£©/1048576
#define  M_Mech          0.000343322f 	      //»úÐµ½Ç¶ÈÏµÊý
#define  SPD_CAL_DIV     20                   //²ÉÑùÆµÂÊ

void Biss_process(float *pos_out);
void Encoder_Speed_Update(float *speed_out);
void Biss_start_transfer(void);
void Get_Electrical_Angle(float *theta_out);

#endif //FOC_ZT_BISS_C_H
