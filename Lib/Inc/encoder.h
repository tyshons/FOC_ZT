//
// Created by tyshon on 2026/3/9.
//

#ifndef FOC_UPDATA_H
#define FOC_UPDATA_H

#define ENCODER_ID 1
#define RX_BUFFER_SIZE 21

extern volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
extern volatile uint8_t rx_index;
extern volatile uint8_t rx_complete;

typedef struct {
  int64_t position;       // 原始40位位置数据
  float angle;            // 计算后的角度 (0-360)
  uint8_t is_valid;       // 数据有效标志
} Encoder_Data;

extern Encoder_Data encoder_data;

void Encoder_Init(void);
void Encoder_Position_Request(uint8_t id);
void Encoder_Position_Receive(void);
void Speed_Update(void);
void Pos_Update(float *OutSpeed);
void Get_Electrical_Angle(float *theta_out);
unsigned int calc_crc32_manual(const unsigned char *buf, unsigned int size);

#endif //FOC_UPDATA_H