//
// Created by tyshon on 2026/3/9.
//
#include <stdint.h>
#include "encoder.h"
#include <math.h>
#include <string.h>

#include "usart.h"

#define PI 3.14159265358979323846f
#define POLE_PAIRS 20.0f      // 电机极对数
#define ENC_RES 1048576.0f // 编码器分辨率

volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
const uint8_t tx_buffer[21] = {
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xFE, 0x3A, 0x49, 0x4E
};

volatile uint8_t rx_complete = 1;
volatile uint8_t dma_tx_done = 0;
volatile uint8_t dma_rx_busy = 0;
volatile uint32_t success_count = 0;
volatile uint32_t error_count = 0;

Encoder_Data encoder_data = {0, 0.0f, 0};

void Encoder_Init(void) {
  rx_complete = 1;
  dma_tx_done = 1;
  dma_rx_busy = 0;
}

void Encoder_Position_Request(uint8_t id) {

  if (rx_complete) {
    rx_complete = 0;
    dma_tx_done = 0;
    if (HAL_UART_Transmit_DMA(&huart3, (uint8_t*)tx_buffer, 21) != HAL_OK) {
      error_count++;
    }
  }
}

void Encoder_Position_Receive(void) {
    uint8_t data_for_crc[17];
    int64_t Position = 0;

    if (rx_buffer[0] != ENCODER_ID) {
        encoder_data.is_valid = 0;
        return;
    }

    const uint32_t received_crc = ((uint32_t)rx_buffer[20] << 24) |
                   ((uint32_t)rx_buffer[19] << 16) |
                   ((uint32_t)rx_buffer[18] << 8)  |
                   ((uint32_t)rx_buffer[17] << 0);

    memcpy(data_for_crc, (const void*)rx_buffer, 17);
    const uint32_t calculated_crc = calc_crc32_manual(data_for_crc, 17);

    if (received_crc != calculated_crc) {
        encoder_data.is_valid = 0;
        return;
    }

    uint8_t pos_byte0 = rx_buffer[1]; // 最高位，含符号位
    uint8_t pos_byte1 = rx_buffer[2];
    uint8_t pos_byte2 = rx_buffer[3];
    uint8_t pos_byte3 = rx_buffer[4];
    uint8_t pos_byte4 = rx_buffer[5]; // 最低位

    if ((pos_byte0 >> 7) == 0) {
        Position = (((int64_t)0x000000) << 40) |
                   (((int64_t)pos_byte0) << 32) |
                   (((int64_t)pos_byte1) << 24) |
                   (((int64_t)pos_byte2) << 16) |
                   (((int64_t)pos_byte3) << 8)  |
                   (((int64_t)pos_byte4) << 0);
    } else {
        Position = (((int64_t)0xFFFFFF) << 40) |
                   (((int64_t)pos_byte0) << 32) |
                   (((int64_t)pos_byte1) << 24) |
                   (((int64_t)pos_byte2) << 16) |
                   (((int64_t)pos_byte3) << 8)  |
                   (((int64_t)pos_byte4) << 0);
    }

    int64_t single_turn_pos = Position % 1048576;

    if (single_turn_pos < 0) {
        single_turn_pos += 1048576;
    }

    encoder_data.position = single_turn_pos;
    int64_t turn_count = Position / 1048576;//圈数

    encoder_data.angle = (float)single_turn_pos * 360.0f / 1048576.0f;
    encoder_data.is_valid = 1;

}

void Encoder_Speed_Update(void) {
  static float last_angle = 0.0f;
  static uint32_t last_time = 0;

  uint32_t current_time = HAL_GetTick();
  float dt = (float)(current_time - last_time) / 1000.0f;

  if (dt <= 0.0f) return;

  float current_angle = encoder_data.angle;
  float delta_angle = current_angle - last_angle;

  if (delta_angle > 180.0f)  delta_angle -= 360.0f;
  if (delta_angle < -180.0f) delta_angle += 360.0f;

  float instant_speed = delta_angle / (dt * 6.0f);

  //一阶低通滤波 y(n) = α * x(n) + (1 - α) * y(n-1)
  encoder_data.speed = SPEED_FILTER_ALPHA * instant_speed +
                          (1.0f - SPEED_FILTER_ALPHA) * encoder_data.speed;

  last_angle = current_angle;
  last_time = current_time;
}

void Get_Electrical_Angle(float *theta_out) {
    static float theta_filt = 0.0f;
    float angle_offset = 247.3f;

    float theta_mech = ((encoder_data.angle - angle_offset) / 180.0f) * PI;

    float theta_elec = theta_mech * POLE_PAIRS;

    theta_elec = fmodf(theta_elec, 2.0f * PI);
    if (theta_elec < 0) {
        theta_elec += 2.0f * PI;
    }

    //theta_filt = 0.8f * theta_elec + 0.2f * theta_filt;

    *theta_out = theta_elec;
}


static unsigned int crc32_for_byte(unsigned int r) {
  for(int j = 0; j < 8; ++j)
      r = (r & 1 ? 0 : (unsigned int)0xEDB88320L) ^ r >> 1;
  return r ^ (unsigned int)0xFF000000L;
}

unsigned int calc_crc32_manual(const unsigned char *buf, unsigned int size) {
  static unsigned int table[0x100];
  unsigned int crc = 0;
  unsigned short i;

  if(!*table) {
    for(i = 0; i < 0x100; ++i)
      table[i] = crc32_for_byte(i);
  }

  for(i = 0; i < size; ++i) {
    crc = table[(unsigned char)crc ^ ((unsigned char*)buf)[i]] ^ crc >> 8;
  }
  return crc;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART3) {
    dma_tx_done = 1;
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    dma_rx_busy = 1;
    HAL_UART_Receive_DMA(&huart3, (uint8_t*)rx_buffer, RX_BUFFER_SIZE);
    }
  }


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART3) {
    dma_rx_busy = 0;
    rx_complete = 1;
    success_count++;
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    Encoder_Position_Receive();
  }
}

