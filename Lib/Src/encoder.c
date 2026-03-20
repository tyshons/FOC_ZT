//
// Created by tyshon on 2026/3/9.
//
#include <stdint.h>
#include "encoder.h"
#include <math.h>
#include <string.h>

#include "usart.h"

#define PI 3.14159265358979323846f
#define POLE_PAIRS 23.0f      // 电机极对数
#define ENC_RES 1048576.0f // 编码器分辨率
#define OFFSET_RAD 0.5236f


volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
const uint8_t tx_buffer[21] = {
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xFE, 0x3A, 0x49, 0x4E
};

volatile uint8_t rx_complete = 1;
volatile uint8_t dma_tx_busy = 0;
volatile uint8_t dma_rx_busy = 0;
volatile uint32_t last_request_time = 0;
volatile uint32_t success_count = 0;
volatile uint32_t error_count = 0;
volatile uint32_t timeout_count = 0;

Encoder_Data encoder_data = {0, 0.0f, 0};

void Encoder_Init(void) {
  rx_complete = 1;
  dma_tx_busy = 0;
  dma_rx_busy = 0;
}

void Encoder_Position_Request(uint8_t id) {
  // if (HAL_GetTick() - last_request_time < 1) {
  //   return;
  // }

  if (dma_tx_busy || dma_rx_busy) {
    return;
  }

  if (!rx_complete && (HAL_GetTick() - last_request_time > 100)) {
    timeout_count++;

    HAL_UART_DMAStop(&huart3);

    rx_complete = 1;
    dma_tx_busy = 0;
    dma_rx_busy = 0;

    __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_ORE);
    __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_NE);
    __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_FE);
  }

  if (!rx_complete) {
    return;
  }

  rx_complete = 0;
  dma_tx_busy = 0;
  dma_rx_busy = 0;
  last_request_time = HAL_GetTick();

  __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_TC);
  __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_RXNE);
  __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_ORE);
  __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_NE);
  __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_FE);
  __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_IDLE);

  dma_tx_busy = 1;
  if (HAL_UART_Transmit_DMA(&huart3, (uint8_t*)tx_buffer, 21) != HAL_OK) {
    dma_tx_busy = 0;
    rx_complete = 1;
    error_count++;
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
    encoder_data.position = single_turn_pos;

    if (single_turn_pos < 0) {
        single_turn_pos += 1048576;
    }

    int64_t turn_count = Position / 1048576;//圈数

    encoder_data.angle = (float)single_turn_pos * 360.0f / 1048576.0f;
    encoder_data.is_valid = 1;

}

void Get_Electrical_Angle(float *theta_out) {

  float theta_mech = (encoder_data.position / ENC_RES) * 2.0f * PI;  //将原始值转换为机械角度 (弧度)
  float theta_elec_raw = theta_mech * POLE_PAIRS;  //乘以极对数得到电角度
  float theta_elec_offset = theta_elec_raw + OFFSET_RAD;//加上零位偏移
  float theta_elec = fmodf(theta_elec_offset, 2.0f * PI);//取模

  if (theta_elec < 0.0f) {
    theta_elec += 2.0f * PI;
  }
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
    dma_tx_busy = 0;
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

    dma_rx_busy = 1;
    if (HAL_UART_Receive_DMA(&huart3, (uint8_t*)rx_buffer, RX_BUFFER_SIZE) != HAL_OK) {
      dma_rx_busy = 0;
      rx_complete = 1;
      error_count++;
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART3) {
    dma_rx_busy = 0;
    rx_complete = 1;
    success_count++;

    Encoder_Position_Receive();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART3) {
    error_count++;

    dma_tx_busy = 0;
    dma_rx_busy = 0;
    rx_complete = 1;

    __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_ORE);
    __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_NE);
    __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_FE);

    HAL_UART_AbortReceive(&huart3);
  }
}

void UART3_IDLE_Callback(void) {
  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET) {
    __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_IDLE);

    if (dma_rx_busy && !rx_complete) {
      dma_rx_busy = 0;
      rx_complete = 1;

      HAL_UART_DMAStop(&huart3);

      Encoder_Position_Receive();
    }
  }
}
