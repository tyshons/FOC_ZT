//
// Created by tyshon on 2026/5/22.
//

#include "ssi.h"
#include "spi.h"
#include <stdint.h>

uint8_t ssi_tx_buf[3] = {0xFF,0xFF,0xFF};
uint8_t ssi_rx_buf[3] = {0};
uint32_t raw_data = 0;
float ssi_pos = 0;
extern float current_angle_sp;

void ssi_process(void) {
  if (HAL_SPI_TransmitReceive_DMA(&hspi1, ssi_tx_buf, ssi_rx_buf, 3) != HAL_OK){
  }

  // raw_data = ((uint32_t)ssi_rx_buf[0] << 16) | ((uint32_t)ssi_rx_buf[1] << 8) | ssi_rx_buf[2];
  // raw_data = raw_data>>4 & 0x7FFFFUL;
  //
  // ssi_pos = (float)raw_data * 360.0f / 524288.0f;
  // *ssi_out = ssi_pos;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {

    uint32_t raw_data = ((uint32_t)ssi_rx_buf[0] << 16) | ((uint32_t)ssi_rx_buf[1] << 8) | ssi_rx_buf[2];
    raw_data = (raw_data >> 4) & 0x7FFFFUL;

    float current_raw_angle = (float)raw_data * 360.0f / 524288.0f;

    // 位置增量限幅
    static float verified_angle = 0.0f;
    static uint8_t is_first = 1;

    if (is_first) {
      verified_angle = current_raw_angle;
      is_first = 0;
    } else {
      float delta = current_raw_angle - verified_angle;
      if (delta > 180.0f)  delta -= 360.0f;
      if (delta < -180.0f) delta += 360.0f;

      // 一个周期内最大允许变化 15 度（可调）
      if (delta < 15.0f && delta > -15.0f) {
        verified_angle = current_raw_angle;
      }
    }
    current_angle_sp = verified_angle;
  }
}
