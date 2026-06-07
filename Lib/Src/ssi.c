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

void ssi_process(float* ssi_out) {
  if (HAL_SPI_TransmitReceive_DMA(&hspi1, ssi_tx_buf, ssi_rx_buf, 3) != HAL_OK){
  }

  raw_data = ((uint32_t)ssi_rx_buf[0] << 16) | ((uint32_t)ssi_rx_buf[1] << 8) | ssi_rx_buf[2];
  raw_data = raw_data>>4 & 0x7FFFFUL;

  ssi_pos = (float)raw_data * 360.0f / 524288.0f;
  *ssi_out = ssi_pos;
}