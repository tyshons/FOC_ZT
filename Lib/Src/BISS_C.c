//
// Created by tyshon on 2026/4/15.
//

#include "BISS_C.h"
#include "spi.h"
#include <stdint.h>
#include <math.h>

#define PI 3.14159265358979323846f

float Mechangle = 0.0f;
int32_t Mechangle1 = 0;
int32_t old_Mechangle1 = 0;
int32_t Elecangle = 0;
float g_electrical_offset = 0.0695f;
float pps = 5.0f;

int32_t diff_raw_b = 0;
int32_t Rev_Count = 0;

volatile int32_t raw_single_turn = 0;
float single_turn = 0.0f;

uint16_t spi_start_fail = 0x01;
uint16_t spi_timout = 0x02;

uint8_t biss_tx_buf[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t biss_rx_buf[8] = {0};

static uint8_t biss_rx_shadow[8] = {0};
uint8_t biss_data_ready = 1;
float Watch_speed = 0.0f;
float Watch_position = 0.0f;

static int find_sync_010(const uint8_t *data, int max_bits)
{
    for (int i = 0; i <= max_bits - 3; i++) {
        uint8_t b0 = (data[i / 8] >> (7 - (i % 8))) & 1;
        uint8_t b1 = (data[(i+1) / 8] >> (7 - ((i+1) % 8))) & 1;
        uint8_t b2 = (data[(i+2) / 8] >> (7 - ((i+2) % 8))) & 1;
        if (b0 == 0 && b1 == 1 && b2 == 0) {
            return i;
        }
    }
    return -1;
}

static uint32_t extract_bits(const uint8_t *data, uint8_t bit_offset, uint8_t nbits)
{
    uint32_t value = 0;
    for (uint8_t i = 0; i < nbits; i++) {
        uint8_t byte_idx = (bit_offset + i) / 8;
        uint8_t bit_idx  = 7 - ((bit_offset + i) % 8);
        if (data[byte_idx] & (1 << bit_idx)) {
            value |= (1UL << (nbits - 1 - i));
        }
    }
    return value;
}

void Biss_start_transfer(void)
{
  if (HAL_SPI_TransmitReceive_DMA(&hspi1, biss_tx_buf, biss_rx_buf, 8) != HAL_OK){

  }

}

void Biss_process(float* pos_out){

  if (!biss_data_ready) return;
  biss_data_ready = 0;
  uint8_t local_buf[8];
  for (int i = 0; i < 8; ++i) local_buf[i] = biss_rx_shadow[i];
  int sync_pos = find_sync_010(local_buf, 64);
  if (sync_pos < 0) {
    return;
  }
  uint32_t data_35bit = extract_bits(local_buf, sync_pos + BISS_SYNC_PATTERN_BITS, BISS_DATA_BITS);
  uint32_t single_turn_23bit = data_35bit & 0x7FFFFFUL;
  uint32_t single_turn_20bit = (single_turn_23bit >> 3) & 0xFFFFFUL;
  raw_single_turn = single_turn_20bit;
  single_turn = (float)single_turn_20bit * 360.0f / ENCODER_COUNTS;
  *pos_out = single_turn;
}

void Encoder_Speed_Update(float *speed_out) {
  static float last_angle = 0.0f;
  static uint32_t last_time = 0;
  static uint8_t is_initialized = 0;
  float current_angle = single_turn ;

  if (!is_initialized) {
    last_angle = current_angle;
    is_initialized = 1;
    if (speed_out) *speed_out = 0.0f;
    return;
  }
  //uint32_t current_time = HAL_GetTick();
  float dt = 0.001f;

  float delta_angle = current_angle - last_angle;

  if (delta_angle > 180.0f)  delta_angle -= 360.0f;
  if (delta_angle < -180.0f) delta_angle += 360.0f;

  float instant_speed = delta_angle / (dt * 6.0f);

  //一阶低通滤波 y(n) = α * x(n) + (1 - α) * y(n-1)
  //current_speed = SPEED_FILTER_ALPHA * instant_speed +
    //                      (1.0f - SPEED_FILTER_ALPHA) * encoder_data.speed;
  *speed_out = instant_speed;
  last_angle = current_angle;
}

void Get_Electrical_Angle(float *theta_out) {
  static float theta_filt = 0.0f;
  float angle_offset = 70.23f;

  float theta_mech = ((single_turn - angle_offset) / 180.0f) * PI;

  float theta_elec = theta_mech * 5.0f;

  theta_elec = fmodf(theta_elec, 2.0f * PI);
  if (theta_elec < 0) {
    theta_elec += 2.0f * PI;
  }

  //theta_filt = 0.8f * theta_elec + 0.2f * theta_filt;

  *theta_out = theta_elec;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1){
    for (int i = 0; i < 8; ++i) {
      biss_rx_shadow[i] = biss_rx_buf[i];
      }
    biss_data_ready = 1;
  }
}
