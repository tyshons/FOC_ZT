//
// 创建于 2026/5/22。
//

#include "ssi.h"
#include "spi.h"
#include "Experiment_Config.h"
#include <stdint.h>

uint8_t ssi_tx_buf[3] = {0xFF,0xFF,0xFF};
uint8_t ssi_rx_buf[3] = {0};
uint32_t raw_data = 0;
float ssi_pos = 0;
static volatile uint32_t ssi_valid_sample_count = 0U;
static volatile uint32_t ssi_last_valid_tick_ms = 0U;
static volatile uint32_t ssi_transfer_start_error_count = 0U;
static volatile uint32_t ssi_rejected_sample_count = 0U;
static volatile uint32_t ssi_spi_error_count = 0U;
static volatile uint8_t ssi_has_valid_sample = 0U;
static float ssi_verified_angle = 0.0f;
static volatile uint8_t ssi_is_first = 1U;
extern float current_angle_sp;

void ssi_process(void) {
  if (HAL_SPI_TransmitReceive_DMA(&hspi1, ssi_tx_buf, ssi_rx_buf, 3U) != HAL_OK) {
    ssi_transfer_start_error_count++;
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {

    uint32_t raw_data = ((uint32_t)ssi_rx_buf[0] << 16) | ((uint32_t)ssi_rx_buf[1] << 8) | ssi_rx_buf[2];
    raw_data = (raw_data >> 4) & 0x7FFFFUL;

    float current_raw_angle = (float)raw_data * 360.0f / 524288.0f;

    // 位置增量限幅
    uint8_t sample_accepted = 0U;
    if (ssi_is_first) {
      ssi_verified_angle = current_raw_angle;
      ssi_is_first = 0U;
      sample_accepted = 1U;
    } else {
      float delta = current_raw_angle - ssi_verified_angle;
      if (delta > 180.0f)  delta -= 360.0f;
      if (delta < -180.0f) delta += 360.0f;

      // 单次采样的最大允许角度变化由统一配置限制。
      if (delta < SSI_MAX_STEP_DEG && delta > -SSI_MAX_STEP_DEG) {
        ssi_verified_angle = current_raw_angle;
        sample_accepted = 1U;
      }
    }
    current_angle_sp = ssi_verified_angle;
    if (sample_accepted) {
      ssi_last_valid_tick_ms = HAL_GetTick();
      ssi_has_valid_sample = 1U;
      ssi_valid_sample_count++;
    } else {
      ssi_rejected_sample_count++;
    }
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1) {
    ssi_spi_error_count++;
  }
}

uint32_t SSI_GetValidSampleCount(void)
{
  return ssi_valid_sample_count;
}

uint32_t SSI_GetFrameAgeMs(void)
{
  if (ssi_has_valid_sample == 0U) {
    return UINT32_MAX;
  }
  return (uint32_t)(HAL_GetTick() - ssi_last_valid_tick_ms);
}

uint8_t SSI_IsFrameFresh(uint32_t max_age_ms)
{
  return (SSI_GetFrameAgeMs() <= max_age_ms) ? 1U : 0U;
}

uint32_t SSI_GetTransferStartErrorCount(void)
{
  return ssi_transfer_start_error_count;
}

uint32_t SSI_GetRejectedSampleCount(void)
{
  return ssi_rejected_sample_count;
}

uint32_t SSI_GetSpiErrorCount(void)
{
  return ssi_spi_error_count;
}

void SSI_RearmValidation(void)
{
  /* 重新使能后，第一帧用于建立机械角度基准，下一帧开始恢复跳变检查。 */
  ssi_is_first = 1U;
  ssi_has_valid_sample = 0U;
}
