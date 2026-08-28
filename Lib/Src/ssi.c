#include "ssi.h"

#include "Experiment_Config.h"
#include "FOC_Control.h"
#include "spi.h"

#include <stdint.h>

typedef struct {
  uint8_t tx_buf[3];
  uint8_t rx_buf[3];
  volatile uint32_t valid_sample_count;
  volatile uint32_t last_valid_tick_ms;
  volatile uint32_t transfer_start_error_count;
  volatile uint32_t rejected_sample_count;
  volatile uint32_t spi_error_count;
  volatile uint8_t has_valid_sample;
  float verified_angle;
  volatile uint8_t is_first;
} SSI_State;

static SSI_State ssi_state_sp = {
    .tx_buf = {0xFFU, 0xFFU, 0xFFU},
    .is_first = 1U,
};
static SSI_State ssi_state_fy = {
    .tx_buf = {0xFFU, 0xFFU, 0xFFU},
    .is_first = 1U,
};

static void ssi_process_axis(SSI_State *state, SPI_HandleTypeDef *hspi)
{
  if (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY) {
    return;
  }
  if (HAL_SPI_TransmitReceive_DMA(hspi, state->tx_buf, state->rx_buf, 3U) !=
      HAL_OK) {
    state->transfer_start_error_count++;
  }
}

void ssi_process_sp(void)
{
  ssi_process_axis(&ssi_state_sp, &hspi1);
}

void ssi_process_fy(void)
{
  ssi_process_axis(&ssi_state_fy, &hspi4);
}

static void ssi_transfer_complete(SSI_State *state, float *current_angle)
{
  uint32_t raw_data = ((uint32_t)state->rx_buf[0] << 16) |
                      ((uint32_t)state->rx_buf[1] << 8) |
                      (uint32_t)state->rx_buf[2];
  raw_data = (raw_data >> 4) & 0x7FFFFUL;
  const float current_raw_angle =
      (float)raw_data * 360.0f / SSI_ENCODER_COUNTS_PER_REV;

  uint8_t sample_accepted = 0U;
  if (state->is_first != 0U) {
    state->verified_angle = current_raw_angle;
    state->is_first = 0U;
    sample_accepted = 1U;
  } else {
    float delta = current_raw_angle - state->verified_angle;
    if (delta > 180.0f) {
      delta -= 360.0f;
    }
    if (delta < -180.0f) {
      delta += 360.0f;
    }
    if ((delta < SSI_MAX_STEP_DEG) && (delta > -SSI_MAX_STEP_DEG)) {
      state->verified_angle = current_raw_angle;
      sample_accepted = 1U;
    }
  }

  *current_angle = state->verified_angle;
  if (sample_accepted != 0U) {
    state->last_valid_tick_ms = HAL_GetTick();
    state->has_valid_sample = 1U;
    state->valid_sample_count++;
  } else {
    state->rejected_sample_count++;
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1) {
    ssi_transfer_complete(&ssi_state_sp, &current_angle_sp);
  } else if (hspi->Instance == SPI4) {
    ssi_transfer_complete(&ssi_state_fy, &current_angle_fy);
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1) {
    ssi_state_sp.spi_error_count++;
  } else if (hspi->Instance == SPI4) {
    ssi_state_fy.spi_error_count++;
  }
}

static uint32_t ssi_frame_age_ms(const SSI_State *state)
{
  if (state->has_valid_sample == 0U) {
    return UINT32_MAX;
  }
  return (uint32_t)(HAL_GetTick() - state->last_valid_tick_ms);
}

uint32_t SSI_GetValidSampleCount_sp(void)
{
  return ssi_state_sp.valid_sample_count;
}

uint32_t SSI_GetValidSampleCount_fy(void)
{
  return ssi_state_fy.valid_sample_count;
}

uint32_t SSI_GetFrameAgeMs_sp(void)
{
  return ssi_frame_age_ms(&ssi_state_sp);
}

uint32_t SSI_GetFrameAgeMs_fy(void)
{
  return ssi_frame_age_ms(&ssi_state_fy);
}

uint8_t SSI_IsFrameFresh_sp(uint32_t max_age_ms)
{
  return (SSI_GetFrameAgeMs_sp() <= max_age_ms) ? 1U : 0U;
}

uint8_t SSI_IsFrameFresh_fy(uint32_t max_age_ms)
{
  return (SSI_GetFrameAgeMs_fy() <= max_age_ms) ? 1U : 0U;
}

uint32_t SSI_GetTransferStartErrorCount_sp(void)
{
  return ssi_state_sp.transfer_start_error_count;
}

uint32_t SSI_GetTransferStartErrorCount_fy(void)
{
  return ssi_state_fy.transfer_start_error_count;
}

uint32_t SSI_GetRejectedSampleCount_sp(void)
{
  return ssi_state_sp.rejected_sample_count;
}

uint32_t SSI_GetRejectedSampleCount_fy(void)
{
  return ssi_state_fy.rejected_sample_count;
}

uint32_t SSI_GetSpiErrorCount_sp(void)
{
  return ssi_state_sp.spi_error_count;
}

uint32_t SSI_GetSpiErrorCount_fy(void)
{
  return ssi_state_fy.spi_error_count;
}

static void ssi_rearm_validation(SSI_State *state)
{
  state->is_first = 1U;
  state->has_valid_sample = 0U;
}

void SSI_RearmValidation_sp(void)
{
  ssi_rearm_validation(&ssi_state_sp);
}

void SSI_RearmValidation_fy(void)
{
  ssi_rearm_validation(&ssi_state_fy);
}
