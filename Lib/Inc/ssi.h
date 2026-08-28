//
// 创建于 2026/5/22。
//

#ifndef FOC_ZT_SSI_H
#define FOC_ZT_SSI_H

#include <stdint.h>

void ssi_process_sp(void);
void ssi_process_fy(void);
uint32_t SSI_GetValidSampleCount_sp(void);
uint32_t SSI_GetValidSampleCount_fy(void);
uint32_t SSI_GetFrameAgeMs_sp(void);
uint32_t SSI_GetFrameAgeMs_fy(void);
uint32_t SSI_GetTransferStartErrorCount_sp(void);
uint32_t SSI_GetTransferStartErrorCount_fy(void);
uint32_t SSI_GetRejectedSampleCount_sp(void);
uint32_t SSI_GetRejectedSampleCount_fy(void);
uint32_t SSI_GetSpiErrorCount_sp(void);
uint32_t SSI_GetSpiErrorCount_fy(void);
uint8_t SSI_IsFrameFresh_sp(uint32_t max_age_ms);
uint8_t SSI_IsFrameFresh_fy(uint32_t max_age_ms);
void SSI_RearmValidation_sp(void);
void SSI_RearmValidation_fy(void);

#endif
