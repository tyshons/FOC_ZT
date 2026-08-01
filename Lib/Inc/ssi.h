//
// 创建于 2026/5/22。
//

#ifndef FOC_ZT_SSI_H
#define FOC_ZT_SSI_H

#include <stdint.h>

void ssi_process(void);
uint32_t SSI_GetValidSampleCount(void);
uint32_t SSI_GetFrameAgeMs(void);
uint32_t SSI_GetTransferStartErrorCount(void);
uint32_t SSI_GetRejectedSampleCount(void);
uint32_t SSI_GetSpiErrorCount(void);
uint8_t SSI_IsFrameFresh(uint32_t max_age_ms);
void SSI_RearmValidation(void);

#endif
