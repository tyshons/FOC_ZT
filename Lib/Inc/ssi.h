//
// Created by tyshon on 2026/5/22.
//

#ifndef FOC_ZT_SSI_H
#define FOC_ZT_SSI_H

#include <stdint.h>

void ssi_process(void);
uint32_t SSI_GetValidSampleCount(void);
void SSI_RearmValidation(void);

#endif //FOC_ZT_SSI_H
