//
// Created by tyshon on 2026/3/9.
//

#ifndef FOC_UPDATA_H
#define FOC_UPDATA_H

typedef struct {
  uint8_t rx_data[5];
} RxData;

void Speed_Update(void);
void Pos_Update(float *OutSpeed);

#endif //FOC_UPDATA_H