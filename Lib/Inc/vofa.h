#ifndef FOC_ZT_VOFA_H
#define FOC_ZT_VOFA_H

#include "main.h"
#include <stdint.h>

#define VOFA_ENABLE 0

void Vofa_Init(void);
void Vofa_Task(void);
uint8_t Vofa_IsEnabled(void);
void Vofa_UartRxCpltCallback(UART_HandleTypeDef *huart);
void Vofa_UartTxCpltCallback(UART_HandleTypeDef *huart);
void Vofa_UartErrorCallback(UART_HandleTypeDef *huart);

#endif
