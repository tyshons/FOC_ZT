#ifndef FOC_ZT_TURNTABLE_COMM_H
#define FOC_ZT_TURNTABLE_COMM_H

#include "main.h"
#include <stdint.h>

#define TURNTABLE_COMM_ENABLE 1

extern volatile uint32_t tt_status_tx_count;
extern volatile uint32_t tt_status_tx_error_count;

void Turntable_Comm_Init(void);
void Turntable_Comm_Task(void);
uint8_t Turntable_Comm_IsEnabled(void);
void Turntable_Comm_UartRxCpltCallback(UART_HandleTypeDef *huart);
void Turntable_Comm_UartTxCpltCallback(UART_HandleTypeDef *huart);
void Turntable_Comm_UartErrorCallback(UART_HandleTypeDef *huart);

#endif
