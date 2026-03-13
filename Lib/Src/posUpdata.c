//
// Created by tyshon on 2026/3/9.
//
#include <stdint.h>
#include "posUpdata.h"

RxData g_receive_message;

void Pos_Update(float *OutSpeed) {
  int64_t Position;
  if((g_receive_message.rx_data[0]>>7) == 0)
  {
    Position = (((int64_t)0x000000)<<40) |\
    (((int64_t)g_receive_message.rx_data[0])<<32) | \
    (((int64_t)g_receive_message.rx_data[1])<<24) | \
    (((int64_t)g_receive_message.rx_data[2])<<16) | \
    (((int64_t)g_receive_message.rx_data[3])<<8) | \
    (((int64_t)g_receive_message.rx_data[4])<<0);
  }
  else
  {
    Position = (((int64_t)0xFFFFFF)<<40) |\
    (((int64_t)g_receive_message.rx_data[0])<<32) | \
    (((int64_t)g_receive_message.rx_data[1])<<24) | \
    (((int64_t)g_receive_message.rx_data[2])<<16) | \
    (((int64_t)g_receive_message.rx_data[3])<<8) | \
    (((int64_t)g_receive_message.rx_data[4])<<0);
  }
}