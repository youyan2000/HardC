#ifndef COM_UART_H
#define COM_UART_H

// USART 驱动 —— CommBase 的子类
// 使用 STM32 HAL 的 UART 外设，以中断方式逐字节接收
// read: 返回 cur (最近收到的字节)

#include "comp_comm.h"
#include "stm32f1xx_hal.h"

// 子类结构体 —— base 必须是第一个成员（保证 &uart.base == &uart）
typedef struct {
  CommBase           base;   // 基类
  UART_HandleTypeDef *huart; // HAL UART 句柄
} Uart;

void uart_init(Uart *me, CommName name, UART_HandleTypeDef *huart, uint8_t *rxb);
void uart_deinit(Uart *me);

#endif
