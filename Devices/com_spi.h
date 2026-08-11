#ifndef COM_SPI_H
#define COM_SPI_H

// SPI 通信驱动 —— CommBase 的子类
// 主模式 SPI, 带 CS 引脚控制
// send: 拉低 CS → SPI 阻塞发送 → 拉高 CS
// bgn:  拉低 CS → 启动单字节中断接收
// read: 返回最近收到的字节

#include "comp_comm.h"
#include "stm32f1xx_hal.h"

// 子类结构体 —— base 必须是第一个成员（保证 &spi.base == &spi）
typedef struct {
  CommBase           base;    // 基类
  SPI_HandleTypeDef  *hspi;   // HAL SPI 句柄
  GPIO_TypeDef       *cs_port; // CS 引脚 GPIO 端口
  uint16_t            cs_pin;  // CS 引脚编号
} Spi;

void spi_init(Spi *me, CommName name, SPI_HandleTypeDef *hspi,
              GPIO_TypeDef *cs_port, uint16_t cs_pin);
void spi_deinit(Spi *me);

#endif
