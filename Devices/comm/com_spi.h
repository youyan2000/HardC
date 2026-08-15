// SPI 传输类 —— CommBase 子类 (Devices/comm, 阶段1 重构)
//
// 语义: 全双工 + 寄存器访问. CS 由本类经 bsp_gpio 自动管理 (HAL-free 化 CS),
//   传输用阻塞 HAL_SPI_Transmit/Receive/TransmitReceive (事务短且全在 MAIN, 100ms 超时可接受).
//
// 数据面 (HAL 句柄 + CS 引脚) 在子类结构体, 不进 CommBase 虚表;
// 完成方式在调用时经 IoCompletion 绑定 (SPI 阻塞语义下 IO_SYNC 天然成立).

#ifndef COM_SPI_H
#define COM_SPI_H

#include "comp_comm.h"
#include "comp_io.h"
#include "comp_error_code.h"
#include "bsp_gpio.h"
#include "bsp_stm32_hal.h"

// 配置 POD
typedef struct {
  SPI_HandleTypeDef *hspi;  // App 注入 HAL 句柄
  BspGpioPin cs;            // CS 引脚 (bsp_gpio 管理)
} SpiConfig;

// SPI 类 — 全双工 + 寄存器访问, CS 自动管理
typedef struct {
  CommBase base;            // 基类 (必须第一成员)
  SPI_HandleTypeDef *hspi;  // HAL SPI 句柄
  BspGpioPin cs;            // CS 引脚
  IoCompletion completion;
} Spi;

void spi_init(Spi *me, const SpiConfig *cfg);
void spi_set_config(Spi *me, const SpiConfig *cfg);
void spi_deinit(Spi *me);

// CS 拉低→发送→拉高
ErrorCode spi_write(Spi *me, CommConstData data, IoCompletion comp);

// CS 拉低→接收→拉高
ErrorCode spi_read(Spi *me, CommData data, IoCompletion comp);

// 全双工: 同时收发
ErrorCode spi_transfer(Spi *me, CommConstData tx, CommData rx, IoCompletion comp);

// 寄存器读: 发 reg 地址 → 收 len 字节
ErrorCode spi_read_reg(Spi *me, uint8_t reg, uint8_t *dat, uint16_t len, IoCompletion comp);

#endif  // COM_SPI_H
