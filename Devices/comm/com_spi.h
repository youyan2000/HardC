// SPI 传输类 —— CommBase 子类 (Devices/comm, 中断事务接入版)
//
// 语义: 全双工 + 寄存器访问. CS 由本类经 bsp_gpio 自动管理 (HAL-free 化 CS),
//   传输经 BSP/bsp_spi.h 中断事务 (非阻塞, 完成回调收 ec), BSP 之上零 HAL.
//
// 数据面 (BspSpi 句柄 + CS 引脚) 在子类结构体, 不进 CommBase 虚表;
// 完成方式在调用时经 IoCompletion 绑定 (异步=IO_ASYNC_FLAG; IO_SYNC 时轮询完成, 仅 MAIN).

#ifndef COM_SPI_H
#define COM_SPI_H

#include "comp_comm.h"
#include "comp_io.h"
#include "comp_error_code.h"
#include "bsp_gpio.h"
#include "bsp_spi.h"

// 配置 POD
typedef struct {
  BspSpi *port;   // 不透明 SPI 后端 (bsp_spi_bind 结果)
  BspGpioPin cs;  // CS 引脚 (bsp_gpio 管理)
} SpiConfig;

// SPI 类 — 全双工 + 寄存器访问, CS 自动管理
typedef struct {
  CommBase base;  // 基类 (必须第一成员)
  BspSpi *port;   // 不透明 SPI 后端
  BspGpioPin cs;  // CS 引脚
  IoCompletion completion;
  volatile int tx_busy;  // 在途中断事务 (bsp_spi 单事务, ISR 清)
  volatile int last_ec;  // 最近完成事务错误码 (0=OK)
  uint8_t cs_active;     // CS 是否被本事务拉低 (完成回调里释放, 异步安全)
} Spi;

ErrorCode spi_init(Spi *me, const SpiConfig *cfg);
ErrorCode spi_set_config(Spi *me, const SpiConfig *cfg);
void spi_deinit(Spi *me);

// CS 拉低→发送→拉高 (非阻塞; IO_SYNC 时轮询完成, 仅 MAIN)
ErrorCode spi_write(Spi *me, CommConstData data, IoCompletion comp);

// CS 拉低→接收→拉高 (非阻塞)
ErrorCode spi_read(Spi *me, CommData data, IoCompletion comp);

// 全双工: 同时收发 (非阻塞)
ErrorCode spi_transfer(Spi *me, CommConstData tx, CommData rx, IoCompletion comp);

// 寄存器读: 发 reg 地址 → 收 len 字节 (非阻塞)
ErrorCode spi_read_reg(Spi *me, uint8_t reg, uint8_t *dat, uint16_t len, IoCompletion comp);

#endif  // COM_SPI_H
