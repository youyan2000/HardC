// I2C 传输类 —— CommBase 子类 (Devices/comm, 仅硬件 I2C, 中断事务接入版)
//
// SW bit-bang 已删除 (用户裁定: SW I2C 用不了 DMA 就必须阻塞 → HardC 不支持).
// 仅硬件 I2C: 寄存器寻址事务经 BSP/bsp_i2c.h 中断事务 (非阻塞, 完成回调收 ec).
//   iic_write_reg/read_reg 保持签名; comp=IO_SYNC 时轮询完成 (仅 MAIN).
// BSP 之上零 HAL: 结构持 BspI2c* 不透明句柄, 不 include bsp_stm32_hal.h.

#ifndef COM_I2C_H
#define COM_I2C_H

#include "comp_comm.h"
#include "comp_io.h"
#include "comp_error_code.h"
#include "bsp_i2c.h"

typedef enum { IIC_HW = 0 } IicMode;  // 仅硬件 I2C (SW 已删)

// I2C 类 — 仅硬件 I2C (中断事务, 不透明 BSP 句柄)
typedef struct {
  CommBase base;     // 基类 (必须第一成员)
  BspI2c *port;      // 不透明 I2C 后端 (bsp_i2c_bind 结果)
  uint8_t dev_addr;  // 7-bit 从机地址
  IicMode mode;      // 当前模式
  IoCompletion completion;
  volatile int tx_busy;  // 在途中断事务 (bsp_i2c 单事务, ISR 清)
  volatile int last_ec;  // 最近完成事务错误码 (0=OK)
} Iic;

// 硬件 I2C 初始化 (port = bsp_i2c_bind 结果; NULL → ERR_ARG)
ErrorCode iic_init_hw(Iic *me, BspI2c *port, uint8_t dev_addr);

void iic_deinit(Iic *me);

// 写寄存器地址 → 写 len 字节 (非阻塞; IO_SYNC 时轮询完成, 仅 MAIN)
ErrorCode iic_write_reg(Iic *me, uint8_t reg, const uint8_t *dat, uint16_t len, IoCompletion comp);

// 写寄存器地址 → 读 len 字节 (非阻塞)
ErrorCode iic_read_reg(Iic *me, uint8_t reg, uint8_t *dat, uint16_t len, IoCompletion comp);

#endif  // COM_I2C_H
