// BSP I2C 硬件抽象接口 — 平台无关中断事务收发 (不透明句柄, BSP 之上零 HAL)
//
// 定位: 供 Devices/comm com_i2c 用"中断事务 + 完成回调"替代阻塞 HAL_I2C_Mem_Write/Read.
//   I2C 是显式寄存器寻址事务 (非流式), 不套 comp_dma_rx/tx 循环 model;
//   用"一次一个事务 + 完成回调"达到非阻塞 (对标 libxr STM32I2C 的中断事务模型).
//
// 事务语义:
//   bsp_i2c_mem_write_async / bsp_i2c_mem_read_async 启动一次中断事务 (非阻塞, 立即返回);
//   完成/失败时调 BspI2cCb(me, ec, ctx) — 同一时刻只允许一个在途事务.
//   平台内部: STM32 用 HAL_I2C_Mem_Write_IT / Mem_Read_IT; C2000 用 I2C 模块中断.
//
// 完成中断: 应用把 I2C_EV/ER IRQ 路由进 HAL_I2C_EV_IRQHandler / ER_IRQHandler.
//   BSP 以 __weak 定义 HAL_I2C_MemTxCpltCallback / MemRxCpltCallback / ErrorCallback,
//   内部转调 bsp_i2c_on_done → 用户 cb. 应用若自定义这些回调, 需自行调 bsp_i2c_on_done.

#ifndef BSP_I2C_H
#define BSP_I2C_H

#include <stdint.h>
#include <stdbool.h>

// 不透明 I2C 句柄 (平台: STM32 = I2C_HandleTypeDef*; C2000 = I2CA/B 基址)
typedef void BspI2c;

// 事务完成回调 (中断上下文, 非阻塞; ec: 0=OK 非0=ErrorCode)
typedef void (*BspI2cCb)(BspI2c *me, int ec, void *ctx);

// 绑定平台 I2C 句柄 → 不透明 BspI2c* (NULL=参数错误)
BspI2c *bsp_i2c_bind(void *h);

// 解绑 (中止在途事务 + 释放引用)
void bsp_i2c_unbind(BspI2c *me);

// 启动中断写寄存器事务: 向 dev_addr 的 reg 写 len 字节; 完成回调 cb(ctx)
//   返回 false = 参数错 / 有在途事务 (busy). 非阻塞.
bool bsp_i2c_mem_write_async(BspI2c *me, uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint16_t len, BspI2cCb cb,
                             void *ctx);

// 启动中断读寄存器事务: 从 dev_addr 的 reg 读 len 字节到 buf; 完成回调 cb(ctx)
bool bsp_i2c_mem_read_async(BspI2c *me, uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint16_t len, BspI2cCb cb,
                            void *ctx);

// 平台完成钩子: HAL I2C 完成/错误回调里调用 (结束在途事务并调用户 cb)
void bsp_i2c_on_done(BspI2c *me, int ec);

#endif  // BSP_I2C_H
