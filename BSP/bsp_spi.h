// BSP SPI 硬件抽象接口 — 平台无关中断事务收发 (不透明句柄, BSP 之上零 HAL)
//
// 定位: 供 Devices/comm com_spi 用"中断事务 + 完成回调"替代阻塞 HAL_SPI_*.
//   SPI 是全双工显式事务 (非流式), 不套 comp_dma_rx/tx 循环 model;
//   用"一次一个事务 + 完成回调"达到非阻塞 (对标 libxr STM32SPI 的中断事务模型).
//
// 事务语义:
//   bsp_spi_transfer_async 启动一次全双工中断事务 (非阻塞, 立即返回);
//   完成/失败时调 BspSpiCb(me, ec, ctx) — 同一时刻只允许一个在途事务.
//   CS 由上层 (com_spi) 用 bsp_gpio 管理, 本层不碰.
//   平台内部: STM32 用 HAL_SPI_TransmitReceive_IT / Transmit_IT / Receive_IT.
//
// 完成中断: 应用把 SPIx IRQ 路由进 HAL_SPI_IRQHandler.
//   BSP 以 __weak 定义 HAL_SPI_TxRxCpltCallback / TxCpltCallback / RxCpltCallback /
//   ErrorCallback, 内部转调 bsp_spi_on_done → 用户 cb. 应用若自定义, 需自行调 bsp_spi_on_done.

#ifndef BSP_SPI_H
#define BSP_SPI_H

#include <stdint.h>
#include <stdbool.h>

// 不透明 SPI 句柄 (平台: STM32 = SPI_HandleTypeDef*; C2000 = SPIA/B 基址)
typedef void BspSpi;

// 事务完成回调 (中断上下文, 非阻塞; ec: 0=OK 非0=ErrorCode)
typedef void (*BspSpiCb)(BspSpi *me, int ec, void *ctx);

// 绑定平台 SPI 句柄 → 不透明 BspSpi* (NULL=参数错误)
BspSpi *bsp_spi_bind(void *h);

// 解绑 (中止在途事务 + 释放引用)
void bsp_spi_unbind(BspSpi *me);

// 启动全双工中断事务: 同时发 tx[0..len) 收 rx[0..len); rx.len 需 >= tx.len.
//   完成回调 cb(ctx). 返回 false = 参数错 / 有在途事务 (busy). 非阻塞.
bool bsp_spi_transfer_async(BspSpi *me, const uint8_t *tx, uint8_t *rx, uint16_t len, BspSpiCb cb, void *ctx);

// 启动单工发送事务: 发 tx[0..len) (rx=NULL); 完成回调 cb(ctx)
bool bsp_spi_write_async(BspSpi *me, const uint8_t *tx, uint16_t len, BspSpiCb cb, void *ctx);

// 启动单工接收事务: 收 rx[0..len) (tx=NULL); 完成回调 cb(ctx)
bool bsp_spi_read_async(BspSpi *me, uint8_t *rx, uint16_t len, BspSpiCb cb, void *ctx);

// 平台完成钩子: HAL SPI 完成/错误回调里调用 (结束在途事务并调用户 cb)
void bsp_spi_on_done(BspSpi *me, int ec);

#endif  // BSP_SPI_H
