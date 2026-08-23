// BSP UART 硬件抽象接口 — 平台无关 DMA 收发原语 (不透明句柄)
//
// 定位: 供 Devices/comm com_uart 接入 DMA model (comp_dma_rx / comp_dma_tx),
//   BSP 之上零 HAL: com_uart 只持 BspUart 不透明句柄 + 调 bsp_uart_* 抽象.
//   平台内部 (bsp_uart_stm32.c 用 HAL, bsp_uart_c2000.c 用 driverlib) 自决.
//
// 接口对应 DMA 三件套:
//   bsp_uart_dma_rx_start      — 启动循环 DMA 接收 (喂 comp_dma_rx 的 DmaRxModel)
//   bsp_uart_dma_rx_remaining  — 当前剩余未读字节 (DmaRxModel::update 的 remaining 源)
//   bsp_uart_dma_tx_start      — 单发 DMA 发送 (喂 comp_dma_tx 的 DmaTxStartFn)
//   bsp_uart_dma_tx_busy       — DMA 是否在途 (供 model/上层查询)
//
// RX 用"位置式循环 DMA" (对齐 comp_dma_rx DmaRxModel, 对标 libxr UartCircularDmaRxModel):
//   单块连续循环缓冲, DMA 写满回绕, 软件读位置经 remaining 推算.
//   (不是 HAL 双缓冲 half 切换 — 那是另一套机制, 与 comp_dma_rx 不匹配.)
//
// 完成中断: 由应用把 UART/DMA IRQ 路由进 HAL_/driverlib 中断处理, 在完成回调里调
//   com_uart 的 on_rx_event / on_tx_done (见 com_uart API).

#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>
#include <stdbool.h>

// 不透明 UART 句柄 (平台: STM32 = UART_HandleTypeDef*; C2000 = SCIA/SCIB 基址)
typedef void BspUart;

// 绑定平台 UART 句柄 → 不透明 BspUart* (NULL=参数错误)
//   h: 平台句柄 (STM32: UART_HandleTypeDef*; C2000: SCIA/B 基址)
BspUart *bsp_uart_bind(void *h);

// 解绑 (停 DMA + 释放引用)
void bsp_uart_unbind(BspUart *me);

// ---- RX (循环 DMA, 位置式 — 对齐 comp_dma_rx DmaRxModel) ----

// 启动循环 DMA 接收: DMA 以循环模式连续写满 buf[0..cap) 后回绕 (位置式).
//   buf/cap: DMA 循环缓冲 (由上层 DmaRxModel 提供, 单块连续区)
void bsp_uart_dma_rx_start(BspUart *me, uint8_t *buf, uint16_t cap);

// 当前 DMA 剩余未读字节数 (0..cap) — comp_dma_rx::update 的 remaining 源
//   STM32: __HAL_DMA_GET_COUNTER(hdmarx); C2000: DMA 剩余计数寄存器
uint16_t bsp_uart_dma_rx_remaining(BspUart *me);

// 停止 DMA 接收
void bsp_uart_dma_rx_stop(BspUart *me);

// RX 事件回调 (后端 RX 中断/事件调用, 通知上层有新字节 → comp_dma_rx update)
typedef void (*BspUartRxEventFn)(BspUart *me, void *ctx);
// 注册 RX 事件回调 (每收到新字节/事件时调用; 可空=不通知)
void bsp_uart_set_rx_event_cb(BspUart *me, BspUartRxEventFn cb, void *ctx);

// ---- TX (单发 DMA) ----

// 启动单发 DMA 发送 data[0..len), 返回 true=已启动 (非阻塞)
bool bsp_uart_dma_tx_start(BspUart *me, const uint8_t *data, uint16_t len);

// DMA 发送是否在途
bool bsp_uart_dma_tx_busy(BspUart *me);

// TX 完成回调 (后端 TX 中断调用, 通知上层续发下一段; 对标 comp_dma_tx on_done)
typedef void (*BspUartTxDoneFn)(BspUart *me, void *ctx);
// 注册 TX 完成回调 (bsp_uart_dma_tx_start 启动的传输完成时调用; 可空=不通知)
void bsp_uart_set_tx_done_cb(BspUart *me, BspUartTxDoneFn cb, void *ctx);

#endif  // BSP_UART_H