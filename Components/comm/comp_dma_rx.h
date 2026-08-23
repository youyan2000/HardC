// 位置式循环 DMA 接收模型 — DMA 环形缓冲 → 软件环形队列 (纯C, static inline, 无锁)
//
// 来源: LibXR src/driver/model/uart_circular_dma_rx_model.hpp — UartCircularDmaRxModel
// 翻译为 HardC 纯C 版本: 一块 DMA 可写循环缓冲 + 软件读位置, DMA 事件时把新产出字节推入
// 软件队列 (comp_ring.h Ring), 兼容任意总线的接收 (UART/SPI/I2C, 以及 ADC 类).
//
// 机制 (对照 LibXR):
//   - DMA 以循环模式在一块连续缓冲 buf[0..size) 里连续写入 (写满回绕).
//   - 软件侧维护 last_pos: 上次已消费的 DMA 写位置.
//   - 每次 DMA 完成/过半/IDLE 事件, 上层取得 "DMA 剩余未读字节数 remaining"
//     (平台后端 GetCircularDmaRxRemaining, 如 STM32 __HAL_DMA_GET_COUNTER),
//     由 current_pos = size - remaining 得当前写位置, 算出 [last_pos, current_pos) 的新字节.
//   - 新字节可能跨环形边界 (wrap), 分两段经 ring_write 推入软件队列 Ring.
//   - 消费者 (MAIN) 用 ring_read 从 Ring 取走; 软件队列满时 ring_write 只写进能装下的,
//     读位置仍推进 (与 LibXR 保守一致: 无法入队的数据被丢弃, 不阻塞 DMA).
//
// 使用示例 (RX):
//   uint8_t dma_buf[64];  Ring  rxq;  uint8_t qbuf[128];  DmaRxModel rxm;
//   dma_rx_model_init(&rxm, dma_buf, sizeof(dma_buf));
//   ring_init(&rxq, qbuf, sizeof(qbuf));
//   // 平台: StartCircularDmaRx(dma_buf, sizeof(dma_buf))
//   // DMA 完成 ISR:  uint16_t remain = <platform>GetRemaining();
//   //                dma_rx_model_update(&rxm, remain, &rxq);   // 推新数据入队
//   // MAIN: ring_read(&rxq, ...) 消费

#ifndef COMP_DMA_RX_H
#define COMP_DMA_RX_H

#include "comp_ring.h"
#include <stdint.h>

typedef struct {
  uint8_t *buf;       // DMA 可写环形缓冲 (调用者提供, 静态分配)
  uint16_t size;      // 缓冲容量 (字节)
  uint16_t last_pos;  // 上次已消费的 DMA 写位置 (软件侧读位置)
} DmaRxModel;

// 初始化: 绑定 DMA 缓冲, 读位置复位到 0
static inline void dma_rx_model_init(DmaRxModel *me, uint8_t *buf, uint16_t size) {
  me->buf = buf;
  me->size = size;
  me->last_pos = 0u;
}

// DMA 当前可写缓冲 (平台启动循环 DMA 用)
static inline uint8_t *dma_rx_model_buffer(const DmaRxModel *me) {
  return me->buf;
}

// DMA 缓冲容量
static inline uint16_t dma_rx_model_size(const DmaRxModel *me) {
  return me->size;
}

// 复位软件读位置到缓冲起点 (启动/重配时调用)
static inline void dma_rx_model_reset_position(DmaRxModel *me) {
  me->last_pos = 0u;
}

// DMA 事件时调用: 用平台提供的"剩余未读字节数"算出新产出区间, 推入软件队列.
//   remaining: DMA 当前剩余未读字节数 (0..size, 由平台后端 GetCircularDmaRxRemaining 提供)
//   q:         消费者 (MAIN) 读取的软件环形队列 (comp_ring.h)
// 返回推入队列的字节数; 队列满时丢数据 (读位置仍推进), 生产者永不阻塞.
static inline uint16_t dma_rx_model_update(DmaRxModel *me, uint16_t remaining, Ring *q) {
  if (remaining > me->size) {
    remaining = me->size;  // 防御: 平台异常计数
  }
  const uint16_t cur = (uint16_t) (me->size - remaining);  // 当前 DMA 写位置
  if (cur == me->last_pos) {
    return 0u;  // 无新数据
  }

  uint16_t pushed = 0u;
  if (cur > me->last_pos) {
    // 无回绕: [last_pos, cur) 一段
    pushed = ring_write(q, &me->buf[me->last_pos], (uint16_t) (cur - me->last_pos));
  } else {
    // 回绕: [last_pos, size) + [0, cur) 两段
    pushed = ring_write(q, &me->buf[me->last_pos], (uint16_t) (me->size - me->last_pos));
    pushed = (uint16_t) (pushed + ring_write(q, me->buf, cur));
  }
  me->last_pos = cur;  // 读位置始终推进到当前写位置 (丢弃无法入队的数据)
  return pushed;
}

#endif  // COMP_DMA_RX_H
