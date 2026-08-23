// 双缓冲 + 软件队列 + 单发 DMA 发送模型 (纯C, static inline, 无锁)
//
// 来源: LibXR src/driver/model/uart_dma_tx_model.hpp — UartDmaTxModel
// 翻译为 HardC 纯C 版本, 并按 HardC 并发模型简化:
//   LibXR 跑在 FreeRTOS 线程上, 用 SerializedService(事件合并+单owner) 防提交线程与
//   完成 ISR 并发推进双缓冲; HardC 是"全序+单一抢占源"(仅 FAST 抢占, 调用树即上下文),
//   天然只有一个 owner 推进, 因此省略 SerializedService, 只保留核心:
//     - MAIN 侧 submit: 数据入软件队列 → 若空闲则取队首装 active → 调 backend 启动 DMA
//     - ISR 侧 on_tx_done: 标空闲 → 若队非空则推进 next 并再启动 DMA
//   单抢占保证 startup 无竞态: MAIN 读 busy==false 时必无在途 DMA, 故无完成 ISR 会
//   "恰好此刻" 抢占并改写状态 (忙标志为 false ⇒ 无在途中断源).
//
// 结构:
//   txq (Ring)          — MAIN 侧提交的待发字节队列
//   db (DoubleBuffer)   — 两块 DMA 缓冲 (active=正在发, pending=下一个)
//   接收方/backend      — tx_start(port, data, len) 平台启动单发 DMA (非阻塞)
//
// 使用示例 (UART TX):
//   DmaTxModel txm;  Ring txq;  uint8_t qbuf[256];
//   uint8_t dma_buf[64 /*分两块各32*/];  DoubleBuffer db;
//   dma_tx_model_init(&txm, &txq, &db, dma_buf, sizeof(dma_buf), port, uart_backend_tx_start);
//   // MAIN: dma_tx_model_submit(&txm, data, len);
//   // DMA 完成 ISR: dma_tx_model_on_done(&txm);

#ifndef COMP_DMA_TX_H
#define COMP_DMA_TX_H

#include "comp_ring.h"
#include "comp_double_buffer.h"
#include <stdint.h>
#include <stdbool.h>

// 平台后端: 启动单发 DMA 发送 (非阻塞, 数据已就绪在 data,len; 返回 true=已启动)
//   port: 平台外设句柄 (STM32 外设 handle / C2000 基址), bind 时注入
typedef bool (*DmaTxStartFn)(void *port, const uint8_t *data, uint16_t len);

typedef struct {
  Ring *txq;              // 待发字节队列 (MAIN 写 / model 读)
  DoubleBuffer *db;       // DMA 双缓冲 (active/pending)
  void *port;             // 平台后端上下文
  DmaTxStartFn tx_start;  // 启动单发 DMA (backend)
  uint16_t active_len;    // 当前 active 块有效字节数
  bool busy;              // DMA 是否持有 active 请求 (在途)
} DmaTxModel;

// 初始化: 装配队列/双缓冲/backend; 缓冲对半切给双缓冲
static inline void dma_tx_model_init(DmaTxModel *me, Ring *txq, DoubleBuffer *db, uint8_t *storage,
                                     uint16_t storage_size, void *port, DmaTxStartFn tx_start) {
  me->txq = txq;
  me->db = db;
  me->port = port;
  me->tx_start = tx_start;
  me->active_len = 0u;
  me->busy = false;
  double_buffer_init(db, storage, storage_size);
}

// ---- 内部: 从队列取队首段装进 active 缓冲并启动 DMA (非阻塞) ----

static inline bool dma_tx_model_start_active(DmaTxModel *me) {
  // 从队列读 min(可装, 队内) 字节到 active 缓冲
  uint16_t cap = double_buffer_size(me->db);
  uint16_t n = ring_avail(me->txq);
  if (n > cap) {
    n = cap;
  }
  if (n == 0u) {
    return false;  // 队列空
  }
  uint8_t *dst = double_buffer_active(me->db);
  uint16_t got = ring_read(me->txq, dst, n);
  me->active_len = got;
  me->busy = true;
  if (me->tx_start && me->tx_start(me->port, dst, got)) {
    return true;
  }
  me->busy = false;  // 后端启动失败
  me->active_len = 0u;
  return false;
}

// ---- MAIN 侧: 提交数据发送 (非阻塞) ----

// 入队并尽量启动; 队列满则只入能装的(返回实际入队字节数, 不阻塞)
static inline uint16_t dma_tx_model_submit(DmaTxModel *me, const uint8_t *data, uint16_t len) {
  uint16_t n = ring_write(me->txq, data, len);
  // 空闲时启动第一段 (单抢占保证无在途 DMA ⇒ 无完成 ISR 抢占, 见头注释)
  if (!me->busy) {
    (void) dma_tx_model_start_active(me);
  }
  return n;
}

// ---- ISR 侧: DMA 完成 ----

// DMA 完成回调 (中断上下文, 非阻塞): 释放 active 并推进下一段
static inline void dma_tx_model_on_done(DmaTxModel *me) {
  me->busy = false;
  me->active_len = 0u;
  if (me->txq && ring_avail(me->txq) != 0u) {
    (void) dma_tx_model_start_active(me);
  }
}

// 查询是否仍有在途 DMA (MAIN 侧串行查询)
static inline bool dma_tx_model_is_busy(const DmaTxModel *me) {
  return me->busy;
}

#endif  // COMP_DMA_TX_H
