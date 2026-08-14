// 双缓冲 (PingPong) — DMA→FAST 采样快照 / 发送-日志双缓冲 (纯C, 零 malloc, 无锁)
//
// 来源: LibXR (bsp-dev-c/Jiu-xiao) src/structure/double_buffer.{hpp,cpp} — DoubleBuffer
// 翻译为 HardC 纯C static inline 版本 (LibXR 用类+模板, HardC 无类 → 字节缓冲 + 显式状态位)
//
// 用途: 一块连续内存对半切分成两个等大缓冲, DMA/外设写一块的同时 FAST 处理另一块
//   - 生产侧 (如 DMA 完成 ISR): FillPending / EnablePending → 标 pending 就绪
//   - 消费侧 (如 FAST 采样处理): HasPending → Switch → ActiveBuffer → 处理快照
//   - pending 就绪位 = [Components/contract/comp_io.h] 的 IO_ASYNC_FLAG 完成语义 (DMA 完成置位, 消费者轮询消费)
//
// 机制 (对照 LibXR):
//   - init 把连续内存对半切分 → buf[0]/buf[1], 单块 size = total/2
//   - active_ 当前活动块; pending_valid_ 备用块是否就绪
//   - FillPending 写非活动块 (PENDING), 置 valid; Switch 在 valid 时翻转 active_
//   - 全序 + 单一抢占源下, 生产侧只碰 Pending 半块, 消费侧只碰 Active 半块 (见 agent.md §1.2)
//   - 两半块由同一连续数组派生, 天然对齐; total_size 必须为 2 的整数倍

#ifndef COMP_DOUBLE_BUFFER_H
#define COMP_DOUBLE_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
  uint8_t *buf[2];           // 两半块指针 (init 对半切分)
  uint16_t size;             // 单块大小 (字节) = total/2
  uint8_t active;            // 当前活动块编号 (0/1)
  volatile uint8_t pending;  // 备用块是否就绪 (生产者置位, Switch 清零) = IO_ASYNC_FLAG
  uint16_t active_len;       // 活动块有效数据长度 (字节)
  uint16_t pending_len;      // 备用块有效数据长度 (字节)
} DoubleBuffer;

// 初始化: 绑定连续内存 (大小 total_size, 必须偶数), 对半切分为 block0/1, 状态清零
static inline void double_buffer_init(DoubleBuffer *me, void *storage, uint16_t total_size) {
  uint8_t *p = (uint8_t *) storage;
  me->buf[0] = p;
  me->buf[1] = p + (total_size / 2u);
  me->size = total_size / 2u;
  me->active = 0u;
  me->pending = 0u;
  me->active_len = 0u;
  me->pending_len = 0u;
}

// 重置运行时状态 (保留已绑定缓冲) — 仅当两端都不会并发访问时调用 (如启动时)
static inline void double_buffer_reset(DoubleBuffer *me) {
  me->active = 0u;
  me->pending = 0u;
  me->active_len = 0u;
  me->pending_len = 0u;
}

// 当前活动缓冲指针 (消费侧读)
static inline uint8_t *double_buffer_active(const DoubleBuffer *me) {
  return me->buf[me->active];
}

// 当前备用缓冲指针 (生产侧写)
static inline uint8_t *double_buffer_pending(const DoubleBuffer *me) {
  return me->buf[1u - me->active];
}

// 按块号访问 (0/1) — 供上层状态机保持稳定块编号语义
static inline uint8_t *double_buffer_block(const DoubleBuffer *me, uint8_t block) {
  return me->buf[block & 1u];
}

// 单块大小 (字节)
static inline uint16_t double_buffer_size(const DoubleBuffer *me) {
  return me->size;
}

// 切换活动块: 仅当备用块就绪时翻转 active 并清 valid
static inline void double_buffer_switch(DoubleBuffer *me) {
  if (me->pending != 0u) {
    me->active ^= 1u;
    me->pending = 0u;
  }
}

// 备用块是否就绪 (消费者轮询)
static inline bool double_buffer_has_pending(const DoubleBuffer *me) {
  return me->pending != 0u;
}

// 生产侧: 写入备用块并标就绪 (不可重入) — 已就绪或超长则失败
static inline bool double_buffer_fill_pending(DoubleBuffer *me, const uint8_t *data, uint16_t len) {
  if (me->pending != 0u || len > me->size) {
    return false;
  }
  uint8_t *dst = me->buf[1u - me->active];
  for (uint16_t i = 0u; i < len; i++) {
    dst[i] = data[i];
  }
  me->pending_len = len;
  me->pending = 1u;
  return true;
}

// 生产侧: 直接写活动块 (原地更新, 不动 pending 状态)
// 刻意差异: LibXR FillActive 只拷贝不记长度, 此处顺带记录 active_len 供消费侧取用
static inline bool double_buffer_fill_active(DoubleBuffer *me, const uint8_t *data, uint16_t len) {
  if (len > me->size) {
    return false;
  }
  uint8_t *dst = me->buf[me->active];
  for (uint16_t i = 0u; i < len; i++) {
    dst[i] = data[i];
  }
  me->active_len = len;
  return true;
}

// 生产侧: 手动置 pending 就绪 (配合 FillActive/手动写缓冲使用)
static inline void double_buffer_enable_pending(DoubleBuffer *me) {
  me->pending = 1u;
}

// 备用块有效数据长度 (仅就绪时可见)
static inline uint16_t double_buffer_get_pending_len(const DoubleBuffer *me) {
  return me->pending != 0u ? me->pending_len : 0u;
}

// 活动块有效数据长度
static inline uint16_t double_buffer_get_active_len(const DoubleBuffer *me) {
  return me->active_len;
}

// 设置备用块数据长度 (DMA 完成 ISR 记录实际字节数)
static inline void double_buffer_set_pending_len(DoubleBuffer *me, uint16_t len) {
  me->pending_len = len;
}

// 设置活动块数据长度
static inline void double_buffer_set_active_len(DoubleBuffer *me, uint16_t len) {
  me->active_len = len;
}

// 翻转活动块编号 (无 valid 校验的强制切换)
static inline void double_buffer_flip_active(DoubleBuffer *me) {
  me->active ^= 1u;
}

// 当前活动块编号 (0/1)
static inline uint8_t double_buffer_active_block(const DoubleBuffer *me) {
  return me->active;
}

// 设置活动块 (false=block0, true=block1)
static inline void double_buffer_set_active_block(DoubleBuffer *me, bool block) {
  me->active = block ? 1u : 0u;
}

#endif  // COMP_DOUBLE_BUFFER_H
