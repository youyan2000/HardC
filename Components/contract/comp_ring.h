// SPSC 无锁环形缓冲 — 单生产者单消费者跨上下文交接 (纯C, 零 malloc, 无锁)
//
// 来源: LibXR (bsp-dev-c/Jiu-xiao) src/structure/queue/spsc_queue_base.hpp — SPSCQueueBase
// 翻译为 HardC 纯C 字节环形缓冲 static inline 版本 (LibXR 用模板+动态分配, HardC 无模板 → 按字节+调用者缓冲)
//
// 用途: ISR→MAIN 字节流 (事件帧/日志流/遥测队列)
//   - 生产者 (如 ISR) 调用 ring_push / ring_write, 永不阻塞
//   - 消费者 (如 MAIN) 调用 ring_pop / ring_read / ring_peek
//   - 全序 + 单一抢占源下, 生产者只碰 head, 消费者只碰 tail (见 agent.md §1.2)
//
// 机制:
//   - 环容量 = size - 1 字节 (保留一格区分 空/满)
//   - size 必须 ≥ 2 (size=1 时容量为 0; size=0 未定义, head%size 会除零)
//   - head/tail 为 volatile uint16_t, 无符号回绕给出已占字节数
//   - 缓冲由调用者提供, 大小任意 (不要求 2 的幂)
//
// 使用示例 (ISR 日志 → 主循环打印):
//   uint8_t log_buf[128];  Ring log;
//   ring_init(&log, log_buf, sizeof(log_buf));
//   // ISR 内: ring_write(&log, (const uint8_t *)"fault", 5);
//   // MAIN 内: while (ring_pop(&log, &b)) uart_putc(b);

#ifndef COMP_RING_H
#define COMP_RING_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
  uint8_t *buf;            // 由调用者提供的缓冲 (大小 = size 字节)
  uint16_t size;           // 缓冲总容量 (可容纳 size-1 字节)
  volatile uint16_t head;  // 生产者写入位置 (只由生产者修改)
  volatile uint16_t tail;  // 消费者读取位置 (只由消费者修改)
} Ring;

// 初始化: 绑定缓冲, 清空
static inline void ring_init(Ring *me, uint8_t *buf, uint16_t size) {
  me->buf = buf;
  me->size = size;
  me->head = 0;
  me->tail = 0;
}

// 已占字节数 (head - tail 无符号回绕恒正确)
static inline uint16_t ring_avail(const Ring *me) {
  return (uint16_t) (me->head - me->tail);
}

// 剩余可写字节数 (最多 size-1)
static inline uint16_t ring_space(const Ring *me) {
  return (uint16_t) (me->size - 1u - ring_avail(me));
}

// 生产者: 压入单字节 (满则返回 false, 不覆盖)
static inline bool ring_push(Ring *me, uint8_t byte) {
  if (ring_space(me) == 0) {
    return false;
  }
  me->buf[me->head % me->size] = byte;
  me->head++;
  return true;
}

// 消费者: 弹出单字节 (空则返回 false)
static inline bool ring_pop(Ring *me, uint8_t *out) {
  if (ring_avail(me) == 0) {
    return false;
  }
  *out = me->buf[me->tail % me->size];
  me->tail++;
  return true;
}

// 消费者: 窥视下一个字节 (不消费)
static inline bool ring_peek(const Ring *me, uint8_t *out) {
  if (ring_avail(me) == 0) {
    return false;
  }
  *out = me->buf[me->tail % me->size];
  return true;
}

// 生产者: 批量写入最多 len 字节, 返回实际写入数 (自动处理环回绕)
static inline uint16_t ring_write(Ring *me, const uint8_t *data, uint16_t len) {
  uint16_t n = ring_space(me);
  if (len < n) {
    n = len;
  }
  uint16_t pos = (uint16_t) (me->head % me->size);
  uint16_t first = (uint16_t) (me->size - pos);  // 到环尾的连续段长度
  if (first > n) {
    first = n;
  }
  for (uint16_t i = 0; i < first; i++) {
    me->buf[pos + i] = data[i];
  }
  for (uint16_t i = first; i < n; i++) {
    me->buf[i - first] = data[i];
  }
  me->head = (uint16_t) (me->head + n);
  return n;
}

// 消费者: 批量读出最多 len 字节, 返回实际读出数 (自动处理环回绕)
static inline uint16_t ring_read(Ring *me, uint8_t *data, uint16_t len) {
  uint16_t n = ring_avail(me);
  if (len < n) {
    n = len;
  }
  uint16_t pos = (uint16_t) (me->tail % me->size);
  uint16_t first = (uint16_t) (me->size - pos);  // 到环尾的连续段长度
  if (first > n) {
    first = n;
  }
  for (uint16_t i = 0; i < first; i++) {
    data[i] = me->buf[pos + i];
  }
  for (uint16_t i = first; i < n; i++) {
    data[i] = me->buf[i - first];
  }
  me->tail = (uint16_t) (me->tail + n);
  return n;
}

// 重置 (清空) — 仅当两端都不会并发访问时调用 (如启动时)
static inline void ring_reset(Ring *me) {
  me->head = 0;
  me->tail = 0;
}

#endif  // COMP_RING_H
