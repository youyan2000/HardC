// 最新值锁存 — FAST→SLOW/MAIN 遥测交接 (纯C, 零 malloc, 无锁)
//
// 来源: LibXR (bsp-dev-c/Jiu-xiao) src/middleware/message/ — Topic<T> + ASyncSubscriber (异步主动拉取最新)
// 翻译为 HardC 纯C 单槽最新值 static inline 版本 (LibXR 用模板+发布订阅, HardC 无模板 → 单生产者直写 + dirty/seq)
//
// 用途: FAST 循环内写最新测量值, SLOW/MAIN 侧按需读取
//   - 生产者 (FAST) 调用 latch_write
//   - 消费者 (SLOW/MAIN) 调用 latch_read / latch_peek
//   - 全序 + 单一抢占源下, 生产者写序列对消费者原子可见 (见 agent.md §1.2)
//
// 语义:
//   - value 恒为最新值 (写即覆盖, 丢弃中间值符合"最新值"定义)
//   - dirty  自上次读取后是否有新写 (生产者置位, 消费者读取后清零)
//   - seq    单调递增写入序号 (用于检测漏读 / 多消费者)
//
// 使用示例 (FAST 采样 → SLOW 聚合):
//   Latch telemetry_latch;
//   latch_init(&telemetry_latch);
//   // FAST: latch_write(&telemetry_latch, adc_vout);
//   // SLOW: float v; if (latch_read(&telemetry_latch, &v)) log_printf("vout=%.3f", v);

#ifndef COMP_LATCH_H
#define COMP_LATCH_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
  volatile float value;    // 最近写入值 (消费者始终可读)
  volatile uint32_t seq;   // 单调递增写入序号 (检测漏读)
  volatile uint8_t dirty;  // 1 = 有新值未读
} Latch;

// 初始化: 值清零, 脏标志清, 序号清零
static inline void latch_init(Latch *me) {
  me->value = 0.0f;
  me->seq = 0u;
  me->dirty = 0u;
}

// 生产者 (FAST): 写入最新值 — 先写值, 再递增序号, 最后置脏 (对消费者原子可见)
static inline void latch_write(Latch *me, float v) {
  me->value = v;
  me->seq++;
  me->dirty = 1u;
}

// 消费者: 读取最新值并清除脏标志; 有未读新值返回 true
//   读序先 dirty 后 value: 若生产者恰在两次读间写新值, 返回最新值但 fresh 可能为 false —
//   值恒为当前, 标志偶有丢失, 该语义对遥测足够 (丢标志不漏值, 见 header 说明)
static inline bool latch_read(Latch *me, float *out) {
  bool fresh = (me->dirty != 0u);
  *out = me->value;
  me->dirty = 0u;
  return fresh;
}

// 消费者: 只读不清脏 (轮询检查而不消费)
static inline float latch_peek(const Latch *me) {
  return me->value;
}

// 消费者: 自上次读取后是否有新值
static inline bool latch_is_dirty(const Latch *me) {
  return me->dirty != 0u;
}

#endif  // COMP_LATCH_H
