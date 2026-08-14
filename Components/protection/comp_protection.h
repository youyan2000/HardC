// 关键保护机制组件 — 工业级保护模式
//
// 提供:
//   1. 滞回比较器 (防模式抖动)
//   2. 去抖动检测器 (防噪声误报)
//   3. 硬限幅器 (PWM/电流输出安全钳)
//   4. 模式切换同步器 (单周期平均过渡)
//   5. ISR 安全标记 (延迟 printf/日志到主循环)
//   6. 心跳看门狗 (ISR 死锁检测+自动复位)
//   7. 软启动/斜坡限制器 (故障恢复用)
//
// 全部 static inline, 零调用开销, ISR 安全.
//
// 模式切换保护+均流+去抖动
//       PWM限幅 + ISR安全

#ifndef COMP_PROTECTION_H
#define COMP_PROTECTION_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "comp_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 1. 滞回比较器 (Hysteresis Comparator) ==========
 *
 * 用途: 防止模式切换抖动 — 进入窗口窄, 退出窗口宽
 *
 * Buck-Boost 实例:
 *   进入窗口: 0.97 < ratio < 1.03  (窄, 不轻易进入)
 *   退出窗口: ratio < 0.90 或 ratio > 1.10 (宽, 不轻易退出)
 */

typedef struct {
  float enter_lo, enter_hi;   // 进入窗口 [enter_lo, enter_hi] — 窄
  float exit_lo,  exit_hi;    // 退出窗口 [exit_lo,  exit_hi]  — 宽
  bool  state;                // 当前状态 (true=在窗口内)
} Hysteresis;

static inline void Hysteresis_Init(Hysteresis *me,
                                   float enter_lo, float enter_hi,
                                   float exit_lo,  float exit_hi) {
  me->enter_lo = enter_lo; me->enter_hi = enter_hi;
  me->exit_lo  = exit_lo;  me->exit_hi  = exit_hi;
  me->state    = false;
}

static inline bool Hysteresis_Update(Hysteresis *me, float value) {
  if (me->state) {
    // 当前在窗口内: 用宽窗口判断退出
    if (value < me->exit_lo || value > me->exit_hi) {
      me->state = false;
    }
  } else {
    // 当前在窗口外: 用窄窗口判断进入
    if (value >= me->enter_lo && value <= me->enter_hi) {
      me->state = true;
    }
  }
  return me->state;
}


/* ========== 2. 去抖动检测器 (Debounce Detector) ==========
 *
 * 用途: 连续 N 次触发才确认, 防止噪声误报
 *
 * 配置:
 *   WARNING 级别: debounce 可配 (如 10次)
 *   FAULT 级别:   固定 80 次 (~2.8ms @28kHz)
 */

typedef struct {
  uint32_t threshold;     // 确认阈值 (连续触发次数)
  uint32_t counter;       // 当前连续计数
  bool     confirmed;     // 已确认状态
} Debounce;

static inline void Debounce_Init(Debounce *me, uint32_t threshold) {
  me->threshold = threshold;
  me->counter   = 0;
  me->confirmed = false;
}

// 每采样周期调用一次. 返回 true = 故障首次确认.
static inline bool Debounce_Update(Debounce *me, bool triggered) {
  if (triggered) {
    if (me->counter < me->threshold) {
      me->counter++;
    }
    if (me->counter >= me->threshold && !me->confirmed) {
      me->confirmed = true;
      return true;   // 首次确认
    }
  } else {
    me->counter   = 0;
    me->confirmed = false;
  }
  return false;
}

static inline void Debounce_Reset(Debounce *me) {
  me->counter   = 0;
  me->confirmed = false;
}

static inline void Debounce_SetThreshold(Debounce *me, uint32_t threshold) {
  me->threshold = threshold;
}


/* ========== 3. 硬限幅器 (Hard Clamp) ==========
 *
 * 用途: 在输出的最后一步强制限幅, 不依赖调用者
 *
 * 教训 #7: PWM 输出必须限幅
 *   "电机 PWM ±7200, 在 write_impl 中 hard clamp, 不可依赖调用者"
 */

static inline float HardClamp_f(float value, float lo, float hi) {
  if (value > hi) return hi;
  if (value < lo) return lo;
  return value;
}

static inline int32_t HardClamp_i32(int32_t value, int32_t lo, int32_t hi) {
  if (value > hi) return hi;
  if (value < lo) return lo;
  return value;
}

static inline uint32_t HardClamp_u32(uint32_t value, uint32_t lo, uint32_t hi) {
  if (value > hi) return hi;
  if (value < lo) return lo;
  return value;
}


/* ========== 4. 模式切换单周期同步器 ==========
 *
 * 用途: 模式切换的第一周期, 所有通道统一用平均输出过渡
 *       防止各通道因模式差异产生环流损坏驱动器
 *
 * 实现:
 *   if (cur_mode != last_mode) {
 *     sync_cmd = (alpha_cmd + beta_cmd + gamma_cmd) / 3.0f;
 *     统一用 sync_cmd 更新所有通道一个周期;
 *     last_mode = cur_mode;
 *   }
 */

typedef struct {
  uint8_t  num_channels;    // 通道数
  uint8_t  last_mode;       // 上一周期模式
  float    avg_cmd;         // 统一平均输出 (同步周期使用)
  bool     sync_pending;    // 当前周期是否需要同步
} ModeSync;

static inline void ModeSync_Init(ModeSync *me, uint8_t num_channels) {
  me->num_channels = num_channels;
  me->last_mode    = 0xFF;  // 无效值, 首次触发同步
  me->avg_cmd      = 0.0f;
  me->sync_pending = false;
}

// 每控制周期调用. 返回 true = 当前周期需用 avg_cmd 统一输出.
// cmds[] 长度为 num_channels
static inline bool ModeSync_Update(ModeSync *me, uint8_t cur_mode,
                                   const float *cmds) {
  if (cur_mode != me->last_mode) {
    float sum = 0.0f;
    for (uint8_t i = 0; i < me->num_channels; i++) {
      sum += cmds[i];
    }
    me->avg_cmd      = sum / (float)me->num_channels;
    me->last_mode    = cur_mode;
    me->sync_pending = true;
    return true;
  }
  me->sync_pending = false;
  return false;
}


/* ========== 5. ISR 安全标记 (Deferred Action) ==========
 *
 * 用途: ISR 中只 set flag, 主循环中执行耗时操作 (printf/send/log)
 *
 * 教训 #6: 不要在 ISR 中发串口
 *   "所有 printf/comm_send 在主循环 BackgroundTask() 中完成"
 *
 * 本结构 = 五原语之 Event-Flag (ISR→MAIN), 见 agent.md §1.2
 *   LibXR Event 的刻意简化 (无阻塞等待): HardC 无线程, "等待" = 各上下文按自身周期轮询
 *   错误分级 (WARNING/FAULT) 归 comp_error.h bitmask, 本结构只做 ISR→MAIN 的置位/轮询交接
 */

// 预定义 action 标记位 (可扩展至 32 个)
#define DEFER_LOG_SENSOR   (1u << 0)   // 传感器数据打印
#define DEFER_SEND_UART    (1u << 1)   // 串口发送
#define DEFER_PID_REPORT   (1u << 2)   // PID 参数上报
#define DEFER_FAULT_REPORT (1u << 3)   // 故障上报
#define DEFER_OLED_FLUSH   (1u << 4)   // OLED 刷新

typedef struct {
  volatile uint32_t flags;   // bitmask of pending actions
} DeferredAction;

static inline void DeferredAction_Init(DeferredAction *me) {
  me->flags = 0;
}

// ISR 中调用: 设置标记位
static inline void DeferredAction_Set(DeferredAction *me, uint32_t mask) {
  me->flags |= mask;
}

// 主循环中调用: 检查+原子清零
static inline uint32_t DeferredAction_Poll(DeferredAction *me, uint32_t mask) {
  uint32_t pending = me->flags & mask;
  me->flags &= ~mask;
  return pending;
}

// 主循环中: 只检查不清除
static inline bool DeferredAction_IsPending(const DeferredAction *me,
                                            uint32_t mask) {
  return (me->flags & mask) != 0;
}


/* ========== 6. 心跳看门狗 (Heartbeat Watchdog) ==========
 *
 * 用途: 检测 ISR 死锁
 *   - 快通道 ISR 中 heartbeat_++
 *   - 慢通道 ISR 中检查: 连续 N tick 心跳不变 → 停止喂狗 → IWDG 复位
 *
 * HRTIM ISR heartbeat++, TIM2 ISR 检查, 100 tick 不变 → 复位
 */

typedef struct {
  volatile uint32_t counter;        // 心跳计数 (ISR1 ++)
  uint32_t          last_snapshot;  // 上次快照值 (ISR2 记录)
  uint32_t          stale_ticks;    // 心跳不变的连续 tick 数
} Heartbeat;

static inline void Heartbeat_Init(Heartbeat *me) {
  me->counter       = 0;
  me->last_snapshot = 0;
  me->stale_ticks   = 0;
}

// 快通道 ISR 中调用
static inline void Heartbeat_Tick(Heartbeat *me) {
  me->counter++;
}

// 慢通道 ISR 中调用. threshold = 允许的最大不变 tick 数
// 返回 true = 心跳超时, 需复位
static inline bool Heartbeat_Check(Heartbeat *me, uint32_t threshold) {
  if (me->counter == me->last_snapshot) {
    me->stale_ticks++;
  } else {
    me->stale_ticks   = 0;
    me->last_snapshot = me->counter;
  }
  return (me->stale_ticks >= threshold);
}


/* ========== 7. 软启动/斜坡限制器 (Rate Limiter) ==========
 *
 * 用途: 限制输出变化率, 防止阶跃冲击 (故障恢复时的"逐步重启"策略)
 */

typedef struct {
  float max_rate;       // 最大变化率 (单位/秒)
  float current;        // 当前输出值
} RateLimiter;

static inline void RateLimiter_Init(RateLimiter *me, float max_rate) {
  me->max_rate = max_rate;
  me->current  = 0.0f;
}

static inline float RateLimiter_Step(RateLimiter *me, float target, float dt) {
  float max_step = me->max_rate * dt;
  float diff     = target - me->current;

  if (diff > max_step)       me->current += max_step;
  else if (diff < -max_step) me->current -= max_step;
  else                       me->current  = target;

  return me->current;
}

static inline void RateLimiter_Reset(RateLimiter *me, float value) {
  me->current = value;
}


#ifdef __cplusplus
}
#endif
#endif  // COMP_PROTECTION_H
