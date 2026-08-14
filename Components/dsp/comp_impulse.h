// DSP 测试信号 — 脉冲发生器
//
// 来源: TI controlSUITE motor_control/math_blocks/v4.3/impulse.h
// 翻译为 HardC 纯C float 版本
//
// 每 Period 个采样周期输出一个满幅脉冲 (0x7FFF 满刻度), 其余周期输出 0.
// 用途: 注入阶跃/冲激测试信号, 配合频响分析仪观测系统动态响应.
//       输出数值与 comp_dlog (数据记录器) 的满刻度约定一致.

#ifndef COMP_IMPULSE_H
#define COMP_IMPULSE_H

#include <stdint.h>

typedef struct {
  uint32_t period;    // 输出脉冲周期 (采样周期数, Q0)
  uint32_t out;       // 输出: 0 或 0x00007FFF (Q0)
  uint32_t counter;   // 计数器 (Q0)
} Impulse;

// 初始化 — period 为两次脉冲之间的采样周期数
static inline void impulse_init(Impulse *me, uint32_t period) {
  me->period = period;
  me->out = 0;
  me->counter = 0;
}

// 单步运行 (ISR 中每采样周期调用)
//   返回: 本周期输出 (0 或 0x00007FFF)
static inline uint32_t impulse_tick(Impulse *me) {
  me->out = 0;          // 进入时先清零
  me->counter++;        // 递增计数器

  if (me->counter >= me->period) {
    me->out = 0x00007FFF;  // 达到周期 → 输出满幅脉冲
    me->counter = 0;       // 复位计数器
  }

  return me->out;
}

#endif  // COMP_IMPULSE_H
