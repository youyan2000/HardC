// 电机控制 — 模 6 换相计数器
//
// 来源: TI controlSUITE motor_control/math_blocks/v4.3/mod6_cnt.h
// 翻译为 C-OOP 纯C float 版本
//
// BLDC 六步换相: 每次触发输入有效, 换相步计数器递增 (0→1→...→5→0).
// 计数 0~5 对应六种换相状态 (电机电角度每 60° 换相一次).
//
// 触发输入: >0 有效 (Q0, 满刻度 0x7FFF), 通常接位置信号或 PWM 周期信号.
// 输出: 当前换相步 (0,1,2,3,4,5), 可索引六步换相电压矢量表.

#ifndef COMP_MOD6_H
#define COMP_MOD6_H

#include <stdint.h>

typedef struct {
  uint32_t trig_input;  // 输入: 换相触发信号 (Q0, >0 有效)
  uint32_t counter;     // 输出: 换相步 (Q0, 0~5)
} Mod6Cnt;

// 初始化 — 计数器清零
static inline void mod6_init(Mod6Cnt *me) {
  me->trig_input = 0;
  me->counter = 0;
}

// 单步运行 (触发沿时调用)
//   trig — 触发信号 (Q0, >0 有效)
//   返回: 当前换相步 (0~5)
static inline uint32_t mod6_tick(Mod6Cnt *me, uint32_t trig) {
  me->trig_input = trig;

  if (trig > 0) {
    if (me->counter == 5) {
      me->counter = 0;  // 到 5 回绕
    } else {
      me->counter++;    // 否则递增
    }
  }

  return me->counter;
}

#endif  // COMP_MOD6_H
