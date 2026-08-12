// 步进电机平台层 — 基类实现
// 来源: Car_Control_Study_Report §23.3 (StepMotorBase 设计)
//       基类只管理: 相序表绑定 + 软限位 + 公共状态字段初始化

#include "comp_step_motor.h"
#include <stddef.h>   // NULL

// ======== 基类构造 ========

void stepmotor_base_init(StepMotorBase *me, const StepPhaseTable *phase_tbl,
                          int32_t limit_lo, int32_t limit_hi) {
  me->ops       = NULL;     // 子类 init 时绑定
  me->phase_tbl = phase_tbl;

  me->steps_remaining = 0;
  me->pos            = 0;
  me->pos_limit_lo   = limit_lo;
  me->pos_limit_hi   = limit_hi;

  me->period_target  = 1000;  // 默认 1ms 周期 (~1kHz, 安全的低速)
  me->period_current = 1000;
  me->ramp_step      = 0;

  me->phase   = 0;
  me->dir     = 1;       // 默认正向
  me->moving  = false;
  me->estop   = false;
}
