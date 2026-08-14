// 步进电机驱动 — StepMotorBase 的子类 (Devices 层实现)
//       + motor_step.c (setspeed/setposition ops 绑定)
//       + user_step_motor.c (A→C→B→D 4 相全步进)
//
// 架构:
//   StepMotorBase (comp_step_motor.h)
//     ↑ container_of
//   StepMotor (本文件) — BSP 函数指针 + ctx + 4相引脚
//
// Bug 规避验证清单:
//   ✅ P0#1: 无 static 局部变量 — lock_cnt/last_dir 等都在 StepMotor 结构体
//   ✅ P0#2: set_rate → bsp_step_pul_set_period(ctx, period) → timer->LOAD
//   ✅ P1#4: set_dir(ctx, high) — ctx 被实际传给 BSP 函数, 不丢弃
//   ✅ P2#5: ramp_step + period_current → period_target 加减速占位
//   ✅ P2#7: pos_limit_lo/hi 软限位检查
//   ✅ P2#8: 初始化时 phase=0, dir=1, 重启不依赖记忆

#include "motor_step.h"
#include "bsp_step_motor.h"
#include "container_of.h"
#include "comp_math.h"   // CLAMP, math_clamp_i32
#include <stddef.h>      // NULL

// ======== 前向声明 ops 回调 ========
static void stepmotor_set_rate_ops(StepMotorBase *base, uint32_t freq_hz);
static void stepmotor_set_steps_ops(StepMotorBase *base, int32_t steps);
static int32_t stepmotor_get_steps_ops(StepMotorBase *base);
static void stepmotor_set_ramp_ops(StepMotorBase *base, uint32_t accel);
static void stepmotor_set_phase_mode_ops(StepMotorBase *base,
                                          const StepPhaseTable *tbl);
static void stepmotor_estop_ops(StepMotorBase *base);

// ======== 虚函数表 (静态常量, 所有实例共享) ========
// ✅ Bug #1: 这是 const 且在文件作用域但只是数据, 无状态 — 安全
static const StepMotorOps stepmotor_ops = {
  .set_rate       = stepmotor_set_rate_ops,
  .set_steps      = stepmotor_set_steps_ops,
  .get_steps      = stepmotor_get_steps_ops,
  .set_ramp       = stepmotor_set_ramp_ops,
  .set_phase_mode = stepmotor_set_phase_mode_ops,
  .emergency_stop = stepmotor_estop_ops,
};

// ======== 辅助: container_of 宏 (已有 container_of.h, 此处直接用) ========

// ======== 构造 ========

void stepmotor_init(StepMotor *me, const StepMotorCfg *cfg) {
  // 基类初始化
  stepmotor_base_init(&me->base, cfg->phase_tbl, cfg->limit_lo, cfg->limit_hi);

  // 绑定虚表
  me->base.ops = &stepmotor_ops;

  // BSP 注入 (硬件操作 — ctx 必须被使用, Bug #4)
  me->phase_pins = cfg->phase_pins;
  me->set_dir    = cfg->set_dir;
  me->dir_ctx    = cfg->dir_ctx;
  me->set_period = cfg->set_period;
  me->pul_ctx    = cfg->pul_ctx;
  me->set_enable = cfg->set_enable;

  // 运行时状态初始化
  me->lock_cnt  = 0;
  me->last_dir  = 1;
  me->locked    = false;
  me->ramping   = false;

  // 初始相: 全部关断
  if (me->set_dir) {
    me->set_dir(me->dir_ctx, true);  // ✅ Bug #4: ctx 被实际使用
  }
  // 关所有相
  uint8_t zero_phase = 0x00;
  bsp_step_write_phase(me->phase_pins, zero_phase);
}

// ======== ops 回调实现 ========

static void stepmotor_set_rate_ops(StepMotorBase *base, uint32_t freq_hz) {
  StepMotor *me = container_of(base, StepMotor, base);

  if (me->base.estop) return;  // 急停后忽略命令

  // 频率 → 定时器周期 (假设定时器时钟 = 80MHz, prescaler = 80 → 1MHz tick)
  // period = 1,000,000 / freq_hz
  // 实际平台通过 prescaler 调整, 此处用占位公式
  uint32_t period;
  if (freq_hz == 0) {
    // 频率=0 → 停止脉冲 (最长周期)
    period = 65535;
  } else {
    // TODO: 从 BSP 获取实际定时器时钟频率
    // 当前使用经验公式: period = 1000000 / freq_hz (1MHz tick base)
    period = 1000000UL / freq_hz;
    if (period < 2)   period = 2;     // 最小周期 (500kHz 上限)
    if (period > 65535) period = 65535; // 16-bit 定时器上限
  }

  me->base.period_target = (uint16_t)period;

  // ✅ Bug #2: 速度控制必须写定时器 LOAD 寄存器
  if (me->set_period && !me->ramping) {
    me->base.period_current = (uint16_t)period;
    me->set_period(me->pul_ctx, (uint16_t)period);  // ✅ Bug #4: ctx 被使用
  }
  // 如果是 ramp 模式, period_current 在 isr_tick 中逐步逼近 period_target
}

static void stepmotor_set_steps_ops(StepMotorBase *base, int32_t steps) {
  StepMotor *me = container_of(base, StepMotor, base);

  if (me->base.estop) return;

  // 方向判断: 正=正向, 负=反向
  if (steps >= 0) {
    me->base.dir = 1;
    me->base.steps_remaining = steps;
  } else {
    me->base.dir = -1;
    // Bug 规避: setposition(-n) — 取绝对值
    me->base.steps_remaining = -steps;
  }

  // 软限位检查 (Bug #7 规避)
  int32_t target_pos = me->base.pos + (int32_t)steps;
  if (me->base.pos_limit_hi > 0 && target_pos > me->base.pos_limit_hi) {
    // 截断到上限
    me->base.steps_remaining = me->base.pos_limit_hi - me->base.pos;
    if (me->base.steps_remaining < 0) me->base.steps_remaining = 0;
  }
  if (me->base.pos_limit_lo < 0 && target_pos < me->base.pos_limit_lo) {
    me->base.steps_remaining = me->base.pos - me->base.pos_limit_lo;
    if (me->base.steps_remaining < 0) me->base.steps_remaining = 0;
  }

  // 方向变化时设置 DIR 引脚
  if (me->base.dir != me->last_dir && me->set_dir) {
    me->set_dir(me->dir_ctx, (me->base.dir > 0));  // ✅ Bug #4: ctx 被使用
    me->last_dir = me->base.dir;
  }

  // 启动脉冲
  me->base.moving = (me->base.steps_remaining > 0);
  me->locked = false;
  if (me->base.moving && me->set_enable) {
    me->set_enable(me->pul_ctx, true);
  }
  // 重置锁止计时
  me->lock_cnt = 0;
}

static int32_t stepmotor_get_steps_ops(StepMotorBase *base) {
  return base->pos;
}

static void stepmotor_set_ramp_ops(StepMotorBase *base, uint32_t accel_hz_per_s) {
  StepMotor *me = container_of(base, StepMotor, base);

  if (accel_hz_per_s == 0) {
    // 无 ramp — 立刻跳变
    me->base.ramp_step = 0;
    me->base.period_current = me->base.period_target;
    me->ramping = false;
    return;
  }

  // ramp: 每 tick 的 period 变化量
  // accel 单位 = Hz/s, 换算为 period_delta/tick
  // 简化: ramp_step = max_period_change_per_tick
  // 实际值取决于定时器 tick 频率和脉冲频率
  me->base.ramp_step = (uint16_t)(accel_hz_per_s / 100);  // 粗略估算
  if (me->base.ramp_step < 1) me->base.ramp_step = 1;
}

static void stepmotor_set_phase_mode_ops(StepMotorBase *base,
                                          const StepPhaseTable *tbl) {
  StepMotor *me = container_of(base, StepMotor, base);
  if (tbl == NULL || tbl->num_steps == 0) return;

  me->base.phase_tbl = tbl;
  // 切换相序表时复位相位 (Bug #8: 不记忆锁止 phase)
  me->base.phase = 0;

  // 立即输出新表的第一个相位
  uint8_t phase_data = tbl->phases[0];
  bsp_step_write_phase(me->phase_pins, phase_data);
}

static void stepmotor_estop_ops(StepMotorBase *base) {
  StepMotor *me = container_of(base, StepMotor, base);

  // 停止脉冲
  if (me->set_enable) {
    me->set_enable(me->pul_ctx, false);
  }

  // 关所有相 (断电)
  uint8_t zero_phase = 0x00;
  bsp_step_write_phase(me->phase_pins, zero_phase);

  me->base.moving  = false;
  me->base.estop   = true;
  me->locked       = false;
  me->lock_cnt     = 0;
  me->base.steps_remaining = 0;
}

// ======== 每脉冲 ISR 更新 (核心) ========

void stepmotor_isr_tick(StepMotor *me) {
  if (me->base.estop) return;

  StepMotorBase *b = &me->base;

  // 1. 无剩余步数 + 非锁止 → 锁止计时
  if (b->steps_remaining == 0 && !me->locked) {
    me->lock_cnt++;
    // 100 tick 自动解锁, 关所有相
    // 防止电机静止时持续通电发热
    if (me->lock_cnt >= 100) {
      me->locked = true;
      if (me->set_enable) {
        me->set_enable(me->pul_ctx, false);
      }
      uint8_t zero_phase = 0x00;
      bsp_step_write_phase(me->phase_pins, zero_phase);
    }
    return;
  }

  if (b->steps_remaining == 0) return;  // 已锁止, 无操作

  // 2. 锁止退出: 恢复脉冲
  if (me->locked) {
    me->locked = false;
    me->lock_cnt = 0;
    if (me->set_enable) {
      me->set_enable(me->pul_ctx, true);
    }
  }

  // 3. 加减速 ramp (Bug #5 规避: period 渐进变化)
  if (me->ramping && b->ramp_step > 0) {
    if (b->period_current < b->period_target) {
      // 加速: period 减小
      b->period_current += b->ramp_step;
      if (b->period_current > b->period_target) {
        b->period_current = b->period_target;
      }
    } else if (b->period_current > b->period_target) {
      // 减速: period 增大
      b->period_current -= b->ramp_step;
      if (b->period_current < b->period_target) {
        b->period_current = b->period_target;
      }
    } else {
      me->ramping = false;
    }
    // 写硬件 (Bug #2: 同步到 LOAD 寄存器)
    if (me->set_period) {
      me->set_period(me->pul_ctx, b->period_current);
    }
  }

  // 4. 方向变化检测
  if (b->dir != me->last_dir) {
    if (me->set_dir) {
      me->set_dir(me->dir_ctx, (b->dir > 0));
    }
    me->last_dir = b->dir;
  }

  // 5. 推进相位
  const StepPhaseTable *tbl = b->phase_tbl;
  if (tbl == NULL || tbl->num_steps == 0) return;

  uint8_t phase_data = tbl->phases[b->phase];
  bsp_step_write_phase(me->phase_pins, phase_data);

  // 相位递增/递减
  if (b->dir > 0) {
    b->phase++;
    if (b->phase >= tbl->num_steps) b->phase = 0;
  } else {
    if (b->phase == 0) {
      b->phase = tbl->num_steps - 1;
    } else {
      b->phase--;
    }
  }

  // 6. 步数递减 + 位置更新
  b->steps_remaining--;
  b->pos += b->dir;  // Bug #6 规避: 步数递减在方向判断之后

  // 7. 完成检测
  if (b->steps_remaining == 0) {
    b->moving = false;
    me->ramping = false;
    // 开始锁止倒计时 (100 tick 后关相)
  }
}
