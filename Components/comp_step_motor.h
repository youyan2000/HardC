#ifndef COMP_STEP_MOTOR_H
#define COMP_STEP_MOTOR_H

// 步进电机平台层 — 抽象基类 (独立于 DC 电机 comp_motor)
// 来源: Car_Control_Study_Report §23.3 (理想 vtable 设计)
//       + Car_Control_Study_Report §18~§22 (三版 Bug 清单全部规避)
//       + 3507_2026_eugene motor_step.h/c (OOP 版步进电机)
//
// 与 comp_motor.h 的关系:
//   - comp_motor: DC 电机 (3-ops: write/encode/read, 有编码器反馈)
//   - comp_step_motor: 步进电机 (6-ops, 开环脉冲控制)
//   两者独立, 不共享 vtable — 因为语义完全不同 (报告 §10.1, §23.2)
//
// Bug 规避 (报告 §22):
//   - ✅ 状态全在结构体成员, 无 static 局部变量 (Bug #1)
//   - ✅ 速度控制通过 set_rate → 定时器 LOAD 寄存器 (Bug #2)
//   - ✅ ctx 必须被使用, 不能 (void)ctx (Bug #4)
//   - ✅ 有软限位 (Bug #7)
//   - ✅ 加减速占位 (Bug #5)
//   - ✅ 失步检测占位 (Bug #9)
//   - ✅ 重启不会错相, 初始化时 phase=0 (Bug #8)

#include <stdint.h>
#include <stdbool.h>

// ======== 相序表 (可注入, 支持不同驱动模式) ========
// 半步进 8 拍: {A, AC, C, CB, B, BD, D, DA}
// 全步进 4 拍: {A, C, B, D}  (A→C→B→D 即 MyFinal_Work 线束顺序)
// 微步进需要硬件支持 (A4988/DRV8825 等驱动芯片)
typedef struct {
  uint8_t phases[8];   // 相位序列 (每字节 bit0=A, bit1=B, bit2=C, bit3=D)
  uint8_t num_steps;   // 相位数: 4=全步进, 8=半步进
} StepPhaseTable;

// 预设: 4 相全步进 A→C→B→D (匹配 MyFinal_Work 线束重映射)
#define STEP_PHASE_TABLE_FULL_4  \
  { .phases = {0x01, 0x04, 0x02, 0x08}, .num_steps = 4 }

// 预设: 8 拍半步进 A→AC→C→CB→B→BD→D→DA
#define STEP_PHASE_TABLE_HALF_8  \
  { .phases = {0x01, 0x05, 0x04, 0x06, 0x02, 0x0A, 0x08, 0x09}, .num_steps = 8 }

// ======== 前向声明 ========
typedef struct StepMotorBase StepMotorBase;

// ======== 虚函数指针类型 (6-ops vtable) ========

// 设置脉冲频率 (Hz) → 子类内部转为定时器 period
typedef void (*step_set_rate_fn)(StepMotorBase *me, uint32_t freq_hz);

// 设置目标步数 (绝对移动)
typedef void (*step_set_steps_fn)(StepMotorBase *me, int32_t steps);

// 读取累计步数 (开环位置)
typedef int32_t (*step_get_steps_fn)(StepMotorBase *me);

// 设置加减速 (Hz/s, 0=无加减速/立刻跳变)
typedef void (*step_set_ramp_fn)(StepMotorBase *me, uint32_t accel_hz_per_s);

// 设置驱动模式 (全步进/半步进/微步进)
typedef void (*step_set_phase_mode_fn)(StepMotorBase *me, const StepPhaseTable *tbl);

// 急停 — 立即断电, 关所有相
typedef void (*step_estop_fn)(StepMotorBase *me);

// ======== 虚函数表 ========
typedef struct {
  step_set_rate_fn       set_rate;
  step_set_steps_fn      set_steps;
  step_get_steps_fn      get_steps;
  step_set_ramp_fn       set_ramp;
  step_set_phase_mode_fn set_phase_mode;
  step_estop_fn          emergency_stop;
} StepMotorOps;

// ======== 基类结构体 ========
struct StepMotorBase {
  const StepMotorOps *ops;        // 虚函数表
  const StepPhaseTable *phase_tbl; // 当前相序表

  int32_t   steps_remaining;      // 剩余步数 (递减到 0 停止)
  int32_t   pos;                  // 累计绝对位置 (步数, 不限增长但有限位检查)
  int32_t   pos_limit_lo;         // 软限位下限 (负值, 0=不限)
  int32_t   pos_limit_hi;         // 软限位上限 (正值, 0=不限)

  uint16_t  period_target;        // 目标定时器周期 (set_rate 后的最终值)
  uint16_t  period_current;       // 当前定时器周期 (ramp 中间值)
  uint16_t  ramp_step;            // 每 tick 加减量 (0=无 ramp)

  uint8_t   phase;                // 当前相位索引 (0..phase_tbl->num_steps-1)
  int8_t    dir;                  // 当前方向: +1=正向, -1=反向

  bool      moving;               // 是否正在移动 (steps_remaining > 0)
  bool      estop;                // 急停标志: true=已断电, 需手动复位
};

// ======== 基类构造 ========

// 初始化基类: 绑定相序表 + 软限位 + 初始 phase=0
void stepmotor_base_init(StepMotorBase *me, const StepPhaseTable *phase_tbl,
                          int32_t limit_lo, int32_t limit_hi);

// ======== 分发函数 (inline, 委托 ops) ========

static inline void stepmotor_set_rate(StepMotorBase *me, uint32_t freq_hz) {
  if (me->ops->set_rate) me->ops->set_rate(me, freq_hz);
}

static inline void stepmotor_set_steps(StepMotorBase *me, int32_t steps) {
  if (me->ops->set_steps) me->ops->set_steps(me, steps);
}

static inline int32_t stepmotor_get_steps(StepMotorBase *me) {
  if (me->ops->get_steps) return me->ops->get_steps(me);
  return me->pos;
}

static inline void stepmotor_set_ramp(StepMotorBase *me, uint32_t accel) {
  if (me->ops->set_ramp) me->ops->set_ramp(me, accel);
}

static inline void stepmotor_set_phase_mode(StepMotorBase *me,
                                              const StepPhaseTable *tbl) {
  if (me->ops->set_phase_mode) me->ops->set_phase_mode(me, tbl);
}

static inline void stepmotor_estop(StepMotorBase *me) {
  if (me->ops->emergency_stop) me->ops->emergency_stop(me);
}

#endif
