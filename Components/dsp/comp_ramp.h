// 斜坡发生器 — 开环相位/频率生成器
//
// 来源: TI controlSUITE solar/v1.2/float (RAMPGEN_F), motor_control (rampgen.h, rmp_cntl.h)
// 翻译为 HardC 纯C float 版本
//
// 用途:
//   RampGen  — 自由运行斜坡 (开环相位发生器, 用于 SPWM 调制波生成)
//   RampCtl  — 受控斜坡 (带加减速限制的给定积分器, 用于软起动/参考值平滑)
//
// 调用方式:
//   ramp_gen_tick(&ramp, freq_pu);   // ISR 中每控制周期调用, freq_pu=设定频率(标幺)
//   float phase = ramp.out;          // 0~1 标幺斜坡, 可用于 sinf(2*PI*phase)

#ifndef COMP_RAMP_H
#define COMP_RAMP_H

// ======================= RampGen (自由运行斜坡/相位累加器) =======================

// 斜坡以标幺值运行 (0~1 = 0~360°), 输出可直接送 sinf(2*PI*out)
typedef struct {
  float freq;             // 输入: 频率 (标幺, 相对于 ISR 频率)
  float angle;            // 状态: 当前相位 (标幺, 0~1)
  float step_max;         // 参数: 最大步长 (标幺, = f_base / f_ISR)
  float out;              // 输出: 当前相位 (标幺)
} RampGen;

#define RAMP_GEN_DEFAULTS { 1.0f, 0.0f, 0.0025f, 0.0f }

// 初始化
//   step_max = f_nom / f_isr  (如: 50Hz/20000Hz = 0.0025)
static inline void ramp_gen_init(RampGen *me, float step_max) {
  me->freq = 1.0f;
  me->angle = 0.0f;
  me->step_max = step_max;
  me->out = 0.0f;
}

// 单步: 角度累加 → 折叠 → 输出
// 返回当前相位 (0~1)
static inline float ramp_gen_tick(RampGen *me, float freq_pu) {
  me->freq = freq_pu;
  me->angle += me->step_max * freq_pu;

  // 折叠 (减法保留分数相位, 比归零法更精确)
  while (me->angle > 1.0f) {
    me->angle -= 1.0f;
  }

  me->out = me->angle;
  return me->out;
}

// 重置到 0
static inline void ramp_gen_reset(RampGen *me) {
  me->angle = 0.0f;
  me->out = 0.0f;
}

// ======================= RampCtl (受控斜坡, 加减速限制) =======================

// 用途: 对阶跃输入做斜率限制, 生成平滑过渡的给定值
// 上升斜率 (slew_up) 和下降斜率 (slew_down) 可独立设置
typedef struct {
  float target;           // 输入: 目标值
  float out;              // 输出: 平滑后的值
  float slew_up;          // 参数: 上升斜率 (单位/秒)
  float slew_down;        // 参数: 下降斜率 (单位/秒)
  float dt;               // 参数: 控制周期 (秒)
  bool  equal_slew;       // 参数: true=上升/下降同斜率 (用 slew_up)
} RampCtl;

#define RAMP_CTL_DEFAULTS { 0.0f, 0.0f, 1.0f, 1.0f, 0.001f, true }

static inline void ramp_ctl_init(RampCtl *me, float slew_up, float slew_down,
                                 float dt, bool equal_slew) {
  me->target = 0.0f;
  me->out = 0.0f;
  me->slew_up = slew_up;
  me->slew_down = slew_down;
  me->dt = dt;
  me->equal_slew = equal_slew;
}

// 单步: 向 target 平滑逼近
static inline float ramp_ctl_tick(RampCtl *me, float target) {
  me->target = target;
  float error = target - me->out;
  float step;

  if (error > 0.0f) {
    // 需要上升
    step = me->slew_up * me->dt;
    if (error > step) {
      me->out += step;
    } else {
      me->out = target;  // 到达目标
    }
  } else if (error < 0.0f) {
    // 需要下降
    step = (me->equal_slew ? me->slew_up : me->slew_down) * me->dt;
    if (-error > step) {
      me->out -= step;
    } else {
      me->out = target;
    }
  }
  // error == 0: 不动

  return me->out;
}

// 强制设值 (跳过斜坡, 如急停时清零)
static inline void ramp_ctl_force(RampCtl *me, float value) {
  me->out = value;
  me->target = value;
}

#endif  // COMP_RAMP_H
