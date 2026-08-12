// 3 状态 PID 调节器 — 反计算抗饱和 + 位置回绕变体
//
// 来源: TI controlSUITE motor_control/math_blocks/v4.3/pid_reg3.h
// 翻译为 C-OOP 纯C float 版本
//
// 与 comp_pi_reg4 (四态 PI: 前馈 + 设定值滤波) 不同的侧重:
//   reg3 采用反计算 (back-calculation) 抗饱和: 输出被限幅时, 把饱和差
//   SatErr = Out - OutPreSat 反馈回积分器, 从根上抑制积分 windup
//
// 标准变体 (pid_reg3_run, 对应 PID_REG3_MACRO):
//   Err   = Ref - Fdb
//   Up    = Kp × Err
//   Ui   += Ki × Up + Kc × SatErr          // 积分作用在比例输出 + 反计算校正
//   Out   = sat(Up + Ui, OutMax, OutMin)
//   SatErr = Out - (Up + Ui)
//
// 位置变体 (pid_reg3_run_pos, 对应 PID_REG3_POS_MACRO):
//   增加微分项 Ud = Kd × (Up - Up1) (作用在比例输出差分), 适合位置控制
//   误差在 ±0.5 (pu) 处回绕: 用于角度等跨越 ±π 被归一化到 [0,1] 的反馈

#ifndef COMP_PID_REG3_H
#define COMP_PID_REG3_H

// ======== 配置 POD — 只读参数 ========
typedef struct {
  float kp;               // 比例增益 (Kp)
  float ki;               // 积分增益 (Ki) — 作用于比例输出
  float kc;               // 积分校正增益 (Kc) — 反计算抗饱和强度
                           //   典型值 0.5~1.0, 越大积分回退越快
  float kd;               // 微分增益 (Kd) — 仅位置变体使用
  float out_max;          // 输出上限
  float out_min;          // 输出下限
} PidReg3Cfg;

// ======== 运行时状态 ========
typedef struct {
  float ref;              // 输入: 参考值
  float fdb;              // 输入: 反馈值
  float err;              // 误差
  float up;               // 比例输出
  float ui;               // 积分输出
  float ud;               // 微分输出 (位置变体)
  float up1;              // 上一拍比例输出 (微分用)
  float out_pre_sat;      // 饱和前输出
  float out;              // 输出 (饱和后)
  float sat_err;          // 饱和差 = Out - OutPreSat
} PidReg3State;

// ======== 默认配置 — 与 TI PIDREG3_DEFAULTS 一致 ========
#define PID_REG3_CFG_DEFAULTS {  \
  1.3f,  /* kp      */           \
  0.02f, /* ki      */           \
  0.5f,  /* kc      */           \
  1.05f, /* kd      */           \
  1.0f,  /* out_max */           \
 -1.0f,  /* out_min */           \
}

// ======== 获取默认配置 ========
static inline PidReg3Cfg pid_reg3_cfg_default(void) {
  PidReg3Cfg cfg = PID_REG3_CFG_DEFAULTS;
  return cfg;
}

// ======== 初始化状态 — 清零 ========
static inline void pid_reg3_init(PidReg3State *me) {
  me->ref = 0.0f;
  me->fdb = 0.0f;
  me->err = 0.0f;
  me->up = 0.0f;
  me->ui = 0.0f;
  me->ud = 0.0f;
  me->up1 = 0.0f;
  me->out_pre_sat = 0.0f;
  me->out = 0.0f;
  me->sat_err = 0.0f;
}

// ======== 重置积分器 — 保留比例/微分状态 ========
static inline void pid_reg3_reset(PidReg3State *me) {
  me->ui = 0.0f;
  me->sat_err = 0.0f;
}

// ======== 标准变体单步计算 — ISR 热路径 (PI + 反计算抗饱和) ========
static inline float pid_reg3_run(PidReg3State *me, const PidReg3Cfg *cfg,
                                 float ref, float fdb) {
  me->ref = ref;
  me->fdb = fdb;
  me->err = ref - fdb;

  me->up = cfg->kp * me->err;                       // 比例
  me->ui = me->ui + cfg->ki * me->up + cfg->kc * me->sat_err;  // 积分 + 反计算
  me->out_pre_sat = me->up + me->ui;                // 饱和前输出

  if (me->out_pre_sat > cfg->out_max) {
    me->out = cfg->out_max;
  } else if (me->out_pre_sat < cfg->out_min) {
    me->out = cfg->out_min;
  } else {
    me->out = me->out_pre_sat;
  }

  me->sat_err = me->out - me->out_pre_sat;          // 饱和差 → 下拍反算积分
  me->up1 = me->up;
  return me->out;
}

// ======== 位置变体单步计算 — 误差回绕 + 微分 (PID) ========
//
// 适用: 位置/角度控制, 反馈被归一化到 [0,1] (如 0~1 对应 0~2π).
// 目标与当前角度跨越 0/1 边界时, 直接求差会得到接近 ±1 的大误差,
// 回绕后得到正确的短路径误差.
static inline float pid_reg3_run_pos(PidReg3State *me, const PidReg3Cfg *cfg,
                                     float ref, float fdb) {
  me->ref = ref;
  me->fdb = fdb;
  me->err = ref - fdb;

  // 误差回绕到 [-0.5, 0.5] — 跨越归一化角度边界的短路径
  if (me->err >= 0.5f) {
    me->err -= 1.0f;
  } else if (me->err <= -0.5f) {
    me->err += 1.0f;
  }

  me->up = cfg->kp * me->err;                       // 比例
  me->ui = me->ui + cfg->ki * me->up + cfg->kc * me->sat_err;  // 积分 + 反计算
  me->ud = cfg->kd * (me->up - me->up1);            // 微分 (作用在比例输出)
  me->out_pre_sat = me->up + me->ui + me->ud;       // 饱和前输出

  if (me->out_pre_sat > cfg->out_max) {
    me->out = cfg->out_max;
  } else if (me->out_pre_sat < cfg->out_min) {
    me->out = cfg->out_min;
  } else {
    me->out = me->out_pre_sat;
  }

  me->sat_err = me->out - me->out_pre_sat;          // 饱和差 → 下拍反算积分
  me->up1 = me->up;
  return me->out;
}

#endif  // COMP_PID_REG3_H
