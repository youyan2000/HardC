// 4 状态 PI 调节器 — 带设定值前馈 + 设定值滤波
//
// 在标准 PI (比例+积分) 基础上增加两条路径, 显著改善指令跟踪性能:
//   状态 1 — 设定值滤波: 对目标参考做一阶低通, 抑制阶跃跳变引起的微分冲击
//   状态 2 — 比例路径 P:  Kp * (sp_filtered - fbk), 即时响应偏差
//   状态 3 — 积分路径 I:  Ki * ∫(sp_filtered - fbk) dt, 消除稳态误差
//   状态 4 — 前馈路径 FF: Kff * sp_raw, 绕过滤波器直接驱动, 加快大范围跟踪
//
// 算法流程: sp_filter → P + I (含抗积分饱和) + FF → 输出限幅
// 抗积分饱和: 输出被限幅时冻结积分器, 避免 windup
//
// 来源: TI controlSUITE motor_control/math_blocks/v4.3/pi_reg4.h
//       (原版为标准 PI, 本版扩展了前馈 + 设定值滤波)

#ifndef COMP_PI_REG4_H
#define COMP_PI_REG4_H

#include <stdbool.h>
#include "comp_math.h"

// ======== 配置 POD — 只读参数, 可放入 Flash ========
typedef struct {
  float kp;               // 比例增益 (Kp), 单位: output/error
  float ki;               // 积分增益 (Ki), 单位: output/(error·s)
                           //   零阶保持离散化: Ki_dt = Ki * dt
  float kff;              // 前馈增益 (Kff), 单位: output/setpoint
                           //   设为 1.0 可让输出快速跟踪设定值指令
  float dt;               // 控制周期 (秒), 如 0.0001 = 100us (10kHz)
  float out_max;          // 输出上限
  float out_min;          // 输出下限
  float sp_fc;            // 设定值滤波器截止频率 (Hz)
                           //   0 = 禁用滤波 (直通), 典型值 10~50Hz
} PiReg4Cfg;

// ======== 运行时状态 — 可修改, 放 RAM ========
typedef struct {
  float integral;         // 积分累加器 (总累计 = Σ Ki·error·dt)
  float sp_filtered;      // 滤波后的设定值
  float last_output;      // 上一次输出 (用于饱和检测)
  bool  initialized;      // 首帧标志 — true 表示已完成初始化
} PiReg4State;

// ======== 默认配置 — 安全初始值 ========
//
// Kp=1, Ki=0 (纯比例), Kff=0 (无前馈), 输出不限幅, 不滤波
#define PI_REG4_CFG_DEFAULTS {  \
  1.0f, /* kp       */         \
  0.0f, /* ki       */         \
  0.0f, /* kff      */         \
  0.001f,/* dt (1ms)*/         \
  1.0f, /* out_max  */         \
 -1.0f, /* out_min  */         \
  0.0f  /* sp_fc, 0=直通 */    \
}

// ======== 获取默认配置 ========
static inline PiReg4Cfg pi_reg4_cfg_default(void) {
  PiReg4Cfg cfg = PI_REG4_CFG_DEFAULTS;
  return cfg;
}

// ======== 初始化状态 — 清零 ========
static inline void pi_reg4_init(PiReg4State *me) {
  me->integral    = 0.0f;
  me->sp_filtered = 0.0f;
  me->last_output = 0.0f;
  me->initialized = false;
}

// ======== 重置积分器 — 保留滤波器状态 ========
//
// 适用场景: 模式切换 (手动↔自动) / 急停后恢复, 避免积分器残留
static inline void pi_reg4_reset(PiReg4State *me) {
  me->integral = 0.0f;
}

// ======== 单步计算 — ISR 热路径 ========
//
// 参数:
//   me        — 运行时状态指针
//   cfg       — 配置参数指针 (只读)
//   setpoint  — 目标指令 (sp_raw)
//   feedback  — 当前采样值 (fbk)
//
// 返回: 限幅后的控制输出
//
// 内部流程:
//   1. 设定值一阶低通滤波 (sp_fc > 0 时生效)
//   2. P 路径 = Kp * (sp_filtered - fbk)
//   3. I 路径 = 累加 Ki * dt * (sp_filtered - fbk), 含抗积分饱和
//   4. FF 路径 = Kff * sp_raw (前馈直通, 不经滤波器)
//   5. sum = P + I + FF → 限幅到 [out_min, out_max]
static inline float pi_reg4_run(PiReg4State *me, const PiReg4Cfg *cfg,
                                float setpoint, float feedback) {
  // --- 设定值滤波 (一阶低通) ---
  // 首帧直接用原始值, 避免从 0 收敛的延迟
  float sp_f;
  if (!me->initialized) {
    me->sp_filtered = setpoint;
    me->initialized = true;
    sp_f = setpoint;
  } else if (cfg->sp_fc <= 0.0f) {
    // 截止频率 ≤ 0 → 直通, 不滤波
    sp_f = setpoint;
    me->sp_filtered = setpoint;
  } else {
    // 一阶 IIR: y[k] = α·x[k] + (1-α)·y[k-1]
    // α = 2π·fc·dt / (1 + 2π·fc·dt)
    float alpha = M_2PI * cfg->sp_fc * cfg->dt;   // 2π·fc·dt
    alpha = alpha / (1.0f + alpha);
    sp_f = alpha * setpoint + (1.0f - alpha) * me->sp_filtered;
    me->sp_filtered = sp_f;
  }

  // --- 误差计算 (基于滤波后的设定值) ---
  float error = sp_f - feedback;

  // --- 比例路径 P ---
  float p_term = cfg->kp * error;

  // --- 积分路径 I (含抗积分饱和) ---
  // 用边界检查判断饱和: 上一拍输出已在限幅边界 且 误差方向仍朝边界外
  // 若饱和则不累加积分, 防止 windup
  float i_term = me->integral;
  bool saturated = (cfg->out_max <= cfg->out_min)
                || (me->last_output >= cfg->out_max && error > 0.0f)
                || (me->last_output <= cfg->out_min && error < 0.0f);

  if (!saturated) {
    // 正常累加: ΔI = Ki · dt · error
    me->integral += cfg->ki * cfg->dt * error;
  }
  // 饱和时不累加, 积分器冻结 (抗积分饱和)
  i_term = me->integral;

  // --- 前馈路径 FF ---
  // 使用原始设定值 (不经滤波器), 保证快速响应
  float ff_term = cfg->kff * setpoint;

  // --- 合并并限幅 ---
  float output = p_term + i_term + ff_term;

  // 输出限幅
  if (output > cfg->out_max) {
    output = cfg->out_max;
  } else if (output < cfg->out_min) {
    output = cfg->out_min;
  }

  // 额外: 若积分项导致越界, 回退积分器 (clamping anti-windup)
  if (output == cfg->out_max && i_term > 0.0f) {
    me->integral = cfg->out_max - p_term - ff_term;
    if (me->integral < 0.0f) me->integral = 0.0f;
  } else if (output == cfg->out_min && i_term < 0.0f) {
    me->integral = cfg->out_min - p_term - ff_term;
    if (me->integral > 0.0f) me->integral = 0.0f;
  }

  me->last_output = output;
  return output;
}

#endif  // COMP_PI_REG4_H
