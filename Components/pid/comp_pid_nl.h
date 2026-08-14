// 非线性 PID 控制器 — 各通路独立幂律整形 (DCL 库 NLPID)
//
// 来源: TI C2000Ware Digital Power SDK c2000ware/libraries/control/DCL/c28/include/DCL_NLPID.h
// 翻译为 HardC 纯C float 版本
//
// 与 comp_pid 线性 PID 的区别: 三个通路 (P/I/D) 各自带非线性整形函数
//   f(e) = sign(e)·|e|^α      (|e| > δ, 幂律区)
//   f(e) = e·γ                 (|e| ≤ δ, 线性区, γ 为小误差增益)
// 参数含义:
//   α (alpha) — 幂律指数 0<α<2: α<1 低误差放大(快), α>1 低误差衰减(慢)
//   δ (delta) — 线性化范围: |e|>δ 进入幂律区
//   γ (gamma) — 线性区增益: 默认 = δ^(α-1) 保证两区边界连续 (可用 nl_pid_gamma_from_delta 计算)
// 用途: 小误差高增益 / 大误差降增益的强鲁棒控制 (电源启动、负载突变)
//
// 并行式结构: 积分带抗饱和 (i16 标志), 微分带二阶滤波 (c1/c2)
// 输入/输出归一化 ±1 (pu), 误差在 run 内折半预处理 (与 DCL 一致)

#ifndef COMP_PID_NL_H
#define COMP_PID_NL_H

#include <math.h>
#include "comp_math.h"

// ======================= 配置 POD — 只读参数 =======================

typedef struct {
  float kp;               // 线性比例增益
  float ki;               // 线性积分增益
  float kd;               // 线性微分增益

  float alpha_p;          // P 通路幂律指数 (0<α<2)
  float alpha_i;          // I 通路幂律指数
  float alpha_d;          // D 通路幂律指数

  float delta_p;          // P 通路线性化范围
  float delta_i;          // I 通路线性化范围
  float delta_d;          // D 通路线性化范围

  float gamma_p;          // P 通路线性区增益
  float gamma_i;          // I 通路线性区增益
  float gamma_d;          // D 通路线性区增益

  float c1;               // D 通路滤波器系数 1
  float c2;               // D 通路滤波器系数 2

  float out_max;          // 输出上限
  float out_min;          // 输出下限
} NlPidCfg;

// ======================= 运行时状态 =======================

typedef struct {
  float ref;              // 输入: 参考值
  float fdb;              // 输入: 反馈值

  float d2;               // D 通路滤波器中间状态 1
  float d3;               // D 通路滤波器中间状态 2
  float i7;               // I 通路积分状态
  float i16;              // 抗饱和标志 (1=未饱和, 0=输出已钳位)

  float up;               // 比例输出 (整形后)
  float ui;               // 积分输出
  float ud;               // 微分输出 (滤波后)
  float out_pre_sat;      // 饱和前输出
  float out;              // 输出 (饱和后)
} NlPidState;

// ======================= 默认配置 — 与 DCL NLPID_DEFAULTS 一致 =======================

#define NL_PID_CFG_DEFAULTS {  \
  1.0f, 0.0f, 0.0f,           /* kp, ki, kd            */ \
  1.0f, 1.0f, 1.0f,           /* alpha_p/i/d (线性)    */ \
  0.1f, 0.1f, 0.1f,           /* delta_p/i/d           */ \
  1.0f, 1.0f, 1.0f,           /* gamma_p/i/d           */ \
  1.0f, 0.0f,                 /* c1, c2 (无滤波)       */ \
  1.0f, -1.0f,                /* out_max, out_min      */ \
}

// ======================= 辅助: γ/δ 换算 =======================

// γ = δ^(α-1) — 由 δ 计算线性区增益, 保证整形函数在两区边界连续
static inline float nl_pid_gamma_from_delta(float alpha, float delta) {
  return powf(delta, (alpha - 1.0f));
}

// δ = γ^(1/(α-1)) — 由 γ 反推线性化范围
static inline float nl_pid_delta_from_gamma(float alpha, float gamma) {
  return powf(gamma, (1.0f / (alpha - 1.0f)));
}

// 设置微分滤波器带宽 (双线性: c1/c2 由截止频率 fc 换算)
//   τ = 1/(2π·fc) ; c1 = 2/(T+2τ) ; c2 = (T−2τ)/(T+2τ)
static inline void nl_pid_set_filter_bw(NlPidCfg *cfg, float fc, float dt) {
  float tau = 1.0f / (M_2PI * fc);

  cfg->c1 = 2.0f / (dt + 2.0f * tau);
  cfg->c2 = (dt - 2.0f * tau) / (dt + 2.0f * tau);
}

// ======================= 初始化/复位 =======================

static inline NlPidCfg nl_pid_cfg_default(void) {
  NlPidCfg cfg = NL_PID_CFG_DEFAULTS;
  return cfg;
}

static inline void nl_pid_init(NlPidState *me) {
  me->ref = 0.0f;
  me->fdb = 0.0f;
  me->d2 = 0.0f;
  me->d3 = 0.0f;
  me->i7 = 0.0f;
  me->i16 = 1.0f;   // 未饱和
  me->up = 0.0f;
  me->ui = 0.0f;
  me->ud = 0.0f;
  me->out_pre_sat = 0.0f;
  me->out = 0.0f;
}

// 复位积分与滤波状态 (含抗饱和标志复位, 与 DCL 一致)
static inline void nl_pid_reset(NlPidState *me) {
  me->d2 = 0.0f;
  me->d3 = 0.0f;
  me->i7 = 0.0f;
  me->i16 = 1.0f;
}

// ======================= 单步运行 =======================

// 并行式非线性 PID 单步
//   clamp_flag — 外部输出钳位标志 (1=正常, 0=外部钳位 → 停积分)
//   返回: 限幅后输出
static inline float nl_pid_run(NlPidState *me, const NlPidCfg *cfg,
                               float ref, float fdb, float clamp_flag) {
  me->ref = ref;
  me->fdb = fdb;

  // ---- 预处理: 误差折半 (输入归一化 ±1 时保持整形有效域) ----
  float v1 = (ref - fdb) * 0.5f;
  float v2 = (v1 < 0.0f) ? -1.0f : 1.0f;   // 符号
  float v3 = MATH_ABS(v1);                    // 幅值

  // ---- 非线性整形: |e|>δ → 幂律 ; |e|≤δ → 线性 ----
  float v4 = (v3 > cfg->delta_p)
             ? (v2 * powf(v3, cfg->alpha_p)) : (v1 * cfg->gamma_p);
  float v5 = (v3 > cfg->delta_i)
             ? (v2 * powf(v3, cfg->alpha_i)) : (v1 * cfg->gamma_i);
  float v9 = (v3 > cfg->delta_d)
             ? (v2 * powf(v3, cfg->alpha_d)) : (v1 * cfg->gamma_d);

  // ---- 积分通路 (抗饱和: i16=0 停止积分) ----
  me->i7 = (v5 * cfg->kp * cfg->ki * me->i16) + me->i7;
  me->ui = me->i7;

  // ---- 微分通路 (二阶滤波器, 抑制噪声放大) ----
  float v10 = v9 * cfg->kd * cfg->c1;
  float v12 = v10 - me->d2 - me->d3;
  me->d2 = v10;
  me->d3 = v12 * cfg->c2;
  me->ud = v12;

  // ---- 输出合成 + 限幅 ----
  float v13 = cfg->kp * (v4 + v12) + me->i7;
  me->up = cfg->kp * v4;
  me->out_pre_sat = v13;

  if (v13 > cfg->out_max) {
    me->out = cfg->out_max;
  } else if (v13 < cfg->out_min) {
    me->out = cfg->out_min;
  } else {
    me->out = v13;
  }

  // 抗饱和标志: 输出被钳位 → 停积分 (v13==out 比较浮点相等在此可行, 因 out 直接取自 v13 的钳位分支)
  me->i16 = (me->out == v13) ? (1.0f * clamp_flag) : 0.0f;

  return me->out;
}

#endif  // COMP_PID_NL_H
