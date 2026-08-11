// 数字滤波器库
//
// 提供一阶低通滤波器和二阶巴特沃斯低通滤波器。
// static inline 函数适合 ISR 热路径, 零调用开销。
// 一阶滤波器内置 dt 缓存优化 — dt 恒定时避免重复浮点除法。
//
// 来源: RM WEILAI_SuperCap

#ifndef COMP_FILTER_H
#define COMP_FILTER_H

#include <math.h>
#include <stdbool.h>
#include "comp_math.h"
#include "bsp_dsp.h"    // 硬件加速 sqrt/biquad (CMSIS-DSP / C2000 / 纯C回退)

// π 常量 (comp_math.h 已定义 M_2PI, 这里补 M_PI)
#ifndef M_PI
#define M_PI 3.14159265f
#endif

/* ======================== 一阶数字低通滤波器 ======================== */

typedef struct {
  float cut_freq_;    // 截止频率 (Hz)
  float last_out_;    // 上一次滤波输出
  float last_k_;      // 缓存的 k 系数 — dt 不变时跳过浮点除法
  float last_t_;      // 上次 dt 值 — 用于检测采样周期是否变化
  bool  initialized_; // 首帧标志 — true 表示已完成初始化
} LowPassFilter;

// 初始化一阶低通滤波器
static inline void
LowPassFilter_Init(LowPassFilter *me, float cut_freq) {
  me->cut_freq_    = cut_freq;
  me->last_out_    = 0.0f;
  me->last_k_      = 0.0f;
  me->last_t_      = 0.0f;
  me->initialized_ = false;
}

// 一阶低通滤波, dt 恒定时自动跳过浮点除法
//
// k = 2π·fc·dt / (1 + 2π·fc·dt)
// out = k·sample + (1-k)·last_out
static inline float __attribute__((always_inline))
LowPassFilter_Update(LowPassFilter *me, float sample, float dt) {
  // 首帧直接返回原值, 不滤波
  if (!me->initialized_) {
    me->last_out_    = sample;
    me->initialized_ = true;
    return sample;
  }

  // dt 缓存优化: ISR 中 dt 恒定, 只在首次或 dt 变化时重算 k
  float k;
  if (me->last_t_ == dt) {
    k = me->last_k_;                // 命中缓存, 跳过浮点除法
  } else {
    k = M_2PI * me->cut_freq_ * dt;
    k = k / (1.0f + k);
    me->last_k_ = k;               // 写入缓存
    me->last_t_ = dt;
  }

  float out = k * sample + (1.0f - k) * me->last_out_;
  me->last_out_ = out;
  return out;
}

// 重置滤波器状态 (跳变后重新收敛)
static inline void
LowPassFilter_Reset(LowPassFilter *me, float sample) {
  me->last_out_ = sample;
}

/* ======================= 二阶巴特沃斯低通滤波器 ======================= */

typedef struct {
  float cutoff_freq_;       // 截止频率 (Hz), ≤0 则直通 (b0=1 其余为0)

  // biquad Direct Form I 系数
  float a1_, a2_;           // 反馈系数
  float b0_, b1_, b2_;      // 前馈系数

  float delay_element_1_;   // DFI 延迟单元 z^-1
  float delay_element_2_;   // DFI 延迟单元 z^-2
} LowPassFilter2p;

// 初始化二阶巴特沃斯低通滤波器
//
// sample_freq: 采样频率 (Hz), cutoff_freq: -3dB 截止频率 (Hz)
// cutoff_freq ≤ 0 → 直通模式 (b0=1, 其余系数为 0)
static inline void
LowPassFilter2p_Init(LowPassFilter2p *me, float sample_freq, float cutoff_freq) {
  me->cutoff_freq_     = cutoff_freq;
  me->delay_element_1_ = 0.0f;
  me->delay_element_2_ = 0.0f;

  if (me->cutoff_freq_ <= 0.0f) {
    // 直通: 不过滤
    me->b0_ = 1.0f;  me->b1_ = 0.0f;  me->b2_ = 0.0f;
    me->a1_ = 0.0f;  me->a2_ = 0.0f;
    return;
  }

  // 双线性变换 (Tustin) 预畸变
  const float FR  = sample_freq / me->cutoff_freq_;
  const float OHM = tanf(M_PI / FR);
  const float C   = 1.0f + 2.0f * cosf(M_PI / 4.0f) * OHM + OHM * OHM;

  me->b0_ = OHM * OHM / C;
  me->b1_ = 2.0f * me->b0_;
  me->b2_ = me->b0_;

  me->a1_ = 2.0f * (OHM * OHM - 1.0f) / C;
  me->a2_ = (1.0f - 2.0f * cosf(M_PI / 4.0f) * OHM + OHM * OHM) / C;
}

// 二阶巴特沃斯滤波
static inline float __attribute__((always_inline))
LowPassFilter2p_Update(LowPassFilter2p *me, float sample) {
  // Direct Form I: w = sample - a1*w1 - a2*w2,  out = b0*w + b1*w1 + b2*w2
  float w = sample - me->delay_element_1_ * me->a1_
                   - me->delay_element_2_ * me->a2_;

  // 防 NaN/Inf 传播: 异常时退化为原值
  // isfinite 同时检测 Inf 和 NaN, 跨平台兼容性更好
  if (!isfinite(w)) {
    w = sample;
  }

  const float OUT = w            * me->b0_
                  + me->delay_element_1_ * me->b1_
                  + me->delay_element_2_ * me->b2_;

  // 推进延迟线
  me->delay_element_2_ = me->delay_element_1_;
  me->delay_element_1_ = w;

  return OUT;
}

// 重置滤波器 — 将稳态 DC 值注入延迟单元, 避免阶跃响应振铃
static inline float
LowPassFilter2p_Reset(LowPassFilter2p *me, float sample) {
  float dval = sample / (me->b0_ + me->b1_ + me->b2_);

  if (isfinite(dval)) {
    me->delay_element_1_ = dval;
    me->delay_element_2_ = dval;
  } else {
    me->delay_element_1_ = sample;
    me->delay_element_2_ = sample;
  }

  return LowPassFilter2p_Update(me, sample);
}

/* ======================== 指数移动平均 EMA ======================== */

// 来源: TI controlSUITE solar/v1.2/float (MATH_EMAVG_F)
// Out = Out + Multiplier * (In - Out)  — 等价于 y[k] = α·x[k] + (1-α)·y[k-1]
// Multiplier 常用范围: 0.001~0.1 (对应时间常数 τ = dt/α)
typedef struct {
  float in;               // 输入: 采样值
  float out;              // 输出: 滤波值
  float multiplier;       // 参数: 平滑系数 α = dt/τ
} MathEmavg;

#define MATH_EMAVG_DEFAULTS { 0.0f, 0.0f, 0.01f }

static inline void math_emavg_init(MathEmavg *me, float alpha) {
  me->in = 0.0f;
  me->out = 0.0f;
  me->multiplier = alpha;
}

static inline float math_emavg_run(MathEmavg *me, float sample) {
  me->in = sample;
  me->out = ((sample - me->out) * me->multiplier) + me->out;
  return me->out;
}

// 强制设值 (跳过平滑, 跳变后立即收敛)
static inline void math_emavg_force(MathEmavg *me, float value) {
  me->out = value;
}

/* ======================== 陷波滤波器 Notch (DF1, 二阶 IIR) ======================= */

// 来源: TI controlSUITE solar/v1.2/float (NOTCH_FLTR_F)
// H(z) = (B0 + B1·z^-1 + B2·z^-2) / (1 - A1·z^-1 - A2·z^-2)
// 典型用途: 滤除电网 PLL 中的 2 倍频纹波 (100/120Hz)
// 系数计算: notch_coeff_update(delta_T, omega, c2_damp, c1_damp, &coeff)

typedef struct {
  float b2;               // 分子 B2 (Z^-2)
  float b1;               // 分子 B1 (Z^-1)
  float b0;               // 分子 B0 (Z^0)
  float a2;               // 分母 A2 (Z^-2)
  float a1;               // 分母 A1 (Z^-1)
} NotchCoeff;

typedef struct {
  float out1;             // 输出延迟 u(k-1)
  float out2;             // 输出延迟 u(k-2)
  float in;               // 当前输入
  float in1;              // 输入延迟 x(k-1)
  float in2;              // 输入延迟 x(k-2)
  float out;              // 输出
} NotchVars;

#define NOTCH_COEFF_DEFAULTS { 0.0f, 0.0f, 1.0f, 0.0f, 0.0f }
#define NOTCH_VARS_DEFAULTS  { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }

static inline void notch_vars_init(NotchVars *me) {
  me->out1 = 0.0f; me->out2 = 0.0f;
  me->in = 0.0f; me->in1 = 0.0f; me->in2 = 0.0f;
  me->out = 0.0f;
}

// Direct Form 1: out = A1*out1 + A2*out2 + B0*in + B1*in1 + B2*in2
static inline float notch_filter_run(NotchVars *me, const NotchCoeff *coeff,
                                     float sample) {
  me->in = sample;

  me->out = coeff->a1 * me->out1 + coeff->a2 * me->out2
          + coeff->b0 * me->in   + coeff->b1 * me->in1 + coeff->b2 * me->in2;

  // 历史移位
  me->out2 = me->out1;
  me->out1 = me->out;
  me->in2 = me->in1;
  me->in1 = me->in;

  return me->out;
}

// 预计算陷波滤波器系数
//   dt:     采样周期 (s)
//   omega:  陷波频率角速度 (rad/s), 如电网 PLL 用 2*2π*50 = 628 rad/s (100Hz)
//   c2, c1: 阻尼系数 (c2 控制带宽, c1 控制深度, 典型值 c2=0.1, c1=0.01)
static inline void notch_coeff_update(float dt, float omega,
                                      float c2, float c1, NotchCoeff *coeff) {
  // 连续: H(s) = (s² + ω²) / (s² + 2·c1·ω·s + ω²)
  // 双线性离散化
  float t = dt;
  float w = omega;
  float w2 = w * w;
  float c = 2.0f / t;     // 双线性常数
  float c2_val = c * c;

  float den = c2_val + 2.0f * c1 * w * c + w2;

  coeff->b0 = (c2_val + w2) / den;
  coeff->b1 = (2.0f * w2 - 2.0f * c2_val) / den;
  coeff->b2 = (c2_val + w2) / den;
  coeff->a1 = (2.0f * c2_val - 2.0f * w2) / den;
  coeff->a2 = (c2_val - 2.0f * c1 * w * c + w2) / den;
}

#endif  // COMP_FILTER_H
