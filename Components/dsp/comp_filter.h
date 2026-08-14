// 数字滤波器库 — 一阶/二阶 IIR 家族 (低通/高通/带通/带阻/陷波)
//
// 分两类 API:
//   1. 便捷运行时滤波器 (PascalCase 旧 API, 参数即时计算):
//      一阶低通 LowPassFilter (dt 制, 内置 dt 缓存优化)
//      二阶巴特沃斯低通 LowPassFilter2p
//      指数移动平均 MathEmavg
//      陷波 Notch (阻尼参数 c2/c1 设计)
//   2. 频率指标 → 系数 设计套件 (snake_case 新 API):
//      iir_filter_design 统一设计器 + 便捷包装 — 巴特沃斯 (通带最大平坦) /
//      切比雪夫 I 型 (通带等纹波, 滚降更陡), 覆盖 LPF/HPF/BPF/BSF/Notch,
//      系数与 comp_iir.h DF22 布局兼容, 可导出到 bsp_dsp.h biquad 硬件加速。
//
// static inline 函数适合 ISR 热路径, 零调用开销。
//
// 既有滤波器 + 经典模拟原型 (Butterworth/Chebyshev)
//       与 RBJ Audio EQ Cookbook (BPF/BSF/Notch)

#ifndef COMP_FILTER_H
#define COMP_FILTER_H

#include <math.h>
#include <stdbool.h>
#include "comp_math.h"
#include "bsp_dsp.h"    // 硬件加速 sqrt/biquad (CMSIS-DSP / C2000 / 纯C回退)
#include "comp_iir.h"   // DF22 布局 — 设计套件系数可直接喂 iir_df22_run

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

/* ============ IIR biquad 设计套件 — 频率指标 → 系数 (Butterworth/Chebyshev) ============ */

// 设计方法:
//   巴特沃斯 (Butterworth)  — 通带最大平坦, fc = -3dB 点
//   切比雪夫 I 型 (Chebyshev) — 通带等纹波 (ripple_db), 滚降更陡; fc = 通带边缘 (纹波上限, 非 -3dB)
// 两者 fc 含义不同, 使用前请确认。
//
// 带通/带阻/陷波恒用 Q 谐振器 (RBJ Audio EQ Cookbook), fc=中心频率, bw=-3dB 带宽 (Q=fc/bw),
// approx 参数对其无效。单 biquad 无法实现切比雪夫带通 (级联留作后续)。
//
// 系数约定与 comp_iir.h IirDf22 完全一致:
//   H(z) = (b0 + b1·z^-1 + b2·z^-2) / (1 + a1·z^-1 + a2·z^-2), a1/a2 直接加
//   DF2 递归: v = in - a1·x1 - a2·x2;  out = b0·v + b1·x1 + b2·x2
// 可直接喂 iir_df22_run, 或经 iir_biquad_to_bsp_biquad 导出到 bsp_dsp.h 硬件加速。
//
// 来源: 经典模拟原型 (Butterworth / Chebyshev I 型) + 双线性变换; RBJ Audio EQ Cookbook (BPF/BSF/Notch)

typedef enum {
  IIR_FILTER_LPF,    // 低通 — fc = 截止频率
  IIR_FILTER_HPF,    // 高通 — fc = 截止频率
  IIR_FILTER_BPF,    // 带通 — fc = 中心频率, bw = 带宽
  IIR_FILTER_BSF,    // 带阻 — fc = 中心频率, bw = 带宽
  IIR_FILTER_NOTCH,  // 陷波 — fc = 陷波中心, bw = 带宽 (窄带带阻, 高 Q)
} IirFilterKind;

typedef enum {
  IIR_APPROX_BUTTERWORTH,  // 通带最大平坦, fc = -3dB 点
  IIR_APPROX_CHEBYSHEV,    // 通带等纹波, 滚降更陡; fc = 通带边缘
} IirApprox;

typedef struct {
  float b0;       // 分子系数 B0
  float b1;       // 分子系数 B1
  float b2;       // 分子系数 B2
  float a1;       // 分母系数 A1 (直接加)
  float a2;       // 分母系数 A2 (直接加)
  float x1;       // DF2 内部状态延迟 1
  float x2;       // DF2 内部状态延迟 2
} IirBiquad;

typedef struct {
  float b0;       // 分子系数 B0
  float b1;       // 分子系数 B1
  float a1;       // 分母系数 A1 (直接加)
  float x1;       // DF2 内部状态延迟
} IirFirst;

#define IIR_BIQUAD_DEFAULTS { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }

// asinh 可移植实现 — C2000 工具链可能缺 asinhf
static inline float iir_asinh(float x) {
  return logf(x + sqrtf(x * x + 1.0f));
}

// 统一设计器 — 从频率指标计算二阶 biquad 系数
//
//   me:        输出 biquad (系数写入, 状态清零)
//   kind:      IIR_FILTER_LPF/HPF/BPF/BSF/NOTCH
//   approx:    IIR_APPROX_BUTTERWORTH/CHEBYSHEV (BPF/BSF/NOTCH 忽略, 恒用 Q 谐振器)
//   fs:        采样率 (Hz)
//   fc:        截止频率 (LPF/HPF) 或 中心频率 (BPF/BSF/NOTCH)
//   bw:        -3dB 带宽 (Hz), 仅 BPF/BSF/NOTCH 使用 (Q = fc/bw)
//   ripple_db: 切比雪夫通带纹波 (dB), 仅 CHEBYSHEV 使用; ≤0 时默认 0.5dB
//
// 非法参数 → 直通 (b0=1 其余 0): fs≤0 / fc≤0 / fc≥fs/2 / BPF/BSF/NOTCH 且 bw≤0
static inline void iir_filter_design(IirBiquad *me, IirFilterKind kind,
                                     IirApprox approx, float fs, float fc,
                                     float bw, float ripple_db) {
  me->x1 = 0.0f;
  me->x2 = 0.0f;
  me->b0 = 1.0f;  me->b1 = 0.0f;  me->b2 = 0.0f;
  me->a1 = 0.0f;  me->a2 = 0.0f;

  bool is_band = (kind == IIR_FILTER_BPF || kind == IIR_FILTER_BSF ||
                  kind == IIR_FILTER_NOTCH);
  if (fs <= 0.0f || fc <= 0.0f || fc >= 0.5f * fs) return;
  if (is_band && bw <= 0.0f) return;

  if (is_band) {
    // RBJ 带通/带阻/陷波 — 带通中心增益 0dB, 带阻/陷波中心深陷
    const float w0    = M_2PI * fc / fs;
    const float alpha = sinf(w0) / (2.0f * fc / bw);   // α = sin(ω0)/(2Q), Q = fc/bw
    const float inv   = 1.0f / (1.0f + alpha);
    const float cosw  = cosf(w0);
    if (kind == IIR_FILTER_BPF) {
      me->b0 =  alpha * inv;
      me->b1 =  0.0f;
      me->b2 = -alpha * inv;
    } else {
      me->b0 = inv;
      me->b1 = -2.0f * cosw * inv;
      me->b2 = inv;
    }
    me->a1 = -2.0f * cosw * inv;
    me->a2 = (1.0f - alpha) * inv;
    return;
  }

  // LPF/HPF — 巴特沃斯或切比雪夫 I 型, 双线性变换 (频率预畸变 Ω = tan(π·fc/fs))
  const float om = tanf(M_PI * fc / fs);

  if (approx == IIR_APPROX_BUTTERWORTH) {
    // 2 阶巴特沃斯: 极点 |s| = Ω, 角度 ±π/4 (与既有 LowPassFilter2p_Init 系数等价)
    const float sq2 = 2.0f * cosf(M_PI / 4.0f);   // √2
    const float c   = 1.0f + sq2 * om + om * om;
    if (kind == IIR_FILTER_LPF) {
      const float b0 = om * om / c;
      me->b0 = b0;
      me->b1 = 2.0f * b0;
      me->b2 = b0;
    } else {
      const float b0 = 1.0f / c;
      me->b0 = b0;
      me->b1 = -2.0f * b0;
      me->b2 = b0;
    }
    me->a1 = 2.0f * (om * om - 1.0f) / c;
    me->a2 = (1.0f - sq2 * om + om * om) / c;
    return;
  }

  // 切比雪夫 I 型 — 2 阶极点置于椭圆: s = -a ± jb, P = a² + b²
  if (ripple_db <= 0.0f) ripple_db = 0.5f;
  const float eps     = sqrtf(powf(10.0f, ripple_db / 10.0f) - 1.0f);  // 纹波因子
  const float v       = 0.5f * iir_asinh(1.0f / eps);
  const float ev      = expf(v);           // sinh/cosh 用 expf, 可移植
  const float sinh_v  = 0.5f * (ev - 1.0f / ev);
  const float cosh_v  = 0.5f * (ev + 1.0f / ev);
  const float a       = sinh_v * sinf(M_PI / 4.0f);
  const float b       = cosh_v * cosf(M_PI / 4.0f);
  const float p       = a * a + b * b;

  if (kind == IIR_FILTER_LPF) {
    // 偶数阶 DC 增益 = 通带纹波下限 → 归一 K = P/√(1+ε²) (K=P 会在中频鼓包)
    const float k     = p / sqrtf(1.0f + eps * eps);
    const float c     = 1.0f + 2.0f * a * om + p * om * om;
    const float b0    = k * om * om / c;
    me->b0 = b0;
    me->b1 = 2.0f * b0;
    me->b2 = b0;
    me->a1 = 2.0f * (p * om * om - 1.0f) / c;
    me->a2 = (1.0f - 2.0f * a * om + p * om * om) / c;
  } else {
    // 高通必须从模拟原型推导 (z→-z 变换会把截止翻到 fs/2-fc, 不可用):
    // 原型 H(s) = K'·s²/(s² + 2a'·s + P'),  a' = a/P, P' = 1/P, K' = 1/√(1+ε²)
    const float ap     = a / p;
    const float pp     = 1.0f / p;
    const float kp     = 1.0f / sqrtf(1.0f + eps * eps);
    const float ch     = 1.0f + 2.0f * ap * om + pp * om * om;
    const float b0     = kp / ch;
    me->b0 = b0;
    me->b1 = -2.0f * b0;
    me->b2 = b0;
    me->a1 = 2.0f * (pp * om * om - 1.0f) / ch;
    me->a2 = (1.0f - 2.0f * ap * om + pp * om * om) / ch;
  }
}

// 便捷包装 — 低通
static inline void iir_lpf_design(IirBiquad *me, IirApprox approx, float fs,
                                  float fc, float ripple_db) {
  iir_filter_design(me, IIR_FILTER_LPF, approx, fs, fc, 0.0f, ripple_db);
}

// 便捷包装 — 高通
static inline void iir_hpf_design(IirBiquad *me, IirApprox approx, float fs,
                                  float fc, float ripple_db) {
  iir_filter_design(me, IIR_FILTER_HPF, approx, fs, fc, 0.0f, ripple_db);
}

// 便捷包装 — 切比雪夫专用 (LPF/HPF; 其余 kind 退化为 Q 谐振器)
static inline void iir_cheby_design(IirBiquad *me, IirFilterKind kind,
                                    float fs, float fc, float ripple_db) {
  iir_filter_design(me, kind, IIR_APPROX_CHEBYSHEV, fs, fc, 0.0f, ripple_db);
}

// 便捷包装 — 带通 (恒 Q 谐振器, 中心增益 0dB)
static inline void iir_bpf_design(IirBiquad *me, float fs, float fc, float bw) {
  iir_filter_design(me, IIR_FILTER_BPF, IIR_APPROX_BUTTERWORTH, fs, fc, bw, 0.0f);
}

// 便捷包装 — 带阻 (恒 Q 谐振器)
static inline void iir_bsf_design(IirBiquad *me, float fs, float fc, float bw) {
  iir_filter_design(me, IIR_FILTER_BSF, IIR_APPROX_BUTTERWORTH, fs, fc, bw, 0.0f);
}

// 便捷包装 — 陷波 (窄带带阻, 高 Q)
static inline void iir_notch_design(IirBiquad *me, float fs, float fc, float bw) {
  iir_filter_design(me, IIR_FILTER_NOTCH, IIR_APPROX_BUTTERWORTH, fs, fc, bw, 0.0f);
}

// biquad 单步运行 — Direct Form 2 (与 iir_df22_run 同结构)
static inline float iir_biquad_run(IirBiquad *me, float in) {
  float v = in - me->a1 * me->x1 - me->a2 * me->x2;
  float out = me->b0 * v + me->b1 * me->x1 + me->b2 * me->x2;
  me->x2 = me->x1;
  me->x1 = v;
  return out;
}

// 重置状态 (不改变系数)
static inline void iir_biquad_reset(IirBiquad *me) {
  me->x1 = 0.0f;
  me->x2 = 0.0f;
}

/* ======================== 一阶 IIR (fs 制) — 低通/高通 ======================== */

// 与既有 dt 制 LowPassFilter 等价 (fs = 1/dt), 区别: 直接用采样率初始化, 无需每次传 dt。
// 双线性变换 (Ω' = tan(π·fc/fs)):
//   低通: b0 = b1 = Ω'/(1+Ω'),  a1 = (Ω'-1)/(1+Ω')
//   高通: b0 = 1/(1+Ω'), b1 = -b0,  a1 = (Ω'-1)/(1+Ω')

// 初始化一阶低通 — fc: -3dB 截止频率
static inline void iir_first_lpf_init(IirFirst *me, float fs, float fc) {
  me->x1 = 0.0f;
  if (fs <= 0.0f || fc <= 0.0f || fc >= 0.5f * fs) {
    me->b0 = 1.0f;  me->b1 = 0.0f;  me->a1 = 0.0f;
    return;
  }
  const float om = tanf(M_PI * fc / fs);
  me->b0 = om / (1.0f + om);
  me->b1 = me->b0;
  me->a1 = (om - 1.0f) / (1.0f + om);
}

// 初始化一阶高通 — fc: -3dB 截止频率
static inline void iir_first_hpf_init(IirFirst *me, float fs, float fc) {
  me->x1 = 0.0f;
  if (fs <= 0.0f || fc <= 0.0f || fc >= 0.5f * fs) {
    me->b0 = 1.0f;  me->b1 = 0.0f;  me->a1 = 0.0f;
    return;
  }
  const float om = tanf(M_PI * fc / fs);
  me->b0 = 1.0f / (1.0f + om);
  me->b1 = -me->b0;
  me->a1 = (om - 1.0f) / (1.0f + om);
}

// 一阶单步运行 — Direct Form 2: v = in - a1·x1;  out = b0·v + b1·x1
static inline float iir_first_run(IirFirst *me, float in) {
  float v = in - me->a1 * me->x1;
  float out = me->b0 * v + me->b1 * me->x1;
  me->x1 = v;
  return out;
}

/* ======================== 导出到既有执行器 ======================== */

// 拷贝到 comp_iir.h IirDf22 (布局与成员名相同, 可直接喂 iir_df22_run)
static inline void iir_biquad_to_df22(const IirBiquad *me, IirDf22 *df22) {
  df22->b0 = me->b0;  df22->b1 = me->b1;  df22->b2 = me->b2;
  df22->a1 = me->a1;  df22->a2 = me->a2;
  df22->x1 = me->x1;  df22->x2 = me->x2;
}

// 导出到 bsp_dsp.h BspBiquadInst — 系数存 {b0, b1, b2, -a1, -a2}
// (bsp_dsp.h 文档约定, 与 CMSIS-DSP arm_biquad_casd_df1_inst_f32 一致)
static inline void iir_biquad_to_bsp_biquad(const IirBiquad *me,
                                            BspBiquadInst *bsp) {
  bsp->coeffs[0] =  me->b0;
  bsp->coeffs[1] =  me->b1;
  bsp->coeffs[2] =  me->b2;
  bsp->coeffs[3] = -me->a1;
  bsp->coeffs[4] = -me->a2;
  bsp->state[0] = me->x1;
  bsp->state[1] = me->x2;
  bsp->state[2] = 0.0f;
  bsp->state[3] = 0.0f;
  bsp_biquad_init(bsp, bsp->coeffs, bsp->state, 1);
}

#endif  // COMP_FILTER_H
