#ifndef PID_QPR_H
#define PID_QPR_H

// QPR (准比例谐振) 控制器 —— PidBase 的子类
//
// 内模原理: 内嵌阻尼谐振器 ω_c*s/(s² + ω_c*s + ω²),
//   在谐振频率 ω 附近提供高增益频带 (而非无穷大单点),
//   牺牲峰值增益换取带宽 → 对电网频率漂移鲁棒
//
// PR vs QPR 对比:
//   PR:  极点恰好在虚轴上 → ω 处增益 ∞ → 零静差但怕频率抖动
//   QPR: 极点左移 ω_c/2 → 增益有限 (≈Ki) 但有带宽 → 适应 ±0.5Hz 电网波动
//   品质因数 Q = ω/ω_c, 典型取 Q=20~50 → ω_c = ω/Q
//
// 传递函数 (连续域):
//   C(s) = Kp + Σ Ki_n * ω_c * s/(s² + ω_c*s + ω_n²)
//   其中 ω_n = n * 2π * f₀, ω_c 控制带宽
//
// 离散化: Tustin 变换
//   归一化 biquad:
//     den = K² + ω_c*K + ω²     (K = 2/dt)
//     b0 = Ki * ω_c * K / den
//     b1 = 0
//     b2 = -b0
//     a1 = 2*(ω² - K²) / den
//     a2 = (K² - ω_c*K + ω²) / den
//   PR 就是 QPR 在 ω_c→0 时的极限 (此时 a2→1)

#include "comp_pid.h"
#include <stdint.h>

// ======== 谐波配置 ========
typedef struct {
  uint8_t order;    // 谐波次数: 1=基波, 3=3次, 5=5次...
  float   ki;       // 谐振增益 (QPR 峰值增益 ≈ Ki)
} QPRHarmonic;

// ======== 配置结构体 ========
#define QPR_MAX_HARMONICS 5

typedef struct {
  float        kp;                      // 比例系数 (全频段)
  float        f0;                      // 基波频率 (Hz), 如 50.0
  float        bandwidth;               // 带宽 ω_c (rad/s), 典型 = 2π*(2~10)Hz
                                        // 或者从品质因数反推: ω_c = 2π*f₀/Q, Q∈[20,50]
  uint8_t      num_harmonics;           // 实际谐波个数 (≤ QPR_MAX_HARMONICS)
  QPRHarmonic  harmonics[QPR_MAX_HARMONICS]; // 各谐波配置
  float        deadzone;                // 死区 (0=不启用)
} QPRConfig;

// ======== 单个谐振器状态 (biquad DF1) ========
typedef struct {
  float b0, b1, b2;   // 分子系数
  float a1, a2;       // 分母系数
  float x1, x2;       // 输入历史: x[n-1], x[n-2]
  float y1, y2;       // 输出历史: y[n-1], y[n-2]
  float ki;           // 保存 ki 以便运行时调参
  float w;             // 保存 ω 以便重算系数
} QPRResonator;

// ======== 子类结构体 ========
typedef struct {
  PidBase     base;                       // 基类 (必须第一个)
  QPRConfig   cfg;                        // 配置 (可热替换)
  float       wc;                         // 缓存: 当前带宽 ω_c (rad/s)
  QPRResonator res[QPR_MAX_HARMONICS];    // 各谐波谐振器状态
} PidQPR;

// ======== 构造 ========

// 初始化 QPR: 预计算所有 biquad 系数 + 绑定 ops
void pid_qpr_init(PidQPR *me, float dt, float out_min, float out_max,
                  const QPRConfig *cfg);

// ======== 运行时调参 ========

// 整体替换配置
void pid_qpr_update_config(PidQPR *me, const QPRConfig *cfg);

// 快捷调参
void pid_qpr_set_kp(PidQPR *me, float kp);
void pid_qpr_set_ki(PidQPR *me, uint8_t harmonic_order, float ki);
void pid_qpr_set_bandwidth(PidQPR *me, float bandwidth_rad_s);

// 运行时调整基波频率 (如电网从 50Hz 飘到 49.8Hz)
// 重算所有谐振器的中心频率 ω_n = n * 2π * f0_new
void pid_qpr_retune_f0(PidQPR *me, float f0_new);

#endif
