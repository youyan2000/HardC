// IIR 数字补偿器库 — DF22 (二阶) / DF23 (三阶) / 2P2Z (极零点)
//
// 来源: TI controlSUITE DCL (DF22, DF23) + Solar (CNTL_2P2Z_F)
// 翻译为 C-OOP 纯C float 版本
//
// 三种实现适用于不同场景:
//   DF22  — 二阶 Direct Form 2, 最少状态存储, 适合 M4F/M7 的 CMSIS-DSP biquad 加速
//   DF23  — 三阶 Direct Form 2, 更高阶补偿 (如 LCL 滤波器谐振抑制)
//   2P2Z  — Direct Form 1 极零点形式, 两级饱和 (软限制 → 硬限制), 控制环经典
//
// 调用方式 (ISR 中每控制周期调用):
//   iir_df22_run(&filt, error);    // error = Ref - Fdbk, 返回补偿输出
//   iir_2p2z_run(&filt, error);    // 同上, 带两级饱和

#ifndef COMP_IIR_H
#define COMP_IIR_H

#include <stdint.h>

// ======================= DF22 — Direct Form 2, 二阶 IIR =======================

// H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
// 注: 分母 a0 = 1 已归一化, a1/a2 符号与 DCL 一致 (直接加)
typedef struct {
  float b0;       // 分子系数 B0 (当前输入)
  float b1;       // 分子系数 B1 (输入延迟 1)
  float b2;       // 分子系数 B2 (输入延迟 2)
  float a1;       // 分母系数 A1 (输出延迟 1)
  float a2;       // 分母系数 A2 (输出延迟 2)
  float x1;       // 内部状态延迟 1
  float x2;       // 内部状态延迟 2
} IirDf22;

// DF22 默认初始化 (全通, 零状态)
#define IIR_DF22_DEFAULTS { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }

// DF22 初始化 — 设置系数, 清零状态
static inline void iir_df22_init(IirDf22 *me, float b0, float b1, float b2,
                                 float a1, float a2) {
  me->b0 = b0;
  me->b1 = b1;
  me->b2 = b2;
  me->a1 = a1;
  me->a2 = a2;
  me->x1 = 0.0f;
  me->x2 = 0.0f;
}

// DF22 单步运行 — Direct Form 2 结构
//   in: 当前输入 (通常为误差信号)
//   返回: 滤波器输出
// 内部状态更新:
//   v = in - a1*x1 - a2*x2
//   out = b0*v + b1*x1 + b2*x2
//   历史: x2 = x1, x1 = v
static inline float iir_df22_run(IirDf22 *me, float in) {
  float v = in - me->a1 * me->x1 - me->a2 * me->x2;
  float out = me->b0 * v + me->b1 * me->x1 + me->b2 * me->x2;
  me->x2 = me->x1;
  me->x1 = v;
  return out;
}

// 重置 DF22 状态 (不改变系数)
static inline void iir_df22_reset(IirDf22 *me) {
  me->x1 = 0.0f;
  me->x2 = 0.0f;
}

// ======================= DF23 — Direct Form 2, 三阶 IIR =======================

// H(z) = (b0 + b1*z^-1 + b2*z^-2 + b3*z^-3) / (1 + a1*z^-1 + a2*z^-2 + a3*z^-3)
typedef struct {
  float b0;       // 分子 B0
  float b1;       // 分子 B1
  float b2;       // 分子 B2
  float b3;       // 分子 B3
  float a1;       // 分母 A1
  float a2;       // 分母 A2
  float a3;       // 分母 A3
  float x1;       // 内部状态延迟 1
  float x2;       // 内部状态延迟 2
  float x3;       // 内部状态延迟 3
} IirDf23;

#define IIR_DF23_DEFAULTS { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }

static inline void iir_df23_init(IirDf23 *me, float b0, float b1, float b2, float b3,
                                 float a1, float a2, float a3) {
  me->b0 = b0; me->b1 = b1; me->b2 = b2; me->b3 = b3;
  me->a1 = a1; me->a2 = a2; me->a3 = a3;
  me->x1 = 0.0f; me->x2 = 0.0f; me->x3 = 0.0f;
}

static inline float iir_df23_run(IirDf23 *me, float in) {
  float v = in - me->a1 * me->x1 - me->a2 * me->x2 - me->a3 * me->x3;
  float out = me->b0 * v + me->b1 * me->x1 + me->b2 * me->x2 + me->b3 * me->x3;
  me->x3 = me->x2;
  me->x2 = me->x1;
  me->x1 = v;
  return out;
}

static inline void iir_df23_reset(IirDf23 *me) {
  me->x1 = 0.0f; me->x2 = 0.0f; me->x3 = 0.0f;
}

// ======================= 2P2Z — Direct Form 1, 两级饱和 =======================

// H(z) = (B0 + B1*z^-1 + B2*z^-2) / (1 - A1*z^-1 - A2*z^-2)
// 注: 系数符号与 DF22 不同 (分母 A1/A2 做减法, 与 Solar CNTL_2P2Z_F 一致)
//
// 两级饱和策略:
//   软限制 (i_min): 在内部历史更新前钳位 (≥ -0.9), 防止深度下冲破坏内部状态
//   硬限制 (min):   最终输出钳位, 应用的真实限幅

typedef struct {
  // 系数
  float b0;         // 分子 B0
  float b1;         // 分子 B1
  float b2;         // 分子 B2
  float a1;         // 分母 A1 (做减法: +A1*out1 + A2*out2)
  float a2;         // 分母 A2

  // 状态
  float out1;       // 输出 u(k-1)
  float out2;       // 输出 u(k-2)
  float err1;       // 误差 e(k-1)
  float err2;       // 误差 e(k-2)

  // 限制
  float max;        // 上限
  float i_min;      // 中间软限制下限 (≥ -0.9, 保护内部状态)
  float min;        // 最终硬限制下限
} Iir2p2z;

#define IIR_2P2Z_DEFAULTS { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, -0.9f, -1.0f }

// 2P2Z 初始化
static inline void iir_2p2z_init(Iir2p2z *me, float b0, float b1, float b2,
                                 float a1, float a2, float max, float min) {
  me->b0 = b0; me->b1 = b1; me->b2 = b2;
  me->a1 = a1; me->a2 = a2;
  me->out1 = 0.0f; me->out2 = 0.0f;
  me->err1 = 0.0f; me->err2 = 0.0f;
  me->max = max;
  me->i_min = (min > -0.9f) ? min : -0.9f;  // 软限制不小于 -0.9
  me->min = min;
}

// 2P2Z 单步运行 — 带两级饱和
//   error: 当前误差 (Ref - Fdbk)
//   返回: 饱和后的补偿输出
static inline float iir_2p2z_run(Iir2p2z *me, float error) {
  // Direct Form 1 差分方程
  float out = me->b0 * error
            + me->b1 * me->err1
            + me->b2 * me->err2
            + me->a1 * me->out1
            + me->a2 * me->out2;

  // 误差历史更新
  me->err2 = me->err1;
  me->err1 = error;

  // 上限钳位
  if (out > me->max) { out = me->max; }

  // 中间软限制 (保护内部状态, 避免深度下冲导致的非线性恢复)
  if (out < me->i_min) { out = me->i_min; }

  // 输出历史更新 (用软限制后的值, 保持状态可控)
  me->out2 = me->out1;
  me->out1 = out;

  // 最终硬限制
  if (out < me->min) { out = me->min; }

  return out;
}

// 重置 2P2Z 状态
static inline void iir_2p2z_reset(Iir2p2z *me) {
  me->out1 = 0.0f; me->out2 = 0.0f;
  me->err1 = 0.0f; me->err2 = 0.0f;
}

// ======== Q15 定点 IIR (v1.2 扩展 — FixedPointLib) ========

// 来源: TI controlSUITE FixedPointLib/v1_20/iir.h
//
// Direct Form 1, Q15 定点实现
// 适用于无 FPU 平台或 Q15 优先的高速场景
// 系数缩放: 1.0 = 0x7FFF (32767)
// 差分方程:
//   y[n] = (b0*x[n] + b1*x[n-1] + ... + bN*x[n-N]
//           - a1*y[n-1] - a2*y[n-2] - ... - aN*y[n-N]) >> shift
// 乘法累加后右移归一化, 输出饱和到 [-32768, 32767]
//
// 调用方式 (ISR 中每控制周期调用):
//   Iir16Cfg cfg = { .b_coeffs = { ... }, .a_coeffs = { ... }, .order = 2, .shift = 15 };
//   Iir16State st;
//   iir16_init(&st);
//   int16_t y = iir16_run(&st, &cfg, x);

#define IIR16_MAX_ORDER  8   // Q15 最大阶数 (delay line 大小)

// Q15 滤波器配置 (可共享, 只读)
typedef struct {
  int16_t b_coeffs[IIR16_MAX_ORDER + 1];  // 分子系数 Q15 (b0..bN), N ≤ IIR16_MAX_ORDER
  int16_t a_coeffs[IIR16_MAX_ORDER + 1];  // 分母系数 Q15 (a1..aN), a[0]=1.0 已归一化, 占位不用
  uint8_t order;                           // 滤波器阶数 N (1-8)
  uint8_t shift;                           // 后缩放右移位数 (0-15)
} Iir16Cfg;

// Q15 滤波器运行状态 (每实例独立)
typedef struct {
  int16_t x_hist[IIR16_MAX_ORDER + 1];  // 输入延迟线: x_hist[0]=x[n], x_hist[i]=x[n-i]
  int16_t y_hist[IIR16_MAX_ORDER + 1];  // 输出历史: y_hist[0]=y[n-1], y_hist[1]=y[n-2], ...
} Iir16State;

// Q15 默认配置 (全通, 零状态)
#define IIR16_CFG_DEFAULTS { {32767,0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0,0}, 0, 0 }

// 初始化 Q15 延迟线 — 全部清零
static inline void iir16_init(Iir16State *me) {
  for (int i = 0; i <= IIR16_MAX_ORDER; i++) {
    me->x_hist[i] = 0;
    me->y_hist[i] = 0;
  }
}

// Q15 IIR 单步运行 — Direct Form 1 + 输出饱和
//   cfg:   滤波器配置 (系数 + 阶数 + 移位数)
//   input: 当前输入 Q15
//   返回:  滤波输出 Q15, 饱和在 [-32768, 32767]
//   int64 累加器保证 17 项乘积累加不溢出 (阶数 ≤8 时最坏情况)
static inline int16_t iir16_run(Iir16State *me, const Iir16Cfg *cfg, int16_t input) {
  // 移位输入历史: x_hist[i] ← x_hist[i-1], 新值进 x_hist[0]
  for (int i = cfg->order; i > 0; i--) {
    me->x_hist[i] = me->x_hist[i - 1];
  }
  me->x_hist[0] = input;

  // Direct Form 1 累加: y = SUM(b_i * x[n-i]) - SUM(a_j * y[n-j])
  // 注: a[0]=1 已归一化, y_hist[j-1] 对 j=1 取 y[n-1], j=2 取 y[n-2], ...
  int64_t acc = 0;
  for (int i = 0; i <= cfg->order; i++) {
    acc += (int64_t)cfg->b_coeffs[i] * (int64_t)me->x_hist[i];
  }
  for (int j = 1; j <= cfg->order; j++) {
    acc -= (int64_t)cfg->a_coeffs[j] * (int64_t)me->y_hist[j - 1];
  }

  // 后缩放: 右移归一化回 Q15
  int64_t y = acc >> cfg->shift;

  // 饱和到 int16_t 范围
  if (y > 32767)  { y = 32767; }
  if (y < -32768) { y = -32768; }

  // 移位输出历史: y_hist[i] ← y_hist[i-1], 新值进 y_hist[0]
  for (int i = cfg->order; i > 0; i--) {
    me->y_hist[i] = me->y_hist[i - 1];
  }
  me->y_hist[0] = (int16_t)y;

  return (int16_t)y;
}

// ======== Q31 定点 IIR (v1.2 扩展 — FixedPointLib) ========

// 来源: TI controlSUITE FixedPointLib/v1_20/iir.h
//
// Direct Form 1, Q31 定点实现 (高精度变体)
// 系数缩放: 1.0 = 0x7FFFFFFF (2147483647)
// 乘法累加后右移归一化, 输出饱和到 [-2^31, 2^31-1]

#define IIR32_MAX_ORDER  8   // Q31 最大阶数

// Q31 滤波器配置 (可共享, 只读)
typedef struct {
  int32_t b_coeffs[IIR32_MAX_ORDER + 1];  // 分子系数 Q31
  int32_t a_coeffs[IIR32_MAX_ORDER + 1];  // 分母系数 Q31 (a[0]=1.0 已归一化, 占位不用)
  uint8_t order;                           // 滤波器阶数 N (1-8)
  uint8_t shift;                           // 后缩放右移位数 (0-31)
} Iir32Cfg;

// Q31 滤波器运行状态 (每实例独立)
typedef struct {
  int32_t x_hist[IIR32_MAX_ORDER + 1];  // 输入延迟线
  int32_t y_hist[IIR32_MAX_ORDER + 1];  // 输出历史
} Iir32State;

#define IIR32_CFG_DEFAULTS { {2147483647,0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0,0}, 0, 0 }

// 初始化 Q31 延迟线 — 全部清零
static inline void iir32_init(Iir32State *me) {
  for (int i = 0; i <= IIR32_MAX_ORDER; i++) {
    me->x_hist[i] = 0;
    me->y_hist[i] = 0;
  }
}

// Q31 IIR 单步运行 — Direct Form 1 + 输出饱和
//   cfg:   滤波器配置
//   input: 当前输入 Q31
//   返回:  滤波输出 Q31, 饱和在 [-2^31, 2^31-1]
static inline int32_t iir32_run(Iir32State *me, const Iir32Cfg *cfg, int32_t input) {
  // 移位输入历史
  for (int i = cfg->order; i > 0; i--) {
    me->x_hist[i] = me->x_hist[i - 1];
  }
  me->x_hist[0] = input;

  // Direct Form 1 累加 (int64 防止溢出; Q31*Q31 → Q62, 17 项安全)
  int64_t acc = 0;
  for (int i = 0; i <= cfg->order; i++) {
    acc += (int64_t)cfg->b_coeffs[i] * (int64_t)me->x_hist[i];
  }
  for (int j = 1; j <= cfg->order; j++) {
    acc -= (int64_t)cfg->a_coeffs[j] * (int64_t)me->y_hist[j - 1];
  }

  // 后缩放: 右移归一化回 Q31
  int64_t y = acc >> cfg->shift;

  // 饱和到 int32_t 范围
  if (y > 2147483647)  { y = 2147483647; }
  if (y < -2147483648LL) { y = -2147483648LL; }

  // 移位输出历史
  for (int i = cfg->order; i > 0; i--) {
    me->y_hist[i] = me->y_hist[i - 1];
  }
  me->y_hist[0] = (int32_t)y;

  return (int32_t)y;
}

#endif  // COMP_IIR_H
