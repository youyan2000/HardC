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

#endif  // COMP_IIR_H
