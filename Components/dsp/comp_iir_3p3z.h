// 数字滤波器库扩展 — 3P3Z 控制器 (三极点三零点)
//
// 来源: TI controlSUITE digital_power CNTL_3P3Z_F (f28x7x_v1.0/C_macros)
// 翻译为 HardC 纯C float static inline 版本
//
// 差分方程 (Direct Form 1):
//   Out[k] = A1*Out[k-1] + A2*Out[k-2] + A3*Out[k-3]
//          + B0*Err[k] + B1*Err[k-1] + B2*Err[k-2] + B3*Err[k-3]
//
// 三级饱和: OutPresat → clamp(max) → clamp(i_min) → clamp(min)
//   中间软限制 i_min 防止内部状态过深导致非线性恢复慢

#ifndef COMP_IIR_3P3Z_H
#define COMP_IIR_3P3Z_H

// ======================= Iir3p3z (三极点三零点, Direct Form 1) =======================

// 系数结构体 (编译期常量, 可放 Flash)
typedef struct {
  float b3;               // 分子 B3 (Z^-3)
  float b2;               // 分子 B2 (Z^-2)
  float b1;               // 分子 B1 (Z^-1)
  float b0;               // 分子 B0 (Z^0)
  float a3;               // 分母 A3 (Z^-3)
  float a2;               // 分母 A2 (Z^-2)
  float a1;               // 分母 A1 (Z^-1)
  float max;              // 上限 (硬钳位)
  float i_min;            // 中间软限制 (保护内部状态)
  float min;              // 下限 (硬钳位)
} Iir3p3zCoeff;

// 变量结构体 (运行时状态, 放 RAM)
typedef struct {
  float ref;              // 输入: 设定值
  float fdbk;             // 输入: 反馈值
  float err;              // 内部: 当前误差
  float err1;             // 内部: 误差延迟 Z^-1
  float err2;             // 内部: 误差延迟 Z^-2
  float err3;             // 内部: 误差延迟 Z^-3
  float out;              // 输出: 饱和后输出
  float out1;             // 内部: 输出延迟 Z^-1
  float out2;             // 内部: 输出延迟 Z^-2
  float out3;             // 内部: 输出延迟 Z^-3
  float out_presat;       // 内部: 预饱和值
} Iir3p3zVars;

#define IIR_3P3Z_COEFF_DEFAULTS { 0, 0, 0, 1, 0, 0, 0, 1.0f, -0.9f, 0 }
#define IIR_3P3Z_VARS_DEFAULTS   { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }

// 初始化变量
static inline void iir_3p3z_vars_init(Iir3p3zVars *me) {
  me->ref = 0.0f; me->fdbk = 0.0f;
  me->err = 0.0f; me->err1 = 0.0f; me->err2 = 0.0f; me->err3 = 0.0f;
  me->out = 0.0f; me->out1 = 0.0f; me->out2 = 0.0f; me->out3 = 0.0f;
  me->out_presat = 0.0f;
}

// 3P3Z 单步运行 — 三级饱和 (输出硬限 → 中间软限 → 下硬限)
static inline float iir_3p3z_run(Iir3p3zVars *me, const Iir3p3zCoeff *coeff,
                                  float ref, float fdbk) {
  me->ref = ref;
  me->fdbk = fdbk;

  // 误差计算
  me->err = ref - fdbk;

  // Direct Form 1 差分方程
  me->out_presat = coeff->a1 * me->out1
                 + coeff->a2 * me->out2
                 + coeff->a3 * me->out3
                 + coeff->b0 * me->err
                 + coeff->b1 * me->err1
                 + coeff->b2 * me->err2
                 + coeff->b3 * me->err3;

  // 误差历史移位
  me->err3 = me->err2;
  me->err2 = me->err1;
  me->err1 = me->err;

  // 上限钳位
  if (me->out_presat > coeff->max) {
    me->out_presat = coeff->max;
  }

  // 中间软限制 (保护内部状态, 避免深度下冲导致的非线性恢复)
  if (me->out_presat < coeff->i_min) {
    me->out_presat = coeff->i_min;
  }

  // 输出历史移位
  me->out3 = me->out2;
  me->out2 = me->out1;
  me->out1 = me->out_presat;

  // 最终下硬限
  me->out = (me->out_presat >= coeff->min) ? me->out_presat : coeff->min;

  return me->out;
}

// 重置状态
static inline void iir_3p3z_reset(Iir3p3zVars *me) {
  me->err = 0.0f; me->err1 = 0.0f; me->err2 = 0.0f; me->err3 = 0.0f;
  me->out = 0.0f; me->out1 = 0.0f; me->out2 = 0.0f; me->out3 = 0.0f;
  me->out_presat = 0.0f;
}

#endif  // COMP_IIR_3P3Z_H
