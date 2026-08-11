// BSP 数字信号处理硬件加速抽象层 (Digital Signal Processing)
//
// 本层封装不同平台的硬件加速 API, 向上提供统一接口:
//   sqrt     — 硬件平方根 (FPU VSQRT / TMU __sqrt / 纯C牛顿迭代)
//   biquad   — 二阶 IIR 滤波器 (CMSIS-DSP SIMD / C2000 CLA / 纯C DFI)
//
// 使用方式:
//   #include "BSP/bsp_dsp.h"
//   float result = bsp_sqrt_f32(2.0f);               // 硬件加速 sqrt
//   bsp_biquad_apply(&inst, &src, &dst, block_size); // 硬件加速 IIR
//
// ======== 平台自动检测 ========
//
// BSP_DSP_ARCH 语义:
//   2 = CMSIS-DSP 硬件加速 (M4F/M7 — FPU + SIMD)
//   1 = CMSIS-DSP 软件实现 (M0/M0+/M3 — 有 arm_math.h 但无 DSP 指令)
//   3 = C2000Ware  (TMS320F28xxx — TMU + CLA)
//   0 = 纯C 回退  (未知平台 / 无 DSP 库)
//
// CMSIS-DSP 硬件 (Cortex-M4F/M7, FPU + DSP 扩展):
//   sqrt:    arm_sqrt_f32 → FPU VSQRT 单周期指令
//   biquad:  arm_biquad_cascade_df1_f32 → M4F/M7 SIMD 双 float 乘加
//
// CMSIS-DSP 软件 (Cortex-M0/M0+/M3, 有 arm_math.h 但无硬件加速):
//   sqrt:    arm_sqrt_f32 → CMSIS-DSP 软件实现
//   biquad:  arm_biquad_cascade_df1_f32 → CMSIS-DSP 软件 DFI
//
// C2000Ware 模式 (TMS320F28xxx):
//   sqrt:    __sqrt → TMU 硬件 sqrt 指令
//   biquad:  DSP_biquad_df1 → CLA 协处理器 (需用户工程提供 DSP.h)
//
// 纯C 回退模式 (默认):
//   sqrt:    牛顿迭代法 (6 次迭代, ISR 安全)
//   biquad:  手动 Direct Form I 展开, 无外部依赖

#ifndef BSP_DSP_H
#define BSP_DSP_H

#include <stdint.h>
#include <math.h>
#include <stdbool.h>

// ======== 平台检测 ========

// Tier 2: CMSIS-DSP 硬件加速 (M4F/M7 — FPU + SIMD)
#if defined(__ARM_FEATURE_DSP) && defined(__FPU_PRESENT) && (__FPU_PRESENT == 1)
  #define BSP_DSP_ARCH  2
  #include "arm_math.h"

// Tier 1: CMSIS-DSP 软件 (M0/M0+/M3 或 禁用了FPU的M4 — 有库但无硬件加速)
#elif (defined(__ARM_ARCH_6M__) || defined(__ARM_ARCH_7M__) || \
       (defined(__ARM_ARCH_7EM__) && (!defined(__FPU_PRESENT) || (__FPU_PRESENT == 0))))
  #if defined(__has_include) && __has_include("arm_math.h")
    #define BSP_DSP_ARCH  1   // CMSIS-DSP 软件实现
    #include "arm_math.h"
  #else
    #define BSP_DSP_ARCH  0   // 纯C 回退 (无 arm_math.h)
  #endif

// Tier 3: C2000Ware (TMS320F28xxx — TMU + CLA)
#elif defined(__TMS320C2000__)
  #define BSP_DSP_ARCH  3
  // C2000Ware DSP/CLA 头文件由用户工程 include path 提供
  #if defined(__has_include) && __has_include("DSP.h")
    #include "DSP.h"
  #endif

// Tier 0: 纯C 回退 (未知平台 / 无 DSP 库)
#else
  #define BSP_DSP_ARCH  0
#endif

// ======== 硬件 sqrt ========

// 平方根 — 硬件加速 / 纯C 回退
//
// STM32 M4F/M7 (CMSIS-DSP HW): arm_sqrt_f32 → FPU VSQRT 单周期
// STM32 M0+/M3 (CMSIS-DSP SW):  arm_sqrt_f32 → CMSIS-DSP 软件实现
// C2000 (TMU):                  __sqrt       → TMU 硬件指令
// 纯C 回退:                     牛顿迭代法
static inline float bsp_sqrt_f32(float x) {
#if BSP_DSP_ARCH == 2 || BSP_DSP_ARCH == 1
  float result;
  arm_sqrt_f32(x, &result);
  return result;
#elif BSP_DSP_ARCH == 3
  return __sqrt(x);  // C2000 TMU 硬件 sqrt
#else
  // 纯C 牛顿迭代法 (无 libm 依赖, ISR 安全)
  if (x <= 0.0f) return 0.0f;
  float val = x;
  float last;
  int i;
  for (i = 0; i < 6; i++) {  // 6 次迭代 ≈ FP32 精度
    last = val;
    val = (val + x / val) * 0.5f;
    if (val == last) break;
  }
  return val;
#endif
}

// ======== Biquad 二阶 IIR 滤波器 (Direct Form I) ========

// 系数存储格式: {b0, b1, b2, -a1, -a2}
// 状态存储格式: {x[n-1], x[n-2], y[n-1], y[n-2]}
#define BSP_BIQUAD_COEFF_COUNT 5
#define BSP_BIQUAD_STATE_COUNT 4

// Biquad 实例 — 统一结构体布局, 所有平台下 coeffs/state 始终存在
// CMSIS-DSP 模式下额外嵌入 cmsis_inst (含系数/状态指针)
typedef struct {
  float coeffs[BSP_BIQUAD_COEFF_COUNT];  // {b0, b1, b2, -a1, -a2}
  float state[BSP_BIQUAD_STATE_COUNT];   // {x1, x2, y1, y2}
#if BSP_DSP_ARCH == 2 || BSP_DSP_ARCH == 1
  arm_biquad_casd_df1_inst_f32 cmsis_inst;  // CMSIS-DSP 实例
#endif
} BspBiquadInst;

// 初始化 Biquad 实例 — 绑定系数和状态
//
// coeffs: 5 元素数组 {b0, b1, b2, -a1, -a2}
// state:  4 元素数组 {x1, x2, y1, y2}, 传 NULL 则清零
// num_stages: 级联级数 (通常为 1), 纯C 模式忽略 (级联由上层管理)
static inline void bsp_biquad_init(BspBiquadInst *me,
                                    float *coeffs, float *state,
                                    int num_stages) {
#if BSP_DSP_ARCH == 2 || BSP_DSP_ARCH == 1
  arm_biquad_cascade_df1_init_f32(&me->cmsis_inst, (uint32_t)num_stages,
                                   coeffs, state);
#else
  // 纯C: 复制系数到结构体, 清零(或复制)状态
  (void)num_stages;
  for (int i = 0; i < BSP_BIQUAD_COEFF_COUNT; i++) {
    me->coeffs[i] = coeffs[i];
  }
  for (int i = 0; i < BSP_BIQUAD_STATE_COUNT; i++) {
    me->state[i] = (state ? state[i] : 0.0f);
  }
#endif
}

// 执行 Biquad 滤波 (单样本或 block)
//
// src:  输入样本数组
// dst:  输出样本数组 (可与 src 相同)
// block_size: 处理样本数 (通常为 1)
//
// 所有平台均执行真实的 Direct Form I 滤波, 不再有静默直通行为.
static inline void bsp_biquad_apply(BspBiquadInst *me,
                                     const float *src, float *dst,
                                     int block_size) {
#if BSP_DSP_ARCH == 2 || BSP_DSP_ARCH == 1
  arm_biquad_cascade_df1_f32(&me->cmsis_inst, (float *)src, dst,
                              (uint32_t)block_size);
#else
  // 纯C Direct Form I — 真实 IIR 滤波
  // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
  for (int i = 0; i < block_size; i++) {
    float x0 = src[i];
    float y0 = me->coeffs[0] * x0
             + me->coeffs[1] * me->state[0]    // x[n-1]
             + me->coeffs[2] * me->state[1]    // x[n-2]
             - me->coeffs[3] * me->state[2]    // y[n-1] (-a1)
             - me->coeffs[4] * me->state[3];   // y[n-2] (-a2)
    // 推进状态: x[n-2]←x[n-1]←x[n], y[n-2]←y[n-1]←y[n]
    me->state[1] = me->state[0];  me->state[0] = x0;
    me->state[3] = me->state[2];  me->state[2] = y0;
    dst[i] = y0;
  }
#endif
}

// ======== CMSIS-DSP 直接兼容层 ========
// 仅在 CMSIS-DSP 可用时暴露 arm_math 类型, 允许现有代码平滑迁移

#if BSP_DSP_ARCH == 2 || BSP_DSP_ARCH == 1
  // arm_math.h 已包含, arm_biquad_casd_df1_inst_f32 等类型可直接使用
  #define BSP_DSP_CMSIS_AVAILABLE 1
#else
  #define BSP_DSP_CMSIS_AVAILABLE 0
#endif

// ======== CLA 数学加速调度 (C2000 CLA 协处理器) ========
//
// CLAmath 是 TI C2000 的 CLA (Control Law Accelerator) 协处理器数学库,
// 提供硬件加速的 sin/cos/sqrt/atan/exp/ln 等超越函数.
//
// 用法: Components 层调用 bsp_sin_f32(x) / bsp_cos_f32(x) 等,
//       BSP 层根据 BSP_DSP_ARCH 选择:
//         ARCH=3 + CLAmath:  CLA 硬件加速 (CLAsin/CLAcos/...)
//         ARCH=3 - CLAmath:  math.h sinf/cosf (FPU)
//         ARCH=1/2:          math.h sinf/cosf (FPU)
//         ARCH=0:            math.h sinf/cosf (纯C)
//
// 注: CLA 函数依赖 CLAmath.lib 链接, 且需要 CLA 初始化.
//     默认使用标准 math.h; 用户工程若含 CLAmath.h 则自动切换.

#if BSP_DSP_ARCH == 3 && defined(__has_include) && __has_include("CLAmath.h")
  #define BSP_DSP_CLA_AVAILABLE 1
  #include "CLAmath.h"
#else
  #define BSP_DSP_CLA_AVAILABLE 0
#endif

// sin(x) — x: rad
static inline float bsp_sin_f32(float x) {
#if BSP_DSP_CLA_AVAILABLE
  return CLAsin(x);
#else
  return sinf(x);
#endif
}

// cos(x) — x: rad
static inline float bsp_cos_f32(float x) {
#if BSP_DSP_CLA_AVAILABLE
  return CLAcos(x);
#else
  return cosf(x);
#endif
}

// sin(x) + cos(x) 同时计算 (CLA 硬件可以一次算两个, 节省周期)
static inline void bsp_sincos_f32(float x, float *sin_out, float *cos_out) {
#if BSP_DSP_CLA_AVAILABLE
  CLAsincos(x, sin_out, cos_out);
#else
  *sin_out = sinf(x);
  *cos_out = cosf(x);
#endif
}

// atan(x)
static inline float bsp_atan_f32(float x) {
#if BSP_DSP_CLA_AVAILABLE
  return CLAatan(x);
#else
  return atanf(x);
#endif
}

// atan2(y, x) — 全象限反正切
static inline float bsp_atan2_f32(float y, float x) {
#if BSP_DSP_CLA_AVAILABLE
  return CLAatan2(y, x);
#else
  return atan2f(y, x);
#endif
}

// asin(x)
static inline float bsp_asin_f32(float x) {
#if BSP_DSP_CLA_AVAILABLE
  return CLAasin(x);
#else
  return asinf(x);
#endif
}

// acos(x)
static inline float bsp_acos_f32(float x) {
#if BSP_DSP_CLA_AVAILABLE
  return CLAacos(x);
#else
  return acosf(x);
#endif
}

// e^x
static inline float bsp_exp_f32(float x) {
#if BSP_DSP_CLA_AVAILABLE
  return CLAexp(x);
#else
  return expf(x);
#endif
}

// 10^x
static inline float bsp_exp10_f32(float x) {
#if BSP_DSP_CLA_AVAILABLE
  return CLAexp10(x);
#else
  return powf(10.0f, x);
#endif
}

// ln(x) — 自然对数
static inline float bsp_ln_f32(float x) {
#if BSP_DSP_CLA_AVAILABLE
  return CLAln(x);
#else
  return logf(x);
#endif
}

// log10(x)
static inline float bsp_log10_f32(float x) {
#if BSP_DSP_CLA_AVAILABLE
  return CLAlog10(x);
#else
  return log10f(x);
#endif
}

// 1/sqrt(x) — 快速倒数平方根 (Newton-Raphson)
static inline float bsp_isqrt_f32(float x) {
#if BSP_DSP_CLA_AVAILABLE
  return CLAisqrt(x);
#else
  if (x <= 0.0f) return 0.0f;
  return 1.0f / sqrtf(x);
#endif
}

// 2^(num/den) — 分数指数
static inline float bsp_exp2_f32(float num, float den) {
#if BSP_DSP_CLA_AVAILABLE
  return CLAexp2(num, den);
#else
  return powf(2.0f, num / den);
#endif
}

// N^x — 任意底数指数
static inline float bsp_expn_f32(float n, float x) {
#if BSP_DSP_CLA_AVAILABLE
  return CLAexpN(n, x);
#else
  return powf(n, x);
#endif
}

// log_N(x) — 任意底数对数
static inline float bsp_logn_f32(float x, float n) {
#if BSP_DSP_CLA_AVAILABLE
  return CLAlogN(x, n);
#else
  return logf(x) / logf(n);
#endif
}

// CLAdiv — 快速 Newton-Raphson 除法 (CLA 硬件 12 周期)
static inline float bsp_fast_div_f32(float num, float den) {
#if BSP_DSP_CLA_AVAILABLE
  return CLAdiv(num, den);
#else
  return num / den;
#endif
}

#endif  // BSP_DSP_H
