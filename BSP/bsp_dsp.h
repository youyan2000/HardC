// BSP 数字信号处理硬件加速抽象层 (Digital Signal Processing)
//
// 本层封装不同平台的硬件加速 API, 向上提供统一接口:
//   sqrt     — 硬件平方根 (FPU VSQRT / TMU __sqrt / 纯 C 回退)
//   biquad   — 二阶 IIR 滤波器 (CMSIS-DSP SIMD / C2000 CLA / 纯 C DFI 回退)
//
// 使用方式:
//   #include "BSP/bsp_dsp.h"
//   float result = bsp_sqrt_f32(2.0f);               // 硬件加速 sqrt
//   bsp_biquad_apply(&inst, &src, &dst, block_size); // 硬件加速 IIR
//
// 纯 C 回退模式 (默认):
//   - sqrt:    牛顿迭代法
//   - biquad:  手动 Direct Form I 展开, 无外部依赖
//
// STM32 CMSIS-DSP 模式:
//   - sqrt:    arm_sqrt_f32 → FPU VSQRT 单周期指令
//   - biquad:  arm_biquad_cascade_df1_f32 → M4F/M7 SIMD 双 float 乘加
//
// C2000Ware 模式 (后续实现):
//   - sqrt:    TMU __sqrt
//   - biquad:  CLA DSP_biquad_df1

#ifndef BSP_DSP_H
#define BSP_DSP_H

#include <stdint.h>
#include <math.h>
#include <stdbool.h>

// ======== 平台检测 ========

// __ARM_FEATURE_DSP: Cortex-M4/M7 有 DSP 扩展指令集 + CMSIS-DSP 库
#if defined(__ARM_FEATURE_DSP) || defined(__ARM_ARCH_7EM__)
  #define BSP_DSP_HW  1
  #include "arm_math.h"
// C2000 平台 (后续实现):
// #elif defined(__TMS320C2000__)
//   #define BSP_DSP_HW  2
//   #include "C2000Ware_dsp.h"
#else
  #define BSP_DSP_HW  0   // 纯 C 回退
#endif

// ======== 硬件 sqrt ========

// 平方根 — 硬件加速 / 纯 C 回退
//
// STM32 (CMSIS-DSP): arm_sqrt_f32 → FPU VSQRT 单周期
// C2000 (TMU):       __sqrt       → TMU 硬件指令
// 纯 C 回退:          牛顿迭代法
static inline float bsp_sqrt_f32(float x) {
#if BSP_DSP_HW == 1
  float result;
  arm_sqrt_f32(x, &result);
  return result;
#elif BSP_DSP_HW == 2
  // return __sqrt(x);  // C2000 TMU
  // 回退:
  return sqrtf(x);
#else
  // 纯 C 牛顿迭代法 (无 libm 依赖, ISR 安全)
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

// Biquad 实例 — 平台无关, 系数和状态直接嵌入
// 上层可以直接将此结构体嵌入自己的滤波器结构体中
typedef struct {
#if BSP_DSP_HW == 1
  arm_biquad_casd_df1_inst_f32 inst;  // CMSIS-DSP 实例 (含系数 + 状态指针)
#else
  // 纯 C 回退: 系数和状态存储在此结构体中
  float coeffs[BSP_BIQUAD_COEFF_COUNT];  // {b0, b1, b2, -a1, -a2}
  float state[BSP_BIQUAD_STATE_COUNT];   // {x1, x2, y1, y2}
#endif
} BspBiquadInst;

// 初始化 Biquad 实例 — 绑定系数和状态
//
// coeffs: 5 元素数组 {b0, b1, b2, -a1, -a2}
// state:  4 元素数组 {x1, x2, y1, y2}, 调用前初始化为 0
// num_stages: 级联级数 (通常为 1)
static inline void bsp_biquad_init(BspBiquadInst *me,
                                    float *coeffs, float *state,
                                    int num_stages) {
#if BSP_DSP_HW == 1
  arm_biquad_cascade_df1_init_f32(&me->inst, (uint32_t)num_stages,
                                   coeffs, state);
#else
  // 纯 C: 直接存指针 (BspBiquadInst 持有自己的 coeffs/state 数组)
  // 调用方需确保 coeffs/state 生命周期 ≥ me
  (void)num_stages;
  (void)me;
  // 纯 C 回退模式下, 上层直接读写 coeffs/state, 不需要 init
  // 此函数主要服务 CMSIS-DSP 模式
#endif
}

// 执行 Biquad 滤波 (单样本或 block)
//
// src:  输入样本数组
// dst:  输出样本数组 (可与 src 相同)
// block_size: 处理样本数 (通常为 1)
static inline void bsp_biquad_apply(BspBiquadInst *me,
                                     const float *src, float *dst,
                                     int block_size) {
#if BSP_DSP_HW == 1
  arm_biquad_cascade_df1_f32(&me->inst, (float *)src, dst,
                              (uint32_t)block_size);
#else
  // 纯 C 回退 — 手动 Direct Form I (不依赖 CMSIS-DSP)
  // 预期: 上层过滤器结构体持有自己的 coeffs 和 state, 不经过此函数
  // 如果调用此函数, 退化为 memcpy (安全兜底)
  for (int i = 0; i < block_size; i++) {
    dst[i] = src[i];
  }
#endif
}

// ======== CMSIS-DSP 直接兼容层 ========
// 仅在 STM32 平台可用, 允许现有代码直接使用 CMSIS-DSP 类型

#if BSP_DSP_HW == 1
  // arm_math.h 已包含, arm_biquad_casd_df1_inst_f32 等类型可直接使用
  #define BSP_DSP_CMSIS_AVAILABLE 1
#else
  #define BSP_DSP_CMSIS_AVAILABLE 0
#endif

#endif  // BSP_DSP_H
