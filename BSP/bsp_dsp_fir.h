// BSP DSP 扩展 — FIR 滤波器硬件加速抽象
//
// 接续 BSP/bsp_dsp.h (§5), 添加 FIR 滤波器平台抽象:
//   fir_f32 — 单级 FIR 滤波 (CMSIS-DSP arm_fir_f32 / C2000 FIR_FP / 纯C)
//
// 使用方式:
//   #include "bsp_dsp.h"
//   #include "bsp_dsp_fir.h"
//   bsp_fir_init(&fir, coeffs, state, num_taps);
//   float y = bsp_fir_apply(&fir, x);

#ifndef BSP_DSP_FIR_H
#define BSP_DSP_FIR_H

#include "bsp_dsp.h"

// ======== FIR 实例结构体 ========

// 最大 FIR 阶数 (纯C回退用)
#define BSP_FIR_MAX_TAPS  128

typedef struct {
  float *coeffs;          // FIR 系数数组 (长度 = num_taps)
  float *state;           // 延迟线 (长度 = num_taps + block_size - 1)
  int   num_taps;         // 抽头数 (滤波器阶数 + 1)

#if BSP_DSP_ARCH == 2 || BSP_DSP_ARCH == 1
  arm_fir_instance_f32  cmsis_inst;   // CMSIS-DSP 实例
  float coeffs_buf[BSP_FIR_MAX_TAPS];  // 系数副本 (CMSIS-DSP 要求)
  float state_buf[BSP_FIR_MAX_TAPS * 2]; // 延迟线缓冲
#else
  // 纯C/C2000: 直接使用 coeffs/state 指针
  float coeffs_buf[BSP_FIR_MAX_TAPS];
  float state_buf[BSP_FIR_MAX_TAPS];
#endif
} BspFirInst;

// ======== FIR 初始化 ========

static inline void bsp_fir_init(BspFirInst *me, const float *coeffs,
                                 int num_taps) {
  me->num_taps = num_taps;

#if BSP_DSP_ARCH == 2 || BSP_DSP_ARCH == 1
  // CMSIS-DSP: 复制系数到内部缓冲
  for (int i = 0; i < num_taps && i < BSP_FIR_MAX_TAPS; i++) {
    me->coeffs_buf[i] = coeffs[i];
  }
  for (int i = 0; i < num_taps * 2 && i < BSP_FIR_MAX_TAPS * 2; i++) {
    me->state_buf[i] = 0.0f;
  }
  arm_fir_init_f32(&me->cmsis_inst, (uint16_t)num_taps,
                   me->coeffs_buf, me->state_buf, 1);
  me->coeffs = me->coeffs_buf;
  me->state = me->state_buf;
#else
  // 纯C/C2000: 拷贝系数, 清零延迟线
  for (int i = 0; i < num_taps && i < BSP_FIR_MAX_TAPS; i++) {
    me->coeffs_buf[i] = coeffs[i];
    me->state_buf[i] = 0.0f;
  }
  me->coeffs = me->coeffs_buf;
  me->state = me->state_buf;
#endif
}

// ======== FIR 单步滤波 ========

// 输入 x, 输出 y
static inline float bsp_fir_apply(BspFirInst *me, float x) {
  float y;

#if BSP_DSP_ARCH == 2 || BSP_DSP_ARCH == 1
  arm_fir_f32(&me->cmsis_inst, &x, &y, 1);
#elif BSP_DSP_ARCH == 3
  // C2000: 使用 FPU FIR (如果有 DSP.h)
  // 简化回退: 纯C实现
  {
    int n = me->num_taps;
    float *c = me->coeffs;
    float *s = me->state;

    // 移位延迟线
    for (int i = n - 1; i > 0; i--) {
      s[i] = s[i - 1];
    }
    s[0] = x;

    // 卷积
    y = 0.0f;
    for (int i = 0; i < n; i++) {
      y += c[i] * s[i];
    }
  }
#else
  // 纯C: 移位延迟线 + 卷积
  {
    int n = me->num_taps;
    float *c = me->coeffs;
    float *s = me->state;

    for (int i = n - 1; i > 0; i--) {
      s[i] = s[i - 1];
    }
    s[0] = x;

    y = 0.0f;
    for (int i = 0; i < n; i++) {
      y += c[i] * s[i];
    }
  }
#endif

  return y;
}

// ======== FIR Block 滤波 ========

static inline void bsp_fir_apply_block(BspFirInst *me, const float *src,
                                        float *dst, int block_size) {
  // 统一走逐样本路径: CMSIS-DSP 在 init 时 blockSize=1,
  // arm_fir_f32 必须与 init 的 blockSize 匹配, 否则结果错误
  for (int i = 0; i < block_size; i++) {
    dst[i] = bsp_fir_apply(me, src[i]);
  }
}

#endif  // BSP_DSP_FIR_H
