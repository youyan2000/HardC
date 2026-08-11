// BSP DSP 扩展 — FFT 快速傅里叶变换硬件加速抽象
//
// 接续 BSP/bsp_dsp.h, 添加 FFT 平台抽象:
//   rfft_f32 — 实数 FFT (CMSIS-DSP arm_rfft_f32 / C2000 CFFT_f32 / 纯C)
//   cfft_f32 — 复数 FFT (CMSIS-DSP arm_cfft_f32 / C2000 CFFT_f32 / 纯C)
//
// 使用方式:
//   bsp_rfft_init(&rfft, fft_size);
//   bsp_rfft_apply(&rfft, src, dst);   // src 是实数, dst 是复数
//
// 注: 纯C 回退不使用蝶形算法 (避免查表依赖), 用 CMSIS-DSP 或 C2000 时才有高效实现
//     对于没有硬件加速的平台, FFT 级数较小 (N≤256) 时用直接 DFT 也可接受

#ifndef BSP_DSP_FFT_H
#define BSP_DSP_FFT_H

#include "bsp_dsp.h"
#include <string.h>

#ifndef M_2PI
#define M_2PI 6.283185f
#endif

// ======== RFFT (实数 FFT) 实例 ========

#define BSP_RFFT_MAX_SIZE  1024

typedef struct {
  int fft_size;           // FFT 点数 (必须 2^n, ≤ BSP_RFFT_MAX_SIZE)

#if BSP_DSP_ARCH == 2 || BSP_DSP_ARCH == 1
  arm_rfft_fast_instance_f32  cmsis_rfft;
#else
  // 纯C: 存储旋转因子表 (sin/cos)
  float twiddle_sin[BSP_RFFT_MAX_SIZE / 2];
  float twiddle_cos[BSP_RFFT_MAX_SIZE / 2];
#endif
} BspRfftInst;

// ======== RFFT 初始化 ========

static inline void bsp_rfft_init(BspRfftInst *me, int fft_size) {
  me->fft_size = fft_size;

#if BSP_DSP_ARCH == 2 || BSP_DSP_ARCH == 1
  arm_rfft_fast_init_f32(&me->cmsis_rfft, (uint16_t)fft_size);
#else
  // 纯C: 预计算旋转因子 W_N^k = exp(-j*2π*k/N)
  for (int k = 0; k < fft_size / 2; k++) {
    float angle = -M_2PI * (float)k / (float)fft_size;
    me->twiddle_sin[k] = sinf(angle);
    me->twiddle_cos[k] = cosf(angle);
  }
#endif
}

// ======== RFFT 执行 ========

// src: 实数输入 (长度 = fft_size)
// dst: 复数输出 (长度 = fft_size, 实虚交替: re[0], im[0], re[1], im[1], ...)
//      频谱格式 (CMSIS-DSP 兼容):
//        dst[0]   = DC (实部)
//        dst[1]   = 0
//        dst[2]   = re[1]
//        dst[3]   = im[1]
//        ...
//        dst[N/2]   = Nyquist (实部)
//        dst[N/2+1] = 0
static inline void bsp_rfft_apply(BspRfftInst *me, const float *src,
                                   float *dst) {
#if BSP_DSP_ARCH == 2 || BSP_DSP_ARCH == 1
  // CMSIS-DSP: 需要临时缓冲区 (merge buffer)
  // dst 也用作临时缓冲区 (就地运算)
  memcpy(dst, src, me->fft_size * sizeof(float));
  arm_rfft_fast_f32(&me->cmsis_rfft, dst, dst, 0);  // 0 = 正向
#elif BSP_DSP_ARCH == 3
  // C2000: CFFT_f32 是复数 FFT, 实数 FFT 先打包为复数再走纯C DFT
  // 实际使用时用户会链接 C2000 DSP 库 (RFFT_f32/CFFT_f32)
  // 回退: 打包 → 纯C DFT
  memcpy(dst, src, (size_t)me->fft_size * sizeof(float));
  for (int i = (int)(me->fft_size) - 1; i >= 0; i--) {
    dst[2 * i + 1] = 0.0f;
    dst[2 * i]     = dst[i];
  }
  // 纯C DFT 计算 (打包后的复数数据)
  {
    size_t n = (size_t)me->fft_size;
    for (size_t k = 0; k < n; k++) {
      float re = 0.0f, im_val = 0.0f;
      for (size_t i = 0; i < n; i++) {
        size_t idx = (i * k) % n;
        float cr = me->twiddle_cos[idx];
        float si_val = me->twiddle_sin[idx];    // sin_val = sin(-θ) = -sin(θ)
        float xr = dst[2 * i];                   // 实部
        float xi = dst[2 * i + 1];               // 虚部
        // DFT: X[k] = Σ x[n]·exp(-j2πkn/N) = Σ (xr+j·xi)(cos-j·sin)
        // = Σ (xr·cos+xi·sin) + j·Σ (xi·cos-xr·sin)
        re    += xr * cr - xi * si_val;          // si_val = -sinθ, -xi·(-sinθ)=+xi·sinθ ✓
        im_val += xi * cr + xr * si_val;         // xr·si_val = xr·(-sinθ) = -xr·sinθ ✓
      }
      dst[2 * k]     = re;
      dst[2 * k + 1] = im_val;
    }
  }
#else
  // 纯C: 直接 DFT (只用于小型 FFT 或离线分析)
  // O(N²) 慢, 但在无 DSP 库的平台上唯一通用方案
  size_t n = (size_t)me->fft_size;
  for (size_t k = 0; k < n; k++) {
    float re = 0.0f, im = 0.0f;
    for (size_t i = 0; i < n; i++) {
      size_t idx = (i * k) % n;
      float cos_val = me->twiddle_cos[idx];
      float sin_val = me->twiddle_sin[idx];   // sin_val = sin(-θ) = -sin(θ)
      // DFT: im = -Σ x·sin(θ) = -Σ x·(-sin_val) = +Σ x·sin_val
      re += src[i] * cos_val;
      im += src[i] * sin_val;
    }
    dst[2 * k]     = re;
    dst[2 * k + 1] = im;
  }
#endif
}

// ======== CFFT (复数 FFT) 实例 ========

#define BSP_CFFT_MAX_SIZE  1024

typedef struct {
  int fft_size;

#if BSP_DSP_ARCH == 2 || BSP_DSP_ARCH == 1
  arm_cfft_instance_f32  cmsis_cfft;
#else
  float twiddle_sin[BSP_CFFT_MAX_SIZE];
  float twiddle_cos[BSP_CFFT_MAX_SIZE];
#endif
} BspCfftInst;

// ======== CFFT 初始化 ========

static inline void bsp_cfft_init(BspCfftInst *me, int fft_size) {
  me->fft_size = fft_size;

#if BSP_DSP_ARCH == 2 || BSP_DSP_ARCH == 1
  arm_cfft_init_f32(&me->cmsis_cfft, (uint16_t)fft_size);
#else
  for (int k = 0; k < fft_size; k++) {
    float angle = -M_2PI * (float)k / (float)fft_size;
    me->twiddle_sin[k] = sinf(angle);
    me->twiddle_cos[k] = cosf(angle);
  }
#endif
}

// ======== CFFT 执行 ========

// data: 复数数组 (实虚交替, 长度 = 2 * fft_size), 就地运算
static inline void bsp_cfft_apply(BspCfftInst *me, float *data) {
#if BSP_DSP_ARCH == 2 || BSP_DSP_ARCH == 1
  arm_cfft_f32(&me->cmsis_cfft, data, 0, 1);  // 0=正向, 1=位反转
#else
  // 纯C DFT 回退
  size_t n = (size_t)me->fft_size;
  // 静态临时缓冲 (避免栈溢出, 非可重入 — 嵌入式通常单线程)
  static float temp[BSP_CFFT_MAX_SIZE * 2];
  for (size_t k = 0; k < n; k++) {
    float re = 0.0f, im = 0.0f;
    for (size_t i = 0; i < n; i++) {
      size_t idx = (i * k) % n;
      float xr = data[2 * i];
      float xi = data[2 * i + 1];
      float cr = me->twiddle_cos[idx];
      float si = me->twiddle_sin[idx];
      re += xr * cr - xi * si;
      im += xr * si + xi * cr;
    }
    temp[2 * k]     = re;
    temp[2 * k + 1] = im;
  }
  memcpy(data, temp, n * 2 * sizeof(float));
#endif
}

#endif  // BSP_DSP_FFT_H
