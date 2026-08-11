// FFT 窗函数 — 18 种经典窗函数生成器 (纯C, 无硬件依赖)
//
// 来源: TI controlSUITE FPU library (fpu_fft_*.h, 18 个独立头文件)
//        + FixedPointLib (fft_*_Q31.h, 定点 Q31 版本)
// 翻译为 C-OOP: 统一 float 窗函数生成器, 单份实现覆盖所有窗口类型
//
// 应用场景:
//   - FFT 频谱泄漏抑制 (与 bsp_dsp_fft.h 配合)
//   - FIR 滤波器设计 (窗函数法: 理想滤波器 × 窗 = 实际 FIR 系数)
//   - 功率谱估计 (Welch/Barlett 方法)
//
// 调用方式:
//   float win[256];
//   fft_win_generate(FFT_WIN_HANN, win, 256);   // 生成 256 点 Hann 窗
//   fft_win_apply(win, data, 256);               // 逐点乘窗

#ifndef COMP_FFT_WINDOW_H
#define COMP_FFT_WINDOW_H

#include <math.h>

// 用独立的 float 常量避免 double→float 隐式缩窄转换
#define FFT_PI  3.1415927f
#define FFT_2PI 6.2831855f

#ifdef __cplusplus
extern "C" {
#endif

// ======================= 窗函数类型枚举 =======================

typedef enum {
  FFT_WIN_RECTANGULAR = 0,     // 矩形窗 — 等效不加窗, 主瓣最窄, 旁瓣 -13dB
  FFT_WIN_BARTLETT,            // Bartlett (三角窗) — 旁瓣 -25dB
  FFT_WIN_HANN,                // Hann (Hanning) — 旁瓣 -31.5dB, 最常用
  FFT_WIN_HAMMING,             // Hamming — 旁瓣 -42.7dB, 第一旁瓣抑制优于 Hann
  FFT_WIN_BLACKMAN,            // Blackman — 旁瓣 -58dB, 3 项余弦
  FFT_WIN_BLACKMAN_HARRIS,     // Blackman-Harris — 旁瓣 -92dB, 4 项余弦, 通用最佳
  FFT_WIN_BLACKMAN_NUTTALL,    // Blackman-Nuttall — 旁瓣 -98dB
  FFT_WIN_NUTTALL,             // Nuttall (4 项最小旁瓣) — 旁瓣 -98dB
  FFT_WIN_FLATTOP,             // Flat Top — 幅值精度最高 (通带波纹 < 0.01dB), 主瓣最宽
  FFT_WIN_BOHMAN,              // Bohman — 卷积窗, 旁瓣 -46dB (自卷积三角窗)
  FFT_WIN_GAUSS,               // Gaussian (α=2.5) — 无旁瓣振铃, 时频最优
  FFT_WIN_KAISER,              // Kaiser (β=6) — 可调 β 控制主瓣/旁瓣权衡
  FFT_WIN_CHEBYSHEV,           // Dolph-Chebyshev — 等波纹旁瓣, 给定衰减下主瓣最窄
  FFT_WIN_TUKEY,               // Tukey (α=0.5, 余弦渐变矩形) — 矩形-Hann 混合
  FFT_WIN_PARZEN,              // Parzen — 旁瓣 -53dB, 4 次分段多项式
  FFT_WIN_TAYLOR,              // Taylor — 近场等旁瓣, 可调旁瓣数
  FFT_WIN_TRIANGULAR,          // Triangular — 与 Bartlett 几乎相同, 端点处理略异
  FFT_WIN_BARTHANN,            // Bartlett-Hann — Bartlett + Hann 混合系数
  FFT_WIN_COUNT
} FftWinType;

// ======================= 窗函数生成 =======================

// 生成 N 点窗系数到 win[] 数组 (调用者分配, 长度 N)
//   type: 窗类型
//   win:  输出系数数组 [0..N-1]
//   N:    窗长度 (≥2)
//
// 注: 大部分窗的系数已归一化 (相干增益 ≈1), 可用于 FIR 设计或 FFT 预处理
static inline void fft_win_generate(FftWinType type, float *win, int size) {
  if (size < 2) {
    if (size == 1) { win[0] = 1.0f; }
    return;
  }

  float m_f = (float)(size - 1);  // m = size-1 用于对称窗

  switch (type) {

  // ---- 矩形窗: w[n] = 1 ----
  case FFT_WIN_RECTANGULAR:
    for (int n = 0; n < size;n++) {
      win[n] = 1.0f;
    }
    break;

  // ---- Bartlett (三角窗): w[n] = 1 - |2n/(N-1) - 1| ----
  case FFT_WIN_BARTLETT:
    for (int n = 0; n < size;n++) {
      win[n] = 1.0f - fabsf(2.0f * (float)n / m_f - 1.0f);
    }
    break;

  // ---- Hann: w[n] = 0.5 - 0.5*cos(2πn/(N-1)) ----
  case FFT_WIN_HANN:
    for (int n = 0; n < size;n++) {
      win[n] = 0.5f - 0.5f * cosf(2.0f * FFT_PI * (float)n / m_f);
    }
    break;

  // ---- Hamming: w[n] = 0.54 - 0.46*cos(2πn/(N-1)) ----
  // 第一旁瓣比 Hann 低 ~11dB, 但远旁瓣不衰减
  case FFT_WIN_HAMMING:
    for (int n = 0; n < size;n++) {
      win[n] = 0.54f - 0.46f * cosf(2.0f * FFT_PI * (float)n / m_f);
    }
    break;

  // ---- Blackman (3项): w[n] = a0 - a1*cos(2πn/N) + a2*cos(4πn/N) ----
  // a0=0.42, a1=0.5, a2=0.08
  case FFT_WIN_BLACKMAN:
    for (int n = 0; n < size;n++) {
      float theta = 2.0f * FFT_PI * (float)n / m_f;
      win[n] = 0.42f - 0.5f * cosf(theta) + 0.08f * cosf(2.0f * theta);
    }
    break;

  // ---- Blackman-Harris (4项最小旁瓣): 4 项余弦和 ----
  // a0=0.35875, a1=0.48829, a2=0.14128, a3=0.01168
  case FFT_WIN_BLACKMAN_HARRIS:
    for (int n = 0; n < size;n++) {
      float theta = 2.0f * FFT_PI * (float)n / m_f;
      win[n] = 0.35875f - 0.48829f * cosf(theta)
              + 0.14128f * cosf(2.0f * theta)
              - 0.01168f * cosf(3.0f * theta);
    }
    break;

  // ---- Blackman-Nuttall: a0=0.3635819, a1=0.4891775, a2=0.1365995, a3=0.0106411 ----
  case FFT_WIN_BLACKMAN_NUTTALL:
    for (int n = 0; n < size;n++) {
      float theta = 2.0f * FFT_PI * (float)n / m_f;
      win[n] = 0.3635819f - 0.4891775f * cosf(theta)
              + 0.1365995f * cosf(2.0f * theta)
              - 0.0106411f * cosf(3.0f * theta);
    }
    break;

  // ---- Nuttall (4项): a0=0.355768, a1=0.487396, a2=0.144232, a3=0.012604 ----
  case FFT_WIN_NUTTALL:
    for (int n = 0; n < size;n++) {
      float theta = 2.0f * FFT_PI * (float)n / m_f;
      win[n] = 0.355768f - 0.487396f * cosf(theta)
              + 0.144232f * cosf(2.0f * theta)
              - 0.012604f * cosf(3.0f * theta);
    }
    break;

  // ---- Flat Top: 5 项余弦, 最小通带波纹 (<0.01dB) ----
  // a0=1.0, a1=1.93, a2=1.29, a3=0.388, a4=0.028
  // 生成后归一化 (除以 a0=1.0 → 无需除, 但需除峰值 ≈4.636)
  case FFT_WIN_FLATTOP: {
    for (int n = 0; n < size;n++) {
      float theta = 2.0f * FFT_PI * (float)n / m_f;
      win[n] = 1.0f - 1.93f * cosf(theta)
              + 1.29f * cosf(2.0f * theta)
              - 0.388f * cosf(3.0f * theta)
              + 0.028f * cosf(4.0f * theta);
    }
    // 归一化到峰值 = 1
    float peak = 4.636f;
    for (int n = 0; n < size;n++) {
      win[n] /= peak;
    }
    break;
  }

  // ---- Bohman: w[n] = (1-|d|)cos(π|d|) + (1/π)sin(π|d|), d = 2n/(N-1)-1 ----
  case FFT_WIN_BOHMAN:
    for (int n = 0; n < size;n++) {
      float d = 2.0f * (float)n / m_f - 1.0f;   // d ∈ [-1, 1]
      float ad = fabsf(d);
      if (ad > 1.0f) { win[n] = 0.0f; }
      else {
        win[n] = (1.0f - ad) * cosf(FFT_PI * ad) + (1.0f / FFT_PI) * sinf(FFT_PI * ad);
      }
    }
    break;

  // ---- Gaussian (α=2.5): w[n] = exp(-0.5*(α*2n/(N-1)-α)²) ----
  // α=2.5 → 端点值为 exp(-3.125) ≈ 0.044, 时频最优
  case FFT_WIN_GAUSS: {
    float alpha = 2.5f;
    for (int n = 0; n < size;n++) {
      float t = alpha * (2.0f * (float)n / m_f - 1.0f);
      win[n] = expf(-0.5f * t * t);
    }
    break;
  }

  // ---- Kaiser (β=6): w[n] = I₀(β√(1-(2n/(N-1)-1)²)) / I₀(β) ----
  // I₀ = 零阶修正 Bessel 函数, 用级数展开近似
  case FFT_WIN_KAISER: {
    float beta = 6.0f;
    // 先计算 I₀(β) 分母
    float i0_beta = 1.0f;
    { float term = 1.0f, half_b = beta * 0.5f;
      for (int k = 1; k < 25; k++) {
        term *= half_b * half_b / (float)(k * k);
        i0_beta += term;
      }
    }
    for (int n = 0; n < size;n++) {
      float t = 2.0f * (float)n / m_f - 1.0f;  // t ∈ [-1, 1]
      float arg = beta * sqrtf(1.0f - t * t);
      // I₀(arg) 级数展开
      float i0_arg = 1.0f;
      { float term = 1.0f, half_a = arg * 0.5f;
        for (int k = 1; k < 25; k++) {
          term *= half_a * half_a / (float)(k * k);
          i0_arg += term;
        }
      }
      win[n] = i0_arg / i0_beta;
    }
    break;
  }

  // ---- Dolph-Chebyshev (衰减 80dB): 等波纹旁瓣 ----
  // 变换域: w[k] = T_M(x₀ cos(πk/M)), M=N-1, x₀=cosh(acosh(10^(α/20))/M)
  case FFT_WIN_CHEBYSHEV: {
    float atten_db = 80.0f;   // 旁瓣衰减 (可调)
    float alpha = powf(10.0f, atten_db / 20.0f);
    float x0 = coshf(acoshf(alpha) / m_f);
    for (int n = 0; n < size;n++) {
      float theta = 2.0f * FFT_PI * (float)n / m_f;
      // Chebyshev 多项式 T_M(x): |x|≤1 时 T_M(x)=cos(M·acos(x))
      float x = x0 * cosf(theta * 0.5f);
      float w_val;
      if (x <= 1.0f) {
        w_val = cosf(m_f * acosf(x));
      } else {
        w_val = coshf(m_f * acoshf(x));
      }
      // 取绝对值, 归一化
      win[n] = fabsf(w_val);
    }
    // 归一化到峰值 = 1
    float peak = win[0];
    if (peak > 0.0f) {
      for (int n = 0; n < size;n++) {
        win[n] /= peak;
      }
    }
    break;
  }

  // ---- Tukey (α=0.5): 余弦渐变缘 + 中部矩形 (余弦矩形混合) ----
  // α=0 → 矩形窗; α=1 → Hann 窗
  case FFT_WIN_TUKEY: {
    float alpha_tukey = 0.5f;
    float half_alpha = alpha_tukey * 0.5f;
    for (int n = 0; n < size;n++) {
      float t = (float)n / m_f;
      if (t < half_alpha) {
        win[n] = 0.5f * (1.0f - cosf(FFT_PI * t / half_alpha));
      } else if (t > 1.0f - half_alpha) {
        win[n] = 0.5f * (1.0f - cosf(FFT_PI * (1.0f - t) / half_alpha));
      } else {
        win[n] = 1.0f;
      }
    }
    break;
  }

  // ---- Parzen: 分段三次多项式 ----
  // w[n] = 1 - 6|d|² + 6|d|³   (|d|≤0.5)
  // w[n] = 2(1-|d|)³             (0.5<|d|≤1)
  case FFT_WIN_PARZEN:
    for (int n = 0; n < size;n++) {
      float d = 2.0f * (float)n / m_f - 1.0f;  // d ∈ [-1, 1]
      float ad = fabsf(d);
      if (ad <= 0.5f) {
        win[n] = 1.0f - 6.0f * ad * ad + 6.0f * ad * ad * ad;
      } else if (ad <= 1.0f) {
        win[n] = 2.0f * (1.0f - ad) * (1.0f - ad) * (1.0f - ad);
      } else {
        win[n] = 0.0f;
      }
    }
    break;

  // ---- Taylor (nbar=4, sll=-30dB): 近场等旁瓣, 可调旁瓣数 ----
  // 简化实现: 4 旁瓣 Taylor 窗近似
  // F_m = m - 1 + 0.5  (m=0..nbar-1)
  case FFT_WIN_TAYLOR: {
    float nbar = 4.0f;    // 等旁瓣数
    float sll = 30.0f;    // 旁瓣电平 (dB)
    float a_val = acoshf(powf(10.0f, sll / 20.0f)) / FFT_PI;
    float sig2 = nbar * nbar / (a_val * a_val + (nbar - 0.5f) * (nbar - 0.5f));
    for (int n = 0; n < size; n++) {
      float x = 2.0f * FFT_PI * ((float)n / m_f - 0.5f);
      float w_val = 1.0f;
      for (int m = 1; m < (int)nbar; m++) {
        float fm = sig2 * (a_val * a_val + ((float)m - 0.5f) * ((float)m - 0.5f));
        float num = 1.0f - (x * x) / ((float)m * (float)m * FFT_PI * FFT_PI);
        float den = 1.0f - (x * x) / (fm * FFT_PI * FFT_PI);
        if (fabsf(den) > 1e-9f) {
          w_val *= num / den;
        }
      }
      win[n] = fabsf(w_val);
    }
    // 归一化
    float peak = win[0];
    if (peak > 0.0f) {
      for (int n = 0; n < size;n++) {
        win[n] /= peak;
      }
    }
    break;
  }

  // ---- Triangular: w[n] = 1 - |2n-(N-1)|/(N-1) ----
  // 与 Bartlett 等价, 但使用绝对值定义
  case FFT_WIN_TRIANGULAR:
    for (int n = 0; n < size;n++) {
      win[n] = 1.0f - fabsf((2.0f * (float)n - m_f) / m_f);
    }
    break;

  // ---- Bartlett-Hann: a0=0.62, a1=0.48, a2=0.38 ----
  // w[n] = a0 - a1|2n/(N-1)-1| - a2*cos(2πn/(N-1))
  case FFT_WIN_BARTHANN:
    for (int n = 0; n < size;n++) {
      float t = fabsf(2.0f * (float)n / m_f - 1.0f);
      win[n] = 0.62f - 0.48f * t + 0.38f * cosf(2.0f * FFT_PI * (float)n / m_f);
    }
    break;

  default:
    // 回退: 矩形窗
    for (int n = 0; n < size;n++) {
      win[n] = 1.0f;
    }
    break;
  }
}

// ======================= 窗函数应用 (逐点乘) =======================

// 对信号 data[] 逐点乘窗, 结果存回 data (就地) 或存到 dst (非就地)
//   win:  窗系数数组 (长度 N)
//   data: 输入数据 (长度 N)
//   dst:  输出数据 (长度 N), 可 == data 就地
//   N:    数据长度
static inline void fft_win_apply(const float *win, const float *data,
                                  float *dst, int size) {
  for (int n = 0; n < size; n++) {
    dst[n] = data[n] * win[n];
  }
}

// ======================= 相干增益 / 噪声带宽 =======================

// 计算窗的相干增益 (coherent gain)
//   CG = (1/N) × Σ w[n]
// 用于 FFT 幅值校正: 实际幅值 = FFT 幅值 / CG
static inline float fft_win_coherent_gain(const float *win, int size) {
  float sum = 0.0f;
  for (int n = 0; n < size; n++) {
    sum += win[n];
  }
  return sum / (float)size;
}

// 计算窗的等效噪声带宽 (ENBW, 单位: bin)
//   ENBW = N × Σ(w[n]²) / (Σ w[n])²
// 用于噪声功率校正
static inline float fft_win_enbw(const float *win, int size) {
  float sum_w = 0.0f, sum_w2 = 0.0f;
  for (int n = 0; n < size; n++) {
    sum_w += win[n];
    sum_w2 += win[n] * win[n];
  }
  if (sum_w == 0.0f) return 1.0f;
  return (float)size * sum_w2 / (sum_w * sum_w);
}

#ifdef __cplusplus
}
#endif

#endif  // COMP_FFT_WINDOW_H
