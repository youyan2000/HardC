// 逐谐波分析 — Goertzel 谐振器频谱 (H1..H50) + 总谐波失真 THD
//
// 来源: TI C2000Ware Digital Power SDK
//   libraries/energy-metrology_library/energy_metrology_f28p55
//   (metrology_calculations.c goertzelMagnitude + USE_GOERTZEL_THD 分支)
// 翻译为 C-OOP 纯C float 版本. TI 的 2048 点 FFT 分支 (fpu_rfft) 为 C2000
// 硬件绑定, 不移植 — Goertzel 逐谐波谐振在任意平台等效
//
// 算法 (goertzelMagnitude):
//   1. 窗口 = 整数个电网周期 (N 个样本), 谐波 bin 精确落点, 无需窗函数
//   2. 对第 h 次谐波, 角频率 ω_h = 2π·h·grid_freq/sample_rate
//      2 阶谐振器: q0 = 2·cos(ω_h)·q1 − q2 + x[k]
//   3. N 点后: DFT bin 实部 re = q1 − q2·cos(ω_h), 虚部 im = q2·sin(ω_h)
//      幅值 A_h = 2·sqrt(re²+im²)/N   (纯正弦精确 bin 时 = 峰值幅值)
//   4. THD = sqrt(Σ_{h=2..50} A_h²)/A_1 × 100
//
// 注意1: 窗口须为整数个电网周期 (N = cycles·Fs/freq), 否则频谱泄漏
//   用 power_goertzel_window_n(cycles, Fs, freq) 计算 N
// 注意2: 缓冲由调用者提供 (零 malloc), 长度 ≥ N, 典型 1600×float ≈ 6.4KB
// 注意3: 最高谐波 h_max ≤ Fs/(2·grid_freq) (Nyquist), TI 默认 50
// 注意4: 与 comp_power_fund.h (基波相关解调) 互补: 本组件给逐次谐波频谱,
//   fund 给基波功率; THD 两者都算, 本组件按逐次谐波精算, fund 按宽频−基波估计

#ifndef COMP_POWER_GOERTZEL_H
#define COMP_POWER_GOERTZEL_H

#include <math.h>
#include <stdint.h>

// ======================= PowerGoertzel (逐谐波分析器) =======================

typedef struct {
  // 参数
  float sample_rate;      // 参数: 采样频率 (Hz)
  float grid_freq;        // 参数: 电网频率 (Hz, 标称 50/60)
  uint16_t window_n;      // 参数: 窗口样本数 N (整数个周期, 非 2 的幂亦可)

  // 内部
  float *buffer;          // 调用者提供的环形缓冲 (长度 ≥ window_n)
  uint16_t write_idx;     // 环形缓冲写指针 (同时指向最早的样本)
  uint32_t sample_count;  // 已采样本数 (饱和到 window_n, 判缓冲是否填满)

  // 输出 (analyze 后有效)
  float thd;              // 输出: 总谐波失真 (%)
} PowerGoertzel;

// 计算整数个周期的窗口样本数 N = round(cycles·sample_rate/grid_freq)
static inline uint16_t power_goertzel_window_n(uint16_t cycles,
                                               float sample_rate,
                                               float grid_freq) {
  return (uint16_t)((float)cycles * sample_rate / grid_freq + 0.5f);
}

// 初始化
//   buffer    — 调用者提供环形缓冲, 长度 ≥ window_n (零 malloc)
//   window_n  — 窗口样本数 (用 power_goertzel_window_n 计算整数周期 N)
static inline void power_goertzel_init(PowerGoertzel *me, float *buffer,
                                       float sample_rate, float grid_freq,
                                       uint16_t window_n) {
  me->sample_rate = sample_rate;
  me->grid_freq = grid_freq;
  me->window_n = window_n;

  me->buffer = buffer;
  me->write_idx = 0;
  me->sample_count = 0u;

  me->thd = 0.0f;
}

// 单采样写入 — ISR 热路径, 每采样周期调用一次
static inline void power_goertzel_sample(PowerGoertzel *me, float x) {
  me->buffer[me->write_idx] = x;
  me->write_idx++;
  if (me->write_idx == me->window_n) {
    me->write_idx = 0;
  }
  me->sample_count++;
  if (me->sample_count > me->window_n) {
    me->sample_count = me->window_n;
  }
}

// 频谱分析 — 每窗口 (N 样本) 调用一次, 逐谐波跑 Goertzel 谐振器
//   mag    — 输出幅值谱, 长度 ≥ max_h+1; mag[0]=0, mag[h]=第 h 次谐波峰值幅值
//   max_h  — 最高谐波次数 (≤ Fs/(2·grid_freq), TI 默认 50)
//   返回: 1 = 缓冲已填满并计算, 0 = 样本不足 (结果无效)
static inline int power_goertzel_analyze(PowerGoertzel *me, float *mag,
                                         uint16_t max_h) {
  if (me->sample_count < me->window_n) {
    return 0;
  }

  const float two_pi = 6.28318530718f;
  const float inv_n = 2.0f / (float)me->window_n;
  const uint16_t start = me->write_idx;   // 最早的样本
  uint16_t h, i;
  float sum_harm = 0.0f;

  mag[0] = 0.0f;

  for (h = 1u; h <= max_h; h++) {
    // 谐波角频率与谐振器系数
    float w = two_pi * (float)h * me->grid_freq / me->sample_rate;
    float coeff = 2.0f * cosf(w);
    float q1 = 0.0f, q2 = 0.0f;
    uint16_t idx = start;

    // 窗口 N 点按时间序 (最老→最新) 跑 2 阶谐振器
    for (i = 0u; i < me->window_n; i++) {
      float q0 = coeff * q1 - q2 + me->buffer[idx];
      q2 = q1;
      q1 = q0;
      idx++;
      if (idx == me->window_n) {
        idx = 0;
      }
    }

    // DFT bin: re = q1 − q2·cos(ω), im = q2·sin(ω), 幅值 = 2·|X_k|/N
    float re = q1 - q2 * cosf(w);
    float im = q2 * sinf(w);
    float amp = inv_n * sqrtf(re * re + im * im);

    mag[h] = amp;
    if (h > 1u) {
      sum_harm += amp * amp;
    }
  }

  // THD = sqrt(Σ_{h≥2} A_h²)/A_1 × 100
  me->thd = (mag[1] > 0.0f) ? (sqrtf(sum_harm) / mag[1]) * 100.0f : 0.0f;

  return 1;
}

#endif  // COMP_POWER_GOERTZEL_H
