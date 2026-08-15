// ADC 信号处理共享承诺 —— 把 ADC 原始码变成干净工程量的纯算法流水线
//
// 定位: Components/adc 域的"信号处理承诺", 与 comp_adc.h (接口/虚表承诺) 平行。
//   职责 = 校准 + 滤波 的组合流水线, 与平台/设备解耦 (纯 C float)。
//   注: 本文件不直接 include 平台库; 经 comp_filter.h 间接传递引用 BSP/bsp_dsp.h
//       (该库有纯 C 回退 BSP_DSP_ARCH=0, 可跨平台编译, 见 bsp_dsp.h)。
//   供 Devices/adc 的 DC / AC / Follower 子类共享 —— 一份信号处理实现, 多设备复用。
//
// 流水线 (每通道独立):
//   raw ──(去DC[可选])──> 滤波(EMA/一阶LPF, alpha) ──> 线性校准(k,b) ──> 工程量
//
// 语义对齐:
//   - 滤波 = TI MATH_EMAVG_F / 现 DC 采样器: out = α·x + (1−α)·out  (α = dt/τ)
//     复用 Components/dsp/comp_filter.h 的 MathEmavg (消化已有资产, 非再造)
//   - 校准 = 线性: value = k · filt + b   (与 adc_dc_sampler.c process_impl 等价)
//
// 使用方式 (Device 子类):
//   AdcSigChannel ch[8];                        // 每通道一组 (含 filter + calib + state)
//   adc_sig_channel_init(&ch[i], k, b, alpha);  // alpha<0 禁流水线; alpha=0 不滤波但校准
//   adc_sig_process(&ch[i], raw);               // 热路径, ISR 安全
//   float eng = adc_sig_process(&ch[i], raw);     // 返回工程量
//   float f  = adc_sig_filt(&ch[i]);               // 取滤波后值 (诊断/中继)
//
// 注: 三相重构 / RMS 等"域级"处理不属于本文件 (那是 AC 子类的三相聚合),
//     本文件只管"原始码 → 单通道工程量"的通用部分。

#ifndef COMP_ADC_SIG_H
#define COMP_ADC_SIG_H

#include <stdint.h>
#include "comp_math.h"        // M_PI / MATH_SQRT 等 (如需)
#include "comp_filter.h"      // MathEmavg — 复用 EMA 滤波器

#ifdef __cplusplus
extern "C" {
#endif

// ======== 单通道校准参数 POD (YAML 可注入) ========
typedef struct {
  float k;  // 线性增益 (工程量/ADC 码) — value = k·filt + b
  float b;  // 线性偏置 (工程量)
} AdcSigCalib;

// ======== 单通道信号处理上下文 (值包含, 多实例安全) ========
// 每通道一组; 不放在 Device 全局, 保证多通道/多设备隔离 + ISR 安全
typedef struct {
  AdcSigCalib calib;    // 校准参数 (k/b)
  MathEmavg   filter;   // EMA 滤波 (multiplier=alpha; <=0 → 不滤波, en 仍可校准)
  float       filt;     // 滤波后值 (诊断/链式中继用)
  uint8_t     en;       // 使能: 0=直通原值 (不滤波不校准), 1=正常流水线
} AdcSigChannel;

// ======== 通道初始化 ========
//   k, b: 线性校准 (value = k·filt + b)
//   alpha: EMA 滤波系数 [0,1]; 传 alpha<0 (如 -1f) → 禁用整个流水线 (en=0, 直通不校准);
//          传 alpha=0 → 不滤波但仍线性校准 (value = k·raw + b)
static inline void adc_sig_channel_init(AdcSigChannel *me, float k, float b, float alpha) {
  me->calib.k = k;
  me->calib.b = b;
  if (alpha < 0.0f) {
    me->en = 0u;                                  // 禁用整个流水线
    (void) math_emavg_init(&me->filter, 0.0f);
  } else {
    me->en = 1u;
    (void) math_emavg_init(&me->filter, alpha);   // alpha=0 → math_emavg_run 直通
  }
  me->filt = 0.0f;
}

// ======== 热路径单步: 原始码 → 滤波 + 校准 ========
// ISR 安全 (无 malloc、无浮点除法热点、无平台库)
static inline float adc_sig_process(AdcSigChannel *me, float raw) {
  float out;
  if (!me->en) {
    out = raw;                                    // 全直通: 原样返回
    me->filt = raw;
  } else if (me->filter.multiplier <= 0.0f) {
    out = me->calib.k * raw + me->calib.b;        // 无滤波: 跳过 EMA 直接校准
    me->filt = raw;
  } else {
    me->filt = math_emavg_run(&me->filter, raw);  // EMA 滤波 (复用 comp_filter.h)
    out = me->calib.k * me->filt + me->calib.b;   // 线性校准
  }
  return out;
}

// ======== 读取通道滤波后值 (诊断/链式中继) ========
static inline float adc_sig_filt(const AdcSigChannel *me) { return me->filt; }

// ======== 整体流水线: 批量处理 N 通道 ========
// 供 Device 在 process 里对整批通道统一跑一遍 (DC 的 1~8ch, AC 的 v/i 通道)
static inline void adc_sig_process_n(AdcSigChannel *ch, const float *raw,
                                     float *eng, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    eng[i] = adc_sig_process(&ch[i], raw[i]);
  }
}

#ifdef __cplusplus
}
#endif

#endif  // COMP_ADC_SIG_H
