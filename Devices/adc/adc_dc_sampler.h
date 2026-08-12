/**
 * @file    adc_dc_sampler.h
 * @brief   通用直流采样器 —— 继承 AdcBase，N 通道线性校准 + 一阶低通滤波
 *
 * 继承关系:
 *   AdcBase  <—  AdcDcSampler (本文件)
 *
 * 适用场景:
 *   - 单相直流采样: 电池电压、温度传感器 (NTC)、电位器
 *   - 多相直流采样: 多路电压/电流监测点
 *
 * 数据处理 (process):
 *   1. EMA 一阶低通滤波: raw_f = alpha * raw + (1-alpha) * raw_f
 *   2. 线性校准:          value = k * raw_f + b
 *
 * 参考:
 *   - WEILAI SuperCap dev_sampler.h: Device_Volt_Sampler / Device_Current_Sampler
 *   - "adc计算得出的数据往往与真实数据之间存在误差，而且基本表现为线性误差"
 *     所以直接将 adc_sum 与真实数据使用一个线性方程转换:
 *     voltage_ / current_ = k * sum_ + b, 并使用一阶线性滤波平稳 adc 数值
 *     —— ENTERPRIZE_RM2024-SuperCap-开源报告
 *
 * 与 AdcAcSampler 区别:
 *   - AdcDcSampler: 直流信号, 线性校准+滤波, 通道数可变 (1~8)
 *   - AdcAcSampler: 交流信号, 差分ADC+三相重构+RMS+VDC, 固定 7 通道
 */

#ifndef ADC_DC_SAMPLER_H
#define ADC_DC_SAMPLER_H

#include "comp_adc.h"
#include "stm32f1xx_hal.h"   // 同时兼容 F1/F4/G4 系列
#include <stdbool.h>

#define ADC_DC_MAX_CH 8  // 直流采样最大通道数

typedef struct {
  AdcBase            base;          // [首成员!] 基类
  uint16_t           raw_buf[ADC_DC_MAX_CH]; // [基类绑定] DMA 缓冲区
  ADC_HandleTypeDef  *hadc;         // HAL ADC 句柄
  DMA_HandleTypeDef  *hdma;         // HAL DMA 句柄

  // 每通道校准参数
  // value[i] = k[i] * raw_f[i] + b[i]
  float  k[ADC_DC_MAX_CH];          // 线性增益 (V/ADC 或 A/ADC)
  float  b[ADC_DC_MAX_CH];          // 线性偏置 (V 或 A)
  float  alpha[ADC_DC_MAX_CH];      // EMA 滤波系数 [0,1], 0=无滤波
  // alpha ≈ 2*PI*fc*Ts, 例: fc=10Hz, Ts=1ms → alpha≈0.063

  // 输出
  float  value[ADC_DC_MAX_CH];      // 工程量 (V 或 A)
  float  raw_f[ADC_DC_MAX_CH];      // 滤波后 ADC 值 (诊断用)

  uint8_t num_ch;                   // 实际通道数 (1 ~ ADC_DC_MAX_CH)
} AdcDcSampler;

// === API =====================================================================

// 初始化直流采样器
// hadc:   CubeMX ADC 句柄
// hdma:   CubeMX DMA 句柄
// num_ch: 通道数 (1~8), 对应 CubeMX ADC 扫描序列的 Rank1~RankN
// k:      线性增益数组 [num_ch] (传 NULL 则默认 k=1.0)
// b:      线性偏置数组 [num_ch] (传 NULL 则默认 b=0.0)
// alpha:  EMA 系数数组 [num_ch] (传 NULL 则默认 alpha=0, 即无滤波)
void adc_dc_sampler_init(AdcDcSampler *me, ADC_HandleTypeDef *hadc,
                         DMA_HandleTypeDef *hdma, uint8_t num_ch,
                         const float *k, const float *b, const float *alpha);

// 反初始化: 停止 DMA、清空 ops
void adc_dc_sampler_deinit(AdcDcSampler *me);

// ADC 转换完成回调 — 在 HAL_ADC_ConvCpltCallback 中调用
// DMA 已将数据写入 base.raw[], 执行滤波+校准
void adc_dc_sampler_fetch(AdcDcSampler *me);

// 获取通道 ch 的工程量 (V 或 A)
float adc_dc_sampler_get_value(const AdcDcSampler *me, int ch);

// 获取通道 ch 的滤波后原始 ADC 值 (诊断用)
float adc_dc_sampler_get_raw(const AdcDcSampler *me, int ch);

#endif  // ADC_DC_SAMPLER_H
