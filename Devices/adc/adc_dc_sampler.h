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
 *   - dev_sampler.h: Device_Volt_Sampler / Device_Current_Sampler
 *   - "adc计算得出的数据往往与真实数据之间存在误差，而且基本表现为线性误差"
 *     所以直接将 adc_sum 与真实数据使用一个线性方程转换:
 *     voltage_ / current_ = k * sum_ + b, 并使用一阶线性滤波平稳 adc 数值
 *
 * 与 AdcAcSampler 区别:
 *   - AdcDcSampler: 直流信号, 线性校准+滤波, 通道数可变 (1~8)
 *   - AdcAcSampler: 交流信号, 差分ADC+三相重构+RMS+VDC, 固定 7 通道
 */

#ifndef ADC_DC_SAMPLER_H
#define ADC_DC_SAMPLER_H

#include "comp_adc.h"
#include "bsp_adc.h"             // BspAdcHandle 不透明句柄 — 跨平台 (STM32/C2000)
#include "comp_double_buffer.h"  // 五原语之 PingPong: DMA→FAST 采样快照 (撕裂读消除)
#include "comp_io.h"             // 运行时契约: I/O 完成方式 (DMA 完成 = IO_ASYNC_FLAG)
#include <stdbool.h>

#define ADC_DC_MAX_CH 8  // 直流采样最大通道数

typedef struct {
  AdcBase base;                         // [首成员!] 基类
  uint16_t raw_buf[2 * ADC_DC_MAX_CH];  // 双倍缓冲: 对半切分为两个快照块 (PingPong), 见 init
  DoubleBuffer dbuf;                    // PingPong 双缓冲状态 (active/pending 块翻转)
  BspAdcHandle *hadc;                   // BSP ADC 句柄 (STM32: &hadc1, C2000: ADC 基址)
  BspAdcHandle *hdma;                   // BSP DMA 句柄 (STM32: &hdma_adc1, C2000: 触发源)
  IoCompletion completion;              // 完成契约: 发起时声明完成方式 (本设备固定 IO_ASYNC_FLAG)

  // 每通道校准参数
  // value[i] = k[i] * raw_f[i] + b[i]
  float k[ADC_DC_MAX_CH];      // 线性增益 (V/ADC 或 A/ADC)
  float b[ADC_DC_MAX_CH];      // 线性偏置 (V 或 A)
  float alpha[ADC_DC_MAX_CH];  // EMA 滤波系数 [0,1], 0=无滤波
  // alpha ≈ 2*PI*fc*Ts, 例: fc=10Hz, Ts=1ms → alpha≈0.063

  // 输出
  float value[ADC_DC_MAX_CH];  // 工程量 (V 或 A)
  float raw_f[ADC_DC_MAX_CH];  // 滤波后 ADC 值 (诊断用)

  uint8_t num_ch;  // 实际通道数 (1 ~ ADC_DC_MAX_CH)
} AdcDcSampler;

// === API =====================================================================

// 初始化直流采样器
// completion: 完成契约 — 本设备 DMA 完成→置 pending 标志 (IO_ASYNC_FLAG), 消费者轮询 fetch; 传其他值即契约违约
// hadc:   BSP ADC 句柄 (STM32: &hadc1, C2000: ADC0_BASE 等 driverlib 基址)
// hdma:   BSP DMA/触发句柄 (STM32: &hdma_adc1, C2000: NULL — 由 ePWM 触发)
// num_ch: 通道数 (1~8), 对应 ADC 扫描序列的 Rank1~RankN
// k:      线性增益数组 [num_ch] (传 NULL 则默认 k=1.0)
// b:      线性偏置数组 [num_ch] (传 NULL 则默认 b=0.0)
// alpha:  EMA 系数数组 [num_ch] (传 NULL 则默认 alpha=0, 即无滤波)
void adc_dc_sampler_init(AdcDcSampler *me, IoCompletion completion, BspAdcHandle *hadc, BspAdcHandle *hdma,
                         uint8_t num_ch, const float *k, const float *b, const float *alpha);

// 反初始化: 停止 DMA、清空 ops
void adc_dc_sampler_deinit(AdcDcSampler *me);

// ADC DMA 完成回调 — 生产侧 (PingPong 快照交接), 在 HAL_ADC_ConvCpltCallback 中转发
// DMA 刚写满非活动块 → 标 pending (IO_ASYNC_FLAG) + 重装到当前活动块 (fetch 切走后即下轮写目标)
// 契约: 生产者置标志 (IO_ASYNC_FLAG) — 与 init 声明不符即配置错误, 不可静默 (assert)
// FAST 侧 adc_dc_sampler_fetch() 看到 pending → 切快照 → 只碰活动块 (撕裂读消除)
// 安全前提: ADC 由控制定时器触发, 每控制周期恰好一次完成 (见 .c 实现注释)
void adc_dc_sampler_on_dma_complete(AdcDcSampler *me);

// ADC 快照获取 — FAST 上下文 (CTX_FAST) 每控制周期调用
// 契约: 消费者轮询标志 (IO_ASYNC_FLAG) — 与 init 声明不符即配置错误, 不可静默 (assert)
// 有 pending 快照则切到新快照, 然后只对活动块做 EMA 滤波 + 校准 (fetch/process 分离)
// DMA 同时写另一块 → 保护阈值读数不撕裂
void adc_dc_sampler_fetch(AdcDcSampler *me);

// 获取通道 ch 的工程量 (V 或 A)
float adc_dc_sampler_get_value(const AdcDcSampler *me, int ch);

// 获取通道 ch 的滤波后原始 ADC 值 (诊断用)
float adc_dc_sampler_get_raw(const AdcDcSampler *me, int ch);

#endif  // ADC_DC_SAMPLER_H
