/**
 * @file    adc_dc_sampler.c
 * @brief   通用直流采样器实现 —— AdcBase 子类
 *
 * 实现 AdcOps 虚函数表:
 *   .start_dma → start_dma_impl (启动 ADC DMA 循环扫描, N 通道)
 *   .read_ch   → read_ch_impl   (读取通道 i 原始值)
 *   .process   → process_impl   (EMA 滤波 + 线性校准)
 *
 * 核心公式 (每通道独立):
 *   raw_f[i] = alpha[i] * raw[i] + (1-alpha[i]) * raw_f[i]   (EMA 一阶低通)
 *   value[i] = k[i] * raw_f[i] + b[i]                         (线性校准)
 *
 * 参考:
 *   - WEILAI SuperCap dev_sampler.h: Device_Sampler_GetVoltage()
 *   - 2025A开源 项目 adc_sampler.cpp: 线性校准+滤波
 */

#include "adc_dc_sampler.h"
#include "container_of.h"
#include <string.h>  // memset
#include <assert.h>

// ====== ops 实现 (static — 封装) =============================================

// 启动 ADC DMA 循环扫描 — N 通道
static void start_dma_impl(AdcBase *base) {
  AdcDcSampler *me = container_of(base, AdcDcSampler, base);
  // ADC 由 TIMx_TRGO / ePWM 硬件触发, DMA 循环模式
  // 经 BSP 抽象分发 (STM32: HAL_ADC_Start_DMA; C2000: 触发源 + 缓冲注册)
  bsp_adc_start_dma(me->hadc, me->hdma, base->raw, me->num_ch);
}

// 读取通道 i 的原始 ADC 值
static uint16_t read_ch_impl(AdcBase *base, int i) {
  if (i < 0 || i >= 8)
    return 0;
  return base->raw[i];
}

// 直流采样处理: EMA 滤波 + 线性校准
// 在 ADC 转换完成 ISR 回调之后调用 (或定时器中调用)
static void process_impl(AdcBase *base) {
  AdcDcSampler *me = container_of(base, AdcDcSampler, base);

  for (int i = 0; i < me->num_ch; i++) {
    float raw = (float) base->raw[i];

    // EMA 一阶低通滤波 (等同 WEILAI LowPassFilter_Apply)
    if (me->alpha[i] > 0.0f) {
      me->raw_f[i] = me->alpha[i] * raw + (1.0f - me->alpha[i]) * me->raw_f[i];
    } else {
      me->raw_f[i] = raw;
    }

    // 线性校准: value = k * raw_f + b
    me->value[i] = me->k[i] * me->raw_f[i] + me->b[i];
  }
}

// ====== ADC 虚函数表 =========================================================

static const AdcOps dc_sampler_ops = {
    .start_dma = start_dma_impl,
    .read_ch = read_ch_impl,
    .process = process_impl,
    .get_sum2 = NULL,  // 直流采样无二值化概念
    .get_ch_bin = NULL,
};

// ====== 构造器 ===============================================================

void adc_dc_sampler_init(AdcDcSampler *me, BspAdcHandle *hadc, BspAdcHandle *hdma, uint8_t num_ch, const float *k,
                         const float *b, const float *alpha) {
  assert(num_ch >= 1 && num_ch <= 8);

  // 调用基类构造器
  adc_base_init(&me->base);
  me->base.name = AdcDcSamplerSensor;
  me->base.ops = &dc_sampler_ops;
  me->base.raw = me->raw_buf;        // 绑定子类 DMA 缓冲区
  me->base.raw_cap = ADC_DC_MAX_CH;  // 最多 8 通道

  // 绑定 HAL 句柄
  me->hadc = hadc;
  me->hdma = hdma;
  me->num_ch = num_ch;

  // 初始化每通道参数 (k/b/alpha 数组允许为 NULL, 表示使用默认值)
  for (int i = 0; i < num_ch; i++) {
    me->k[i] = k ? k[i] : 1.0f;              // 默认: 直接输出 ADC 原始值
    me->b[i] = b ? b[i] : 0.0f;              // 默认: 无偏置
    me->alpha[i] = alpha ? alpha[i] : 0.0f;  // 默认: 无滤波
    me->value[i] = 0.0f;
    me->raw_f[i] = 0.0f;
  }

  // 未使用通道清零
  for (int i = num_ch; i < 8; i++) {
    me->k[i] = 1.0f;
    me->b[i] = 0.0f;
    me->alpha[i] = 0.0f;
    me->value[i] = 0.0f;
    me->raw_f[i] = 0.0f;
  }
}

// ====== 数据获取 =============================================================

// 反初始化: 停止 DMA、清空 BSP 句柄、清空 ops
void adc_dc_sampler_deinit(AdcDcSampler *me) {
  if (me->hadc != NULL) {
    bsp_adc_stop_dma(me->hadc, me->hdma);
  }
  me->hadc = NULL;
  me->hdma = NULL;
  adc_base_deinit(&me->base);
}

// ADC 转换完成回调 — 在 HAL_ADC_ConvCpltCallback 中调用
// DMA 已将 N 通道数据写入 base.raw[], 执行滤波+校准
void adc_dc_sampler_fetch(AdcDcSampler *me) {
  // DMA 已自动将 ADC 数据写入 base.raw[]
  // 执行 EMA 滤波 + 线性校准
  adc_process(&me->base);
}

// 获取通道 ch 的工程量
float adc_dc_sampler_get_value(const AdcDcSampler *me, int ch) {
  if (ch < 0 || ch >= me->num_ch)
    return 0.0f;
  return me->value[ch];
}

// 获取通道 ch 的滤波后原始 ADC 值 (诊断用)
float adc_dc_sampler_get_raw(const AdcDcSampler *me, int ch) {
  if (ch < 0 || ch >= me->num_ch)
    return 0.0f;
  return me->raw_f[ch];
}
