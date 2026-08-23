// 通用直流采样器实现 —— AdcBase 子类
//
// 实现 AdcOps 虚函数表:
//   .start_dma → start_dma_impl (启动 ADC DMA 循环扫描, N 通道)
//   .read_ch   → read_ch_impl   (读取通道 i 原始值)
//   .process   → process_impl   (EMA 滤波 + 线性校准)
//
// 核心公式 (每通道独立):
//   raw_f[i] = alpha[i] * raw[i] + (1-alpha[i]) * raw_f[i]   (EMA 一阶低通)
//   value[i] = k[i] * raw_f[i] + b[i]                         (线性校准)
//
// 参考:
//   - dev_sampler.h: Device_Sampler_GetVoltage()
//   - adc_sampler.cpp: 线性校准+滤波

#include "adc_dc_sampler.h"
#include "container_of.h"
#include <string.h>  // memset
#include <assert.h>

// ====== ops 实现 (static — 封装) =============================================

// 启动 ADC DMA 循环扫描 — N 通道
// PingPong: 从非活动块起步 (invariant: DMA 总写非活动块, FAST 只读活动块). init 后 active=0 → 写 buf[1]
static void start_dma_impl(AdcBase *base) {
  AdcDcSampler *me = container_of(base, AdcDcSampler, base);
  // ADC 由 TIMx_TRGO / ePWM 硬件触发, DMA 循环模式
  // 经 BSP 抽象分发 (STM32: HAL_ADC_Start_DMA; C2000: 触发源 + 缓冲注册)
  bsp_adc_start_dma(me->hadc, me->hdma, (uint16_t *) double_buffer_pending(&me->dbuf), me->num_ch);
}

// 读取通道 i 的原始 ADC 值
static uint16_t read_ch_impl(AdcBase *base, int i) {
  if (i < 0 || i >= ADC_DC_MAX_CH)
    return 0;
  return base->raw[i];
}

// 直流采样处理: 共享信号处理 (comp_adc_sig) — EMA 滤波 + 线性校准
// 每通道 adc_sig_process(sig[i], raw): filt = EMA(alpha, raw), value = k·filt + b
// 与原手写 process (alpha>0: raw_f=α·raw+(1−α)·raw_f; 否则 raw_f=raw; value=k·raw_f+b) 数值等价
static void process_impl(AdcBase *base) {
  AdcDcSampler *me = container_of(base, AdcDcSampler, base);

  for (int i = 0; i < me->num_ch; i++) {
    float raw = (float) base->raw[i];
    me->value[i] = adc_sig_process(&me->sig[i], raw);  // 过滤 + 校准 → 工程量
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

void adc_dc_sampler_init(AdcDcSampler *me, IoCompletion completion, BspAdcHandle *hadc, BspAdcHandle *hdma,
                         uint8_t num_ch, const float *k, const float *b, const float *alpha) {
  assert(num_ch >= 1 && num_ch <= 8);

  // 调用基类构造器
  adc_base_init(&me->base);
  me->base.name = AdcDcSamplerSensor;
  me->base.ops = &dc_sampler_ops;

  // PingPong 双缓冲: 双倍 raw_buf 对半切分, 两块各 num_ch 个 16bit 采样
  // base.raw 绑活动块 (FAST 读); DMA 写非活动块, 完成 ISR 标 pending 后由 fetch 切换
  double_buffer_init(&me->dbuf, me->raw_buf, (uint16_t) (2u * num_ch * sizeof(uint16_t)));
  me->base.raw = (uint16_t *) double_buffer_active(&me->dbuf);
  me->base.raw_cap = ADC_DC_MAX_CH;  // 最多 8 通道

  // 绑定 HAL 句柄
  me->hadc = hadc;
  me->hdma = hdma;
  me->num_ch = num_ch;

  // 完成契约: 本设备 DMA 完成→置标志, 消费侧轮询 — 只支持 IO_ASYNC_FLAG (声明其他值 = 违约)
  me->completion = completion;
  assert(me->completion == IO_ASYNC_FLAG);

  // 初始化每通道共享信号处理流水线 (k/b/alpha 数组允许为 NULL, 表示默认值)
  for (int i = 0; i < num_ch; i++) {
    float kk = k ? k[i] : 1.0f;              // 默认: 直接输出 ADC 原始值
    float bb = b ? b[i] : 0.0f;              // 默认: 无偏置
    float aa = alpha ? alpha[i] : 0.0f;      // 默认: 无滤波
    adc_sig_channel_init(&me->sig[i], kk, bb, aa);
    me->value[i] = 0.0f;
  }

  // 未使用通道清零 (禁流水线直通)
  for (int i = num_ch; i < 8; i++) {
    adc_sig_channel_init(&me->sig[i], 1.0f, 0.0f, -1.0f);  // en=0, 直通不校准
    me->value[i] = 0.0f;
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

// ADC DMA 完成 ISR — 生产侧交接 (PingPong, 见 agent.md §1.2 / comp_double_buffer.h)
// 不变量: DMA 总被重装到"完成时活动块"; 该块被 fetch 切走后即非活动块 → 完成时写满的正是非活动块
//   ① 标 pending (IO_ASYNC_FLAG) 供 FAST 消费; ② 重装到完成时活动块 (下轮写目标)
// 安全前提: ADC 由控制定时器触发 (单转换源, 每控制周期恰好一次完成), 重装生效在下一次触发,
//   FAST 的 fetch 在周期首部切快照 → DMA 从不写 FAST 正在读的活动块 (撕裂读消除)
// 违反前提 (触发率 > 控制率, 一周期多次完成) → pending/活动块映射错乱, 属配置错误不可静默
void adc_dc_sampler_on_dma_complete(AdcDcSampler *me) {
  // 契约: 生产者置标志 (IO_ASYNC_FLAG) — init 声明不符即配置错误不可静默
  assert(me->completion == IO_ASYNC_FLAG);
  // A4 原子性: 重装目标必须在 enable_pending 之前捕获. 完成时活动块 = 下轮 DMA 写目标
  // (FAST fetch 切走后它即非活动块). 若在 enable 之后才求值, STM32 上 FAST 可抢占本 ISR
  // 先切快照 → active 翻转, 重装就会落到刚写满/FAST 正在读的块 → 撕裂 (C2000 PIE 不嵌套无此窗口)
  uint8_t *next = double_buffer_active(&me->dbuf);
  double_buffer_set_pending_len(&me->dbuf, (uint16_t) (2u * me->num_ch));  // 16bit × N 通道
  double_buffer_enable_pending(&me->dbuf);                                 // 快照就绪
  bsp_adc_restart_dma(me->hadc, me->hdma, (uint16_t *) next, me->num_ch);
}

// FAST 上下文: 切快照 + 只碰活动块 (fetch/process 分离)
// 有 pending (DMA 完成 ISR 已标) → 切到新快照; 无新快照 → 保持上一快照
void adc_dc_sampler_fetch(AdcDcSampler *me) {
  // 契约: 消费者轮询标志 (IO_ASYNC_FLAG) — init 声明不符即配置错误不可静默
  assert(me->completion == IO_ASYNC_FLAG);
  if (double_buffer_has_pending(&me->dbuf)) {
    double_buffer_switch(&me->dbuf);
    me->base.raw = (uint16_t *) double_buffer_active(&me->dbuf);
  }
  // EMA 滤波 + 线性校准 — 只读活动块, DMA 同时写另一块, 无撕裂读
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
  return adc_sig_filt(&me->sig[ch]);
}
