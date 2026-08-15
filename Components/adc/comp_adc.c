// ADC 基类实现 —— AdcBase 父类的构造/析构/分派
//
// 提供统一基类契约: 成员清零 (init/deinit)、ops 虚表分派 (start_dma/read_ch/process)
// 子类 (AdcDcSampler/AdcAcSampler/AdcFollower) 在 init 时绑定自身 static ops

#include "comp_adc.h"
#include <assert.h>
#include <stddef.h>

// 基类构造 —— 成员清零，raw/raw_cap/ops 由子类 init 时绑定
void adc_base_init(AdcBase *me) {
  me->ops     = NULL;
  me->raw     = NULL;
  me->raw_cap = 0;
  me->pos     = 0;
}

// 基类析构 —— 清除 ops 和缓冲区指针
void adc_base_deinit(AdcBase *me) {
  me->ops     = NULL;
  me->name    = 0;
  me->raw     = NULL;
  me->raw_cap = 0;
  me->pos     = 0;
}

// 分发函数: 断言 ops->start_dma 必须存在
void adc_start_dma(AdcBase *me) {
  assert(me->ops->start_dma);
  me->ops->start_dma(me);
}

// 分发函数: 断言 ops->read_ch 必须存在
uint16_t adc_read_ch(AdcBase *me, int i) {
  assert(me->ops->read_ch);
  return me->ops->read_ch(me, i);
}

// 分发函数: 断言 ops->process 必须存在
void adc_process(AdcBase *me) {
  assert(me->ops->process);
  me->ops->process(me);
}

// 分发函数: 触发传感器计数 (可选 — 仅循迹传感器有意义)
int16_t adc_get_sum2(AdcBase *me) {
  if (me->ops->get_sum2) {
    return me->ops->get_sum2(me);
  }
  return 0;
}

// 分发函数: 通道 i 二值化结果 (可选 — 仅循迹传感器有意义)
int16_t adc_get_ch_bin(AdcBase *me, int i) {
  if (me->ops->get_ch_bin) {
    return me->ops->get_ch_bin(me, i);
  }
  return 0;
}
