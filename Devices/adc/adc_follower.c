// 红外循迹传感器驱动实现 —— AdcBase 子类
//
// 实现 AdcOps 虚函数表: start_dma / read_ch / process
// ADC1 + DMA1 通道 1, 8 路扫描 (PA0-PA7), TIM3 TRGO 触发, DMA 循环写 raw[8]

#include "adc_follower.h"
#include "container_of.h"
#include <stdio.h>

// -------- ops 实现 --------

static void dma_impl(AdcBase *base) {
  AdcFollower *me = container_of(base, AdcFollower, base);
  // 统一走 BSP 抽象 (STM32: HAL_ADC_Start_DMA; C2000: 触发源 + 缓冲注册) — 跨平台
  bsp_adc_start_dma(me->hadc, me->hdma, (uint16_t *)base->raw, 8);
}

// 读取通道 i 原始 ADC 值 (用于串口查询, FC 01 协议)
static uint16_t read_impl(AdcBase *base, int i) {
  return base->raw[i];
}

// 红外循迹处理: 二值化 + 独热码 + 位置偏差映射 (校准模式下追踪黑值)
static void process_impl(AdcBase *base) {
  AdcFollower *me = container_of(base, AdcFollower, base);

  // 校准模式: 持续追踪每路离白值最远的采样值
  if (me->cal_step == 2) {
    for (int i = 0; i < 8; i++) {
      int cur_diff = (int)base->raw[i] - (int)me->cal_white[i];
      if (cur_diff < 0) cur_diff = -cur_diff;
      int old_diff = (int)me->cal_black[i] - (int)me->cal_white[i];
      if (old_diff < 0) old_diff = -old_diff;
      if (cur_diff > old_diff) {
        me->cal_black[i] = base->raw[i];
      }
    }
  }

  me->sum  = 0;
  me->sum2 = 0;

  for (int i = 0; i < 8; i++) {
    int hit = (me->polarity[i] > 0) ? (base->raw[i] < me->threshold[i])
                                       : (base->raw[i] > me->threshold[i]);
    if (hit) {
      me->ch_bin[i] = 1;
      me->ch_val[i] = 1 << i;
      me->sum2++;
    } else {
      me->ch_bin[i] = 0;
      me->ch_val[i] = 0;
    }
    me->sum += me->ch_val[i];
  }

  // 独热码之和 → 位置偏差 (-7~+7, P2PD 非线性 PID 天然区分大小误差)
  switch (me->sum) {
    case 1:   base->pos = -7; break;  // 0x01: s0
    case 3:   base->pos = -6; break;  // 0x03: s0,1
    case 2:   base->pos = -5; break;  // 0x02: s1
    case 6:   base->pos = -4; break;  // 0x06: s1,2
    case 4:   base->pos = -3; break;  // 0x04: s2
    case 12:  base->pos = -2; break;  // 0x0C: s2,3
    case 8:   base->pos = -1; break;  // 0x08: s3
    case 24:  base->pos =  0; break;  // 0x18: s3,4 (中心)
    case 16:  base->pos =  1; break;  // 0x10: s4
    case 48:  base->pos =  2; break;  // 0x30: s4,5
    case 32:  base->pos =  3; break;  // 0x20: s5
    case 96:  base->pos =  4; break;  // 0x60: s5,6
    case 64:  base->pos =  5; break;  // 0x40: s6
    case 192: base->pos =  6; break;  // 0xC0: s6,7
    case 128: base->pos =  7; break;  // 0x80: s7
    default:  base->pos =  0; break;
  }
}

// 返回触发传感器计数
static int16_t get_sum2_impl(AdcBase *base) {
  AdcFollower *me = container_of(base, AdcFollower, base);
  return me->sum2;
}

// 返回通道 i 的二值化结果 (0/1)
static int16_t get_ch_bin_impl(AdcBase *base, int i) {
  AdcFollower *me = container_of(base, AdcFollower, base);
  if (i < 0 || i >= 8) return 0;
  return me->ch_bin[i];
}

static const AdcOps follower_ops = {
  .start_dma  = dma_impl,
  .read_ch    = read_impl,
  .process    = process_impl,
  .get_sum2   = get_sum2_impl,
  .get_ch_bin = get_ch_bin_impl,
};

// -------- 构造 / 析构 --------

void adc_follower_init(AdcFollower *me, BspAdcHandle *hadc, BspAdcHandle *hdma,
                       const int16_t *threshold) {
  adc_base_init(&me->base);
  me->base.name    = AdcFollowerSensor;
  me->base.ops     = &follower_ops;
  me->base.raw     = me->raw_buf;  // 绑定子类 DMA 缓冲区
  me->base.raw_cap = 8;              // 8 通道红外
  me->hadc         = hadc;
  me->hdma         = hdma;
  for (int i = 0; i < 8; i++) {
    me->threshold[i] = threshold[i];
    me->ch_bin[i]    = 0;
    me->ch_val[i]    = 0;
    me->cal_white[i] = 0;
    me->cal_black[i] = 0;
    me->polarity[i]  = 1;
  }
  me->sum      = 0;
  me->sum2     = 0;
  me->cal_step = 0;
  me->cal_send = 0;
  me->cal_pending = 0;
  me->sns_pending = 0;
  me->sns_send    = 0;
}

// 反初始化: 停止 DMA、清空 BSP 句柄、清空 ops 和基类字段 (对齐 DC/AC deinit)
void adc_follower_deinit(AdcFollower *me) {
  if (me->hadc != NULL) {
    bsp_adc_stop_dma(me->hadc, me->hdma);
  }
  me->hadc = NULL;
  me->hdma = NULL;
  adc_base_deinit(&me->base);
}

// -------- 校准 (三步协议: EE 01 / EE 02 / EE 03) --------

void adc_follower_cal_white(AdcFollower *me) {
  for (int i = 0; i < 8; i++) {
    me->cal_white[i] = me->base.raw[i];
    me->cal_black[i] = me->base.raw[i];  // 初始化为白值
  }
  me->cal_step = 1;
  me->cal_send = 1;  // 主循环发送白值
}

void adc_follower_cal_black_start(AdcFollower *me) {
  if (me->cal_step != 1) return;
  me->cal_step = 2;
}

// 校准完成: 计算 8 路阈值 = 白值×0.4 + 黑值×0.6 (偏黑 10%), 判断极性, 格式化结果到 buf
int adc_follower_cal_finish(AdcFollower *me, char *buf, int max_len) {
  if (me->cal_step != 2) return 0;

  for (int i = 0; i < 8; i++) {
    me->threshold[i] = (me->cal_white[i] * 4 + me->cal_black[i] * 6) / 10;  // 阈值偏黑10%
    me->polarity[i]  = (me->cal_white[i] > me->cal_black[i]) ? 1 : -1;
  }
  me->cal_step = 0;
  me->cal_send = 0;

  return snprintf(buf, max_len,
    "TH:%d,%d,%d,%d,%d,%d,%d,%d\r\n"
    "W:%d,%d,%d,%d,%d,%d,%d,%d\r\n"
    "B:%d,%d,%d,%d,%d,%d,%d,%d\r\n",
    me->threshold[0], me->threshold[1], me->threshold[2], me->threshold[3],
    me->threshold[4], me->threshold[5], me->threshold[6], me->threshold[7],
    me->cal_white[0], me->cal_white[1], me->cal_white[2], me->cal_white[3],
    me->cal_white[4], me->cal_white[5], me->cal_white[6], me->cal_white[7],
    me->cal_black[0], me->cal_black[1], me->cal_black[2], me->cal_black[3],
    me->cal_black[4], me->cal_black[5], me->cal_black[6], me->cal_black[7]);
}

// -------- ISR 串口字节分发 (封装 calibration / sensor 协议, 避免 App 层直接读子类字段) --------

uint8_t adc_follower_handle_cal_byte(AdcFollower *me, uint8_t byte) {
  if (me->cal_pending) {
    me->cal_pending = 0;
    if (byte == 0x01) { adc_follower_cal_white(me); }
    if (byte == 0x02) { adc_follower_cal_black_start(me); }
    if (byte == 0x03) { me->cal_send = 2; }
    return 1;
  }
  if (byte == 0xee) {
    me->cal_pending = 1;
    return 1;
  }
  return 0;
}

uint8_t adc_follower_handle_sns_byte(AdcFollower *me, uint8_t byte) {
  if (me->sns_pending) {
    me->sns_pending = 0;
    if (byte == 0x01) { me->sns_send = 1; }
    if (byte == 0x02) { me->sns_send = 2; }
    if (byte == 0x03) { me->sns_send = 3; }
    return 1;
  }
  if (byte == 0xfc) {
    me->sns_pending = 1;
    return 1;
  }
  return 0;
}
