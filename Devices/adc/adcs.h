#ifndef ADCS_H
#define ADCS_H

// ADHardC 全局句柄 — 应用层唯一入口
// 使用方式: #include "adcs.h" → adc_read_ch(g_adc_follower, 0)
//
// 命名规则: g_adc_<用途> (全局句柄 g_ 前缀)

#include "comp_adc.h"

// 红外循迹传感器 (8ch, Follower 模式)
extern AdcBase *g_adc_follower;

// DC 采样器 (EMA + kx+b 校准)
extern AdcBase *g_adc_dc;

// AC 采样器 (三相 RMS)
extern AdcBase *g_adc_ac;

// 用户自定义 ADC 句柄 (0-3)
extern AdcBase *g_adc_user0;
extern AdcBase *g_adc_user1;
extern AdcBase *g_adc_user2;
extern AdcBase *g_adc_user3;

#endif
