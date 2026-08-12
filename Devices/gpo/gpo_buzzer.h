#ifndef DEV_GPO_BUZZER_H
#define DEV_GPO_BUZZER_H

// 无源蜂鸣器驱动 —— GpoBase 子类
// PWM 控制: on/off 启停 PWM, set 调频率 (音调)

#include "comp_gpo.h"
#include "stm32f1xx_hal.h"

// 子类结构体 —— base 必须是第一个成员
typedef struct {
  GpoBase            base;  // 基类 (必须为第一个成员)
  TIM_HandleTypeDef *htim;  // 定时器句柄
  uint32_t           ch;    // PWM 通道
  uint32_t           duty;  // 当前占空比 (决定频率/音调)
} GpoBuzzer;

void gpo_buzzer_init  (GpoBuzzer *me, GpoName name,
                       TIM_HandleTypeDef *htim, uint32_t ch, uint32_t duty);
void gpo_buzzer_deinit(GpoBuzzer *me);

#endif
