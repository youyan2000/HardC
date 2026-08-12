#ifndef DEV_GPO_FAN_H
#define DEV_GPO_FAN_H

// 风扇驱动 —— GpoBase 子类
// PWM 控制: on/off 启停 PWM, set 调节转速 (占空比)

#include "comp_gpo.h"
#include "stm32f1xx_hal.h"

// 子类结构体 —— base 必须是第一个成员
typedef struct {
  GpoBase            base;  // 基类 (必须为第一个成员)
  TIM_HandleTypeDef *htim;  // 定时器句柄
  uint32_t           ch;    // PWM 通道
  uint32_t           duty;  // 当前占空比 (决定转速)
} GpoFan;

void gpo_fan_init  (GpoFan *me, GpoName name,
                    TIM_HandleTypeDef *htim, uint32_t ch, uint32_t duty);
void gpo_fan_deinit(GpoFan *me);

#endif
