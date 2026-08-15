// 风扇外设 — OutputBase 子类, TIM PWM
//
// 迁移: 原 Devices/gpo/gpo_fan.c (阶段2 peripheral 域收编)
// 已知偏离: 保留 TIM_HandleTypeDef* (bsp_pwm 后端未覆盖 F1 通用 TIM PWM), 记录待收编
// 语义: on/off 启停 PWM 输出, set 调占空比 (转速)

#ifndef PER_FAN_H
#define PER_FAN_H

#include "comp_output.h"
#include "bsp_stm32_hal.h"

typedef struct {
  OutputBase base;          // 基类 (第一成员)
  TIM_HandleTypeDef *htim;  // 定时器句柄
  uint32_t ch;              // PWM 通道
  uint32_t duty;            // 当前占空比 (决定转速)
} PerFan;

// 初始化: 绑 TIM 通道 + 默认占空比 → 绑 ops
void per_fan_init(PerFan *me, const char *name, TIM_HandleTypeDef *htim, uint32_t ch, uint32_t duty);

// 反初始化: 停止 PWM → 清 ops
void per_fan_deinit(PerFan *me);

#endif  // PER_FAN_H
