// 有源蜂鸣器外设 — OutputBase 子类, GPIO 开关
//
// 迁移: 原 Devices/gpo/gpo_beep.c (阶段2 peripheral 域收编)
// 改造: GPIO_TypeDef*/pin + active_low → BspGpioPin (HAL-free)
// 语义: 通电就叫, 断电就停 (无需 PWM), 典型用途: 按键反馈音 (100ms 短鸣)

#ifndef PER_BEEP_H
#define PER_BEEP_H

#include "comp_output.h"
#include "bsp_gpio.h"

typedef struct {
  OutputBase base;  // 基类 (第一成员)
  BspGpioPin pin;   // 输出引脚
} PerBeep;

// 初始化: 配输出 → 默认关闭 → 绑 ops
void per_beep_init(PerBeep *me, const char *name, BspGpioPin pin);

// 反初始化: 强制关闭 → 清 ops
void per_beep_deinit(PerBeep *me);

#endif  // PER_BEEP_H
