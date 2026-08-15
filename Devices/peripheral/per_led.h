// LED 外设 — OutputBase 子类, GPIO 开关
//
// 迁移: 原 Devices/gpo/gpo_led.c (阶段2 peripheral 域收编)
// 改造: GPIO_TypeDef*/pin + active_low → BspGpioPin (HAL-free, bsp_gpio 无 active-low 反转)
// 简化: 原双模 (GPIO/PWM 调光) 只保留 GPIO 开关, PWM 调光能力并入 per_buzzer/per_fan 形态

#ifndef PER_LED_H
#define PER_LED_H

#include "comp_output.h"
#include "bsp_gpio.h"

typedef struct {
  OutputBase base;  // 基类 (第一成员)
  BspGpioPin pin;   // 输出引脚
} PerLed;

// 初始化: 配输出 → 默认关闭 → 绑 ops
void per_led_init(PerLed *me, const char *name, BspGpioPin pin);

// 反初始化: 关闭 → 清 ops
void per_led_deinit(PerLed *me);

#endif  // PER_LED_H
