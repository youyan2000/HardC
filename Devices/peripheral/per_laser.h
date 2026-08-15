#ifndef PER_LASER_H
#define PER_LASER_H

// 激光笔外设 — OutputBase 子类, GPIO 开关
//
// 迁移: 原 Devices/gpo/gpo_laser.c (阶段2 peripheral 域收编)
// 改造: GPIO_TypeDef*/pin + active_low → BspGpioPin (HAL-free)
// 安全优先: 初始化默认关闭, 绝不允许意外点亮

#include "comp_output.h"
#include "bsp_gpio.h"

typedef struct {
  OutputBase base;  // 基类 (第一成员)
  BspGpioPin pin;   // 输出引脚
} PerLaser;

// 初始化: 配输出 → 默认关闭 (安全) → 绑 ops
void per_laser_init(PerLaser *me, const char *name, BspGpioPin pin);

// 反初始化: 强制关闭 → 清 ops
void per_laser_deinit(PerLaser *me);

#endif  // PER_LASER_H
