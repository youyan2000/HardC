// GPIO 电平 + 中断类 —— CommBase 子类 (Devices/comm, 阶段1 重构)
//
// 语义: 电平读写 + 边沿中断. 无去抖、无 active-low 反转 (按键去抖归 Module 层 HMI,
//   删除 com_key). 完全零 HAL — 全部转发 bsp_gpio.h 平台无关抽象.
//
// 数据面 (引脚 + 中断回调) 在子类结构体, 不进 CommBase 虚表.

#ifndef COM_GPIO_H
#define COM_GPIO_H

#include "comp_comm.h"
#include "comp_io.h"
#include "comp_error_code.h"
#include "bsp_gpio.h"
#include <stdbool.h>

// 配置 POD
typedef struct {
  BspGpioPin pin;
} GpioConfig;

// GPIO 类 — 电平读写 + 中断 (无去抖, 无 active-low 反转, 留上层 HMI)
typedef struct {
  CommBase base;           // 基类 (必须第一成员)
  BspGpioPin pin;          // 引脚 (bsp_gpio 管理)
  bsp_gpio_irq_fn irq_cb;  // 中断回调
  void *irq_ctx;           // 中断回调上下文
} Gpio;

void gpio_init(Gpio *me, const GpioConfig *cfg);
void gpio_set_config(Gpio *me, const GpioConfig *cfg);
void gpio_cfg_output(Gpio *me);
void gpio_cfg_input(Gpio *me, BspGpioPull pull);
void gpio_write(Gpio *me, bool level);
bool gpio_read(Gpio *me);
void gpio_toggle(Gpio *me);

// 使能边沿中断 (0=成功), 回调上下文保存在实例内
ErrorCode gpio_enable_irq(Gpio *me, BspGpioEdge edge, bsp_gpio_irq_fn cb, void *ctx);
void gpio_disable_irq(Gpio *me);

#endif  // COM_GPIO_H
