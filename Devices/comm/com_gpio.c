// GPIO 电平 + 中断类实现 —— CommBase 子类 (Devices/comm, 阶段1 重构)
//
// 纯转发层: 全部操作经 bsp_gpio.h 平台无关抽象, 零 HAL include.
// 无去抖/无 active-low 反转 — 上层语义 (按键去抖) 归 Module 层 HMI.

#include "com_gpio.h"
#include <stddef.h>

// -------- 构造 / 配置 --------

// 初始化: 契约身份 + 存引脚
void gpio_init(Gpio *me, const GpioConfig *cfg) {
  comm_base_init(&me->base, "gpio");
  me->pin = cfg->pin;
  me->irq_cb = NULL;
  me->irq_ctx = NULL;
}

// 重配: 换引脚 (不改变契约身份)
void gpio_set_config(Gpio *me, const GpioConfig *cfg) {
  me->pin = cfg->pin;
}

// -------- 电平操作 (转发 bsp_gpio) --------

void gpio_cfg_output(Gpio *me) {
  bsp_gpio_cfg_output(&me->pin);
}

void gpio_cfg_input(Gpio *me, BspGpioPull pull) {
  bsp_gpio_cfg_input(&me->pin, pull);
}

void gpio_write(Gpio *me, bool level) {
  bsp_gpio_write(&me->pin, level);
}

bool gpio_read(Gpio *me) {
  return bsp_gpio_read(&me->pin);
}

void gpio_toggle(Gpio *me) {
  bsp_gpio_toggle(&me->pin);
}

// -------- 中断 --------

// 使能边沿中断: 存回调/上下文 + 转发 bsp_gpio_enable_irq
ErrorCode gpio_enable_irq(Gpio *me, BspGpioEdge edge, bsp_gpio_irq_fn cb, void *ctx) {
  me->irq_cb = cb;
  me->irq_ctx = ctx;
  if (bsp_gpio_enable_irq(&me->pin, edge, cb, ctx) != 0) {
    return ERR_ARG;
  }
  return ERR_OK;
}

// 关闭边沿中断: 转发 + 清回调
void gpio_disable_irq(Gpio *me) {
  bsp_gpio_disable_irq(&me->pin);
  me->irq_cb = NULL;
  me->irq_ctx = NULL;
}
