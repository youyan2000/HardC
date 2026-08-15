// 有源蜂鸣器外设 — OutputBase 子类, GPIO 开关实现
// 迁移自 Devices/gpo/gpo_beep.c, 引脚改走 BspGpioPin
// 通电就叫, 断电就停 —— 无需 PWM

#include "per_beep.h"
#include "container_of.h"

// 打开: 写高电平 → 蜂鸣
static void beep_on_impl(OutputBase *base) {
  PerBeep *me = container_of(base, PerBeep, base);
  bsp_gpio_write(&me->pin, true);
}

// 关闭: 写低电平 → 静音
static void beep_off_impl(OutputBase *base) {
  PerBeep *me = container_of(base, PerBeep, base);
  bsp_gpio_write(&me->pin, false);
}

// 翻转: BSP 原语翻转输出
static void beep_toggle_impl(OutputBase *base) {
  PerBeep *me = container_of(base, PerBeep, base);
  bsp_gpio_toggle(&me->pin);
}

// 有源蜂鸣器虚表 — set 为 NULL (无音调调节)
static const OutputOps per_beep_ops = {
    .on = beep_on_impl,
    .off = beep_off_impl,
    .set = NULL,
    .toggle = beep_toggle_impl,
};

// 初始化: 配输出 → 默认关闭 → 绑 ops
void per_beep_init(PerBeep *me, const char *name, BspGpioPin pin) {
  output_base_init(&me->base, name);
  me->pin = pin;
  bsp_gpio_cfg_output(&me->pin);
  bsp_gpio_write(&me->pin, false);  // 默认关闭
  me->base.ops = &per_beep_ops;
}

// 反初始化: 强制关闭 → 清 ops
void per_beep_deinit(PerBeep *me) {
  bsp_gpio_write(&me->pin, false);
  output_base_deinit(&me->base);
}
