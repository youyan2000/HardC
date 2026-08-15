// LED 外设 — OutputBase 子类, GPIO 开关实现
// 迁移自 Devices/gpo/gpo_led.c, 引脚改走 BspGpioPin

#include "per_led.h"
#include "container_of.h"

// 打开: 写高电平
static void led_on_impl(OutputBase *base) {
  PerLed *me = container_of(base, PerLed, base);
  bsp_gpio_write(&me->pin, true);
}

// 关闭: 写低电平
static void led_off_impl(OutputBase *base) {
  PerLed *me = container_of(base, PerLed, base);
  bsp_gpio_write(&me->pin, false);
}

// 翻转: BSP 原语翻转输出
static void led_toggle_impl(OutputBase *base) {
  PerLed *me = container_of(base, PerLed, base);
  bsp_gpio_toggle(&me->pin);
}

// LED 虚表 — set 为 NULL (纯开关)
static const OutputOps per_led_ops = {
    .on = led_on_impl,
    .off = led_off_impl,
    .set = NULL,
    .toggle = led_toggle_impl,
};

// 初始化: 配输出 → 默认关闭 → 绑 ops
void per_led_init(PerLed *me, const char *name, BspGpioPin pin) {
  output_base_init(&me->base, name);
  me->pin = pin;
  bsp_gpio_cfg_output(&me->pin);
  bsp_gpio_write(&me->pin, false);  // 默认关闭
  me->base.ops = &per_led_ops;
}

// 反初始化: 关闭 → 清 ops
void per_led_deinit(PerLed *me) {
  bsp_gpio_write(&me->pin, false);
  output_base_deinit(&me->base);
}
