// 激光笔外设 — OutputBase 子类, GPIO 开关实现
// 迁移自 Devices/gpo/gpo_laser.c, 引脚改走 BspGpioPin
// 安全优先: 初始化默认关闭, 避免意外点亮

#include "per_laser.h"
#include "container_of.h"

// 打开: 写高电平
static void laser_on_impl(OutputBase *base) {
  PerLaser *me = container_of(base, PerLaser, base);
  bsp_gpio_write(&me->pin, true);
}

// 关闭: 写低电平
static void laser_off_impl(OutputBase *base) {
  PerLaser *me = container_of(base, PerLaser, base);
  bsp_gpio_write(&me->pin, false);
}

// 翻转: BSP 原语翻转输出
static void laser_toggle_impl(OutputBase *base) {
  PerLaser *me = container_of(base, PerLaser, base);
  bsp_gpio_toggle(&me->pin);
}

// 激光笔虚表 — set 为 NULL (无电平调节)
static const OutputOps per_laser_ops = {
    .on = laser_on_impl,
    .off = laser_off_impl,
    .set = NULL,
    .toggle = laser_toggle_impl,
};

// 初始化: 配输出 → 默认关闭 (安全) → 绑 ops
void per_laser_init(PerLaser *me, const char *name, BspGpioPin pin) {
  output_base_init(&me->base, name);
  me->pin = pin;
  bsp_gpio_cfg_output(&me->pin);
  bsp_gpio_write(&me->pin, false);  // 默认关闭 —— 激光笔绝不能在初始化时意外点亮
  me->base.ops = &per_laser_ops;
}

// 反初始化: 强制关闭 → 清 ops
void per_laser_deinit(PerLaser *me) {
  bsp_gpio_write(&me->pin, false);
  output_base_deinit(&me->base);
}
