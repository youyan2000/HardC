// 激光笔驱动 —— GpoBase 子类, GPIO 开关
// 安全优先: 初始化默认关闭, 避免意外点亮

#include "gpo_laser.h"
#include "container_of.h"

// 打开: 写有效电平
static void laser_on_impl(GpoBase *base) {
  GpoLaser *me = container_of(base, GpoLaser, base);
  GPIO_PinState level = me->active_low ? GPIO_PIN_RESET : GPIO_PIN_SET;
  HAL_GPIO_WritePin(me->port, me->pin, level);
  me->is_on = true;
}

// 关闭: 写无效电平
static void laser_off_impl(GpoBase *base) {
  GpoLaser *me = container_of(base, GpoLaser, base);
  GPIO_PinState level = me->active_low ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(me->port, me->pin, level);
  me->is_on = false;
}

// 翻转: 基于 is_on 状态
static void laser_toggle_impl(GpoBase *base) {
  GpoLaser *me = container_of(base, GpoLaser, base);
  if (me->is_on) {
    laser_off_impl(base);
  } else {
    laser_on_impl(base);
  }
}

// 激光笔虚表 —— set 为 NULL, 无电压调节
static const GpoOps gpo_laser_ops = {
  .on     = laser_on_impl,
  .off    = laser_off_impl,
  .set    = NULL,
  .toggle = laser_toggle_impl,
};

// 初始化: 绑定 GPIO → 默认关闭 (安全)
void gpo_laser_init(GpoLaser *me, GpoName name,
                    GPIO_TypeDef *port, uint16_t pin, bool active_low) {
  gpo_base_init(&me->base, name);
  me->port       = port;
  me->pin        = pin;
  me->active_low = active_low;
  me->is_on      = false;
  me->base.ops   = &gpo_laser_ops;

  // 默认关闭 —— 激光笔绝不能在初始化时意外点亮
  GPIO_PinState off_level = active_low ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(me->port, me->pin, off_level);
}

// 反初始化: 强制关闭 → 清除
void gpo_laser_deinit(GpoLaser *me) {
  GPIO_PinState off_level = me->active_low ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(me->port, me->pin, off_level);
  gpo_base_deinit(&me->base);
  me->port  = NULL;
  me->pin   = 0;
  me->is_on = false;
}
