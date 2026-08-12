// 有源蜂鸣器驱动 —— GpoBase 子类, GPIO 开关
// 通电就叫, 断电就停 —— 无需 PWM

#include "gpo_beep.h"
#include "container_of.h"

// 打开: 写有效电平 → 蜂鸣
static void beep_on_impl(GpoBase *base) {
  GpoBeep *me = container_of(base, GpoBeep, base);
  GPIO_PinState level = me->active_low ? GPIO_PIN_RESET : GPIO_PIN_SET;
  HAL_GPIO_WritePin(me->port, me->pin, level);
  me->is_on = true;
}

// 关闭: 写无效电平 → 静音
static void beep_off_impl(GpoBase *base) {
  GpoBeep *me = container_of(base, GpoBeep, base);
  GPIO_PinState level = me->active_low ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(me->port, me->pin, level);
  me->is_on = false;
}

// 翻转: 基于 is_on 状态
static void beep_toggle_impl(GpoBase *base) {
  GpoBeep *me = container_of(base, GpoBeep, base);
  if (me->is_on) {
    beep_off_impl(base);
  } else {
    beep_on_impl(base);
  }
}

// 有源蜂鸣器虚表 —— set 为 NULL, 无音调调节
static const GpoOps gpo_beep_ops = {
  .on     = beep_on_impl,
  .off    = beep_off_impl,
  .set    = NULL,
  .toggle = beep_toggle_impl,
};

// 初始化: 绑定 GPIO → 默认关闭
void gpo_beep_init(GpoBeep *me, GpoName name,
                   GPIO_TypeDef *port, uint16_t pin, bool active_low) {
  gpo_base_init(&me->base, name);
  me->port       = port;
  me->pin        = pin;
  me->active_low = active_low;
  me->is_on      = false;
  me->base.ops   = &gpo_beep_ops;

  // 默认关闭
  GPIO_PinState off_level = active_low ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(me->port, me->pin, off_level);
}

// 反初始化: 强制关闭 → 清除
void gpo_beep_deinit(GpoBeep *me) {
  GPIO_PinState off_level = me->active_low ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(me->port, me->pin, off_level);
  gpo_base_deinit(&me->base);
  me->port  = NULL;
  me->pin   = 0;
  me->is_on = false;
}
