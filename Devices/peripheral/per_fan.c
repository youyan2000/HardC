// 风扇外设 — OutputBase 子类, TIM PWM 实现
// 迁移自 Devices/gpo/gpo_fan.c, 基类改名 GpoBase → OutputBase
// on/off: 启停 PWM 输出, set: 调整占空比 (改变转速)

#include "per_fan.h"
#include "container_of.h"

// 打开: 启动 PWM + 恢复当前转速
static void fan_on_impl(OutputBase *base) {
  PerFan *me = container_of(base, PerFan, base);
  HAL_TIM_PWM_Start(me->htim, me->ch);
  __HAL_TIM_SET_COMPARE(me->htim, me->ch, me->duty);
}

// 关闭: 停止 PWM
static void fan_off_impl(OutputBase *base) {
  PerFan *me = container_of(base, PerFan, base);
  HAL_TIM_PWM_Stop(me->htim, me->ch);
}

// 翻转: 读比较值判断启停状态
static void fan_toggle_impl(OutputBase *base) {
  PerFan *me = container_of(base, PerFan, base);
  if (__HAL_TIM_GET_COMPARE(me->htim, me->ch) > 0) {
    HAL_TIM_PWM_Stop(me->htim, me->ch);
  } else {
    HAL_TIM_PWM_Start(me->htim, me->ch);
    __HAL_TIM_SET_COMPARE(me->htim, me->ch, me->duty);
  }
}

// 设置转速: 更新占空比并立即生效
static void fan_set_impl(OutputBase *base, uint32_t level) {
  PerFan *me = container_of(base, PerFan, base);
  me->duty = level;
  __HAL_TIM_SET_COMPARE(me->htim, me->ch, level);
}

// 风扇虚表 — 三个操作全部实现
static const OutputOps per_fan_ops = {
    .on = fan_on_impl,
    .off = fan_off_impl,
    .set = fan_set_impl,
    .toggle = fan_toggle_impl,
};

// 初始化: 绑 TIM 通道和默认转速 → 绑 ops
void per_fan_init(PerFan *me, const char *name, TIM_HandleTypeDef *htim, uint32_t ch, uint32_t duty) {
  output_base_init(&me->base, name);
  me->htim = htim;
  me->ch = ch;
  me->duty = duty;
  me->base.ops = &per_fan_ops;
}

// 反初始化: 停止 PWM → 清 ops
void per_fan_deinit(PerFan *me) {
  HAL_TIM_PWM_Stop(me->htim, me->ch);
  output_base_deinit(&me->base);
  me->htim = NULL;
  me->ch = 0;
  me->duty = 0;
}
