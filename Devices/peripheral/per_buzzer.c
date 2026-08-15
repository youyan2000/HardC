// 无源蜂鸣器外设 — OutputBase 子类, TIM PWM 实现
// 迁移自 Devices/gpo/gpo_buzzer.c, 基类改名 GpoBase → OutputBase
// on/off: 启停 PWM 输出, set: 调整占空比 (改变音调)

#include "per_buzzer.h"
#include "container_of.h"

// 打开: 启动 PWM + 恢复当前占空比
static void buzzer_on_impl(OutputBase *base) {
  PerBuzzer *me = container_of(base, PerBuzzer, base);
  HAL_TIM_PWM_Start(me->htim, me->ch);
  __HAL_TIM_SET_COMPARE(me->htim, me->ch, me->duty);
}

// 关闭: 停止 PWM
static void buzzer_off_impl(OutputBase *base) {
  PerBuzzer *me = container_of(base, PerBuzzer, base);
  HAL_TIM_PWM_Stop(me->htim, me->ch);
}

// 翻转: 读比较值判断启停状态
static void buzzer_toggle_impl(OutputBase *base) {
  PerBuzzer *me = container_of(base, PerBuzzer, base);
  if (__HAL_TIM_GET_COMPARE(me->htim, me->ch) > 0) {
    HAL_TIM_PWM_Stop(me->htim, me->ch);
  } else {
    HAL_TIM_PWM_Start(me->htim, me->ch);
    __HAL_TIM_SET_COMPARE(me->htim, me->ch, me->duty);
  }
}

// 设置占空比: 更新并立即生效
static void buzzer_set_impl(OutputBase *base, uint32_t level) {
  PerBuzzer *me = container_of(base, PerBuzzer, base);
  me->duty = level;
  __HAL_TIM_SET_COMPARE(me->htim, me->ch, level);
}

// 无源蜂鸣器虚表 — 三个操作全部实现
static const OutputOps per_buzzer_ops = {
    .on = buzzer_on_impl,
    .off = buzzer_off_impl,
    .set = buzzer_set_impl,
    .toggle = buzzer_toggle_impl,
};

// 初始化: 绑 TIM 通道和默认占空比 → 绑 ops
void per_buzzer_init(PerBuzzer *me, const char *name, TIM_HandleTypeDef *htim, uint32_t ch, uint32_t duty) {
  output_base_init(&me->base, name);
  me->htim = htim;
  me->ch = ch;
  me->duty = duty;
  me->base.ops = &per_buzzer_ops;
}

// 反初始化: 停止 PWM → 清 ops
void per_buzzer_deinit(PerBuzzer *me) {
  HAL_TIM_PWM_Stop(me->htim, me->ch);
  output_base_deinit(&me->base);
  me->htim = NULL;
  me->ch = 0;
  me->duty = 0;
}
