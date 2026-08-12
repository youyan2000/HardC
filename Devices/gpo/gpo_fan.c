// 风扇驱动 —— GpoBase 子类, TIM PWM 控制
// on/off: 启停 PWM 输出, set: 调整占空比 (改变转速)

#include "gpo_fan.h"
#include "container_of.h"

// 打开: 启动 PWM + 恢复当前转速
static void fan_on_impl(GpoBase *base) {
  GpoFan *me = container_of(base, GpoFan, base);
  HAL_TIM_PWM_Start(me->htim, me->ch);
  __HAL_TIM_SET_COMPARE(me->htim, me->ch, me->duty);
}

// 关闭: 停止 PWM
static void fan_off_impl(GpoBase *base) {
  GpoFan *me = container_of(base, GpoFan, base);
  HAL_TIM_PWM_Stop(me->htim, me->ch);
}

// 翻转: 读比较值判断启停状态
static void fan_toggle_impl(GpoBase *base) {
  GpoFan *me = container_of(base, GpoFan, base);
  if (__HAL_TIM_GET_COMPARE(me->htim, me->ch) > 0) {
    HAL_TIM_PWM_Stop(me->htim, me->ch);
  } else {
    HAL_TIM_PWM_Start(me->htim, me->ch);
    __HAL_TIM_SET_COMPARE(me->htim, me->ch, me->duty);
  }
}

// 设置转速: 更新占空比并立即生效
static void fan_set_impl(GpoBase *base, uint32_t duty) {
  GpoFan *me = container_of(base, GpoFan, base);
  me->duty = duty;
  __HAL_TIM_SET_COMPARE(me->htim, me->ch, duty);
}

// 风扇虚表 —— 三个操作全部实现
static const GpoOps gpo_fan_ops = {
  .on     = fan_on_impl,
  .off    = fan_off_impl,
  .set    = fan_set_impl,
  .toggle = fan_toggle_impl,
};

// 初始化: 绑定 TIM 通道和默认转速 → 注册 ops
void gpo_fan_init(GpoFan *me, GpoName name,
                  TIM_HandleTypeDef *htim, uint32_t ch, uint32_t duty) {
  gpo_base_init(&me->base, name);
  me->htim      = htim;
  me->ch        = ch;
  me->duty      = duty;
  me->base.ops  = &gpo_fan_ops;
}

// 反初始化: 停止 PWM → 清除
void gpo_fan_deinit(GpoFan *me) {
  HAL_TIM_PWM_Stop(me->htim, me->ch);
  gpo_base_deinit(&me->base);
  me->htim = NULL;
  me->ch   = 0;
  me->duty = 0;
}
