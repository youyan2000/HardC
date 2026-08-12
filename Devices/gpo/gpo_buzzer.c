// 无源蜂鸣器驱动 —— GpoBase 子类, TIM PWM 控制
// on/off: 启停 PWM 输出, set: 调整频率 (改变音调)

#include "gpo_buzzer.h"
#include "container_of.h"

// 打开: 启动 PWM + 恢复当前频率
static void buzzer_on_impl(GpoBase *base) {
  GpoBuzzer *me = container_of(base, GpoBuzzer, base);
  HAL_TIM_PWM_Start(me->htim, me->ch);
  __HAL_TIM_SET_COMPARE(me->htim, me->ch, me->duty);
}

// 关闭: 停止 PWM
static void buzzer_off_impl(GpoBase *base) {
  GpoBuzzer *me = container_of(base, GpoBuzzer, base);
  HAL_TIM_PWM_Stop(me->htim, me->ch);
}

// 翻转: 读比较值判断启停状态
static void buzzer_toggle_impl(GpoBase *base) {
  GpoBuzzer *me = container_of(base, GpoBuzzer, base);
  if (__HAL_TIM_GET_COMPARE(me->htim, me->ch) > 0) {
    HAL_TIM_PWM_Stop(me->htim, me->ch);
  } else {
    HAL_TIM_PWM_Start(me->htim, me->ch);
    __HAL_TIM_SET_COMPARE(me->htim, me->ch, me->duty);
  }
}

// 设置频率: 更新占空比并立即生效
static void buzzer_set_impl(GpoBase *base, uint32_t duty) {
  GpoBuzzer *me = container_of(base, GpoBuzzer, base);
  me->duty = duty;
  __HAL_TIM_SET_COMPARE(me->htim, me->ch, duty);
}

// 无源蜂鸣器虚表 —— 三个操作全部实现
static const GpoOps gpo_buzzer_ops = {
  .on     = buzzer_on_impl,
  .off    = buzzer_off_impl,
  .set    = buzzer_set_impl,
  .toggle = buzzer_toggle_impl,
};

// 初始化: 绑定 TIM 通道和默认频率 → 注册 ops
void gpo_buzzer_init(GpoBuzzer *me, GpoName name,
                     TIM_HandleTypeDef *htim, uint32_t ch, uint32_t duty) {
  gpo_base_init(&me->base, name);
  me->htim      = htim;
  me->ch        = ch;
  me->duty      = duty;
  me->base.ops  = &gpo_buzzer_ops;
}

// 反初始化: 停止 PWM → 清除
void gpo_buzzer_deinit(GpoBuzzer *me) {
  HAL_TIM_PWM_Stop(me->htim, me->ch);
  gpo_base_deinit(&me->base);
  me->htim = NULL;
  me->ch   = 0;
  me->duty = 0;
}
