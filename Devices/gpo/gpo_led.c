// LED 驱动 —— GpoBase 子类, 双模 GPIO / PWM
// GPIO 模式: 开关灯, 支持高低电平有效
// PWM  模式: 调光, 支持亮度渐变

#include "gpo_led.h"
#include "container_of.h"

// ===== GPIO 模式虚函数实现 =====

// GPIO 打开: 写有效电平
static void led_gpio_on(GpoBase *base) {
  GpoLed *me = container_of(base, GpoLed, base);
  GPIO_PinState level = me->active_low ? GPIO_PIN_RESET : GPIO_PIN_SET;
  HAL_GPIO_WritePin(me->port, me->pin, level);
  me->is_on = true;
}

// GPIO 关闭: 写无效电平
static void led_gpio_off(GpoBase *base) {
  GpoLed *me = container_of(base, GpoLed, base);
  GPIO_PinState level = me->active_low ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(me->port, me->pin, level);
  me->is_on = false;
}

// GPIO 翻转: 基于 is_on 状态
static void led_gpio_toggle(GpoBase *base) {
  GpoLed *me = container_of(base, GpoLed, base);
  if (me->is_on) {
    led_gpio_off(base);
  } else {
    led_gpio_on(base);
  }
}

// GPIO 模式虚表 —— set 为 NULL
static const GpoOps gpo_led_gpio_ops = {
  .on     = led_gpio_on,
  .off    = led_gpio_off,
  .set    = NULL,
  .toggle = led_gpio_toggle,
};

// ===== PWM 模式虚函数实现 =====

// PWM 打开: 启动 PWM + 恢复当前亮度
static void led_pwm_on(GpoBase *base) {
  GpoLed *me = container_of(base, GpoLed, base);
  HAL_TIM_PWM_Start(me->htim, me->ch);
  __HAL_TIM_SET_COMPARE(me->htim, me->ch, me->duty);
}

// PWM 关闭: 停止 PWM
static void led_pwm_off(GpoBase *base) {
  GpoLed *me = container_of(base, GpoLed, base);
  HAL_TIM_PWM_Stop(me->htim, me->ch);
}

// PWM 翻转: 读当前比较值判断亮灭
static void led_pwm_toggle(GpoBase *base) {
  GpoLed *me = container_of(base, GpoLed, base);
  if (__HAL_TIM_GET_COMPARE(me->htim, me->ch) > 0) {
    HAL_TIM_PWM_Stop(me->htim, me->ch);
  } else {
    HAL_TIM_PWM_Start(me->htim, me->ch);
    __HAL_TIM_SET_COMPARE(me->htim, me->ch, me->duty);
  }
}

// PWM 设置亮度: 更新占空比并立即生效
static void led_pwm_set(GpoBase *base, uint32_t duty) {
  GpoLed *me = container_of(base, GpoLed, base);
  me->duty = duty;
  __HAL_TIM_SET_COMPARE(me->htim, me->ch, duty);
}

// PWM 模式虚表 —— 三个操作全部实现
static const GpoOps gpo_led_pwm_ops = {
  .on     = led_pwm_on,
  .off    = led_pwm_off,
  .set    = led_pwm_set,
  .toggle = led_pwm_toggle,
};

// ===== 初始化 / 反初始化 =====

// GPIO 模式初始化
void gpo_led_init_gpio(GpoLed *me, GpoName name,
                       GPIO_TypeDef *port, uint16_t pin, bool active_low) {
  gpo_base_init(&me->base, name);
  me->use_pwm    = false;
  me->port       = port;
  me->pin        = pin;
  me->active_low = active_low;
  me->is_on      = false;
  me->htim       = NULL;
  me->ch         = 0;
  me->duty       = 0;
  me->base.ops   = &gpo_led_gpio_ops;

  // 默认关闭
  GPIO_PinState off_level = active_low ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(me->port, me->pin, off_level);
}

// PWM 模式初始化
void gpo_led_init_pwm(GpoLed *me, GpoName name,
                      TIM_HandleTypeDef *htim, uint32_t ch, uint32_t duty) {
  gpo_base_init(&me->base, name);
  me->use_pwm    = true;
  me->port       = NULL;
  me->pin        = 0;
  me->active_low = false;
  me->is_on      = false;
  me->htim       = htim;
  me->ch         = ch;
  me->duty       = duty;
  me->base.ops   = &gpo_led_pwm_ops;
}

// 反初始化
void gpo_led_deinit(GpoLed *me) {
  if (me->use_pwm) {
    HAL_TIM_PWM_Stop(me->htim, me->ch);
  } else {
    GPIO_PinState off_level = me->active_low ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(me->port, me->pin, off_level);
  }
  gpo_base_deinit(&me->base);
  me->port  = NULL;
  me->pin   = 0;
  me->is_on = false;
  me->htim  = NULL;
  me->ch    = 0;
  me->duty  = 0;
}
