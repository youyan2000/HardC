#ifndef DEV_GPO_LED_H
#define DEV_GPO_LED_H

// LED 驱动 —— GpoBase 子类
// 双模: GPIO 模式 (开关) / PWM 模式 (调亮度)
//   - GPIO 模式: gpo_led_init_gpio() → on/off 写电平, set=NULL
//   - PWM  模式: gpo_led_init_pwm()  → on/off 启停 PWM, set 调亮度

#include "comp_gpo.h"
#include "stm32f1xx_hal.h"

// 子类结构体 —— base 必须是第一个成员
typedef struct {
  GpoBase  base;        // 基类 (必须为第一个成员)

  bool     use_pwm;     // true=PWM 调光, false=GPIO 开关

  // GPIO 字段 (use_pwm=false 时有效)
  GPIO_TypeDef *port;   // GPIO 端口
  uint16_t      pin;    // GPIO 引脚号
  bool          active_low;  // true=低电平亮, false=高电平亮
  bool          is_on;       // 当前开关状态

  // PWM 字段 (use_pwm=true 时有效)
  TIM_HandleTypeDef *htim;  // 定时器句柄
  uint32_t           ch;    // PWM 通道
  uint32_t           duty;  // 当前占空比 (0~ARR)
} GpoLed;

// GPIO 模式初始化: 开关灯
void gpo_led_init_gpio(GpoLed *me, GpoName name,
                       GPIO_TypeDef *port, uint16_t pin, bool active_low);

// PWM 模式初始化: 调光灯
void gpo_led_init_pwm(GpoLed *me, GpoName name,
                      TIM_HandleTypeDef *htim, uint32_t ch, uint32_t duty);

// 反初始化
void gpo_led_deinit(GpoLed *me);

#endif
