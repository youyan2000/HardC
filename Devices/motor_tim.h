// TIM PWM + AB 编码器电机驱动 —— MotorBase 的子类
// PWM: TIM4 CH1-CH4, TB6612 驱动, d>0 正转 d<0 反转, 10kHz
// 编码器: 一对 AB 相, 4 倍频鉴相
//
// 来源: LitteCar 项目 (SmCar/LitteCar_STM32)
// 适配: STM32_OOP 框架 (this → me, 统一 include 路径)

#ifndef MOTOR_TIM_H
#define MOTOR_TIM_H

#include "../Components/comp_motor.h"
#include "stm32f1xx_hal.h"

// 编码器 AB 相引脚配置
typedef struct {
  GPIO_TypeDef *a_port;
  uint16_t      a_pin;
  GPIO_TypeDef *b_port;
  uint16_t      b_pin;
} AbEnc;

// PWM 通道配置 —— 指向 TIM CCR 寄存器
typedef struct {
  __IO uint32_t *ccr1;
  __IO uint32_t *ccr2;
} PwmCh;

typedef struct {
  MotorBase base;  // 基类: ops 虚表 + name
  PwmCh     pwm;   // PWM 双通道: ccr1 正向桥臂, ccr2 反向桥臂, 指向 TIMx->CCRx
  AbEnc     enc;   // 编码器 AB 相引脚, 4 倍频鉴相
  int16_t   cnt;   // 本周期脉冲增量, read() 时清零
  int8_t    inv;   // 方向取反: +1 正常, -1 时 PWM 输出和编码器计数同时翻转
} MotTim;

void mt_init(MotTim *me, MotName name, const PwmCh *pwm, const AbEnc *enc, int8_t inv);

#endif
