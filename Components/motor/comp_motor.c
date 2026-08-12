// 电机平台层 —— 抽象基类 + ops 分发
// 子类: MotTim (TIM4 PWM + AB 编码器)
// 应用层只通过 MotorBase* 句柄 + motor_write/encode/read 操作
//
// 来源: LitteCar 项目 (SmCar/LitteCar_STM32)
// 适配: STM32_OOP 框架 (this → me, 统一 include 路径)

#include "comp_motor.h"
#include <assert.h>
#include <stddef.h>

// 基类构造: ops 由子类 init 时绑定
void motor_init(MotorBase *me) {
  me->ops = NULL;
}

// 写入 PWM: 断言 ops->write 必须存在 → 委托子类
void motor_write(MotorBase *me, int16_t d) {
  assert(me->ops->write);
  me->ops->write(me, d);
}

// 编码器脉冲输入: 断言 ops->encode 必须存在 → 委托子类 (EXTI 回调中调用)
void motor_encode(MotorBase *me, uint16_t pin) {
  assert(me->ops->encode);
  me->ops->encode(me, pin);
}

// 读取并清零速度: 断言 ops->read 必须存在 → 委托子类 (每 10ms 调用)
int16_t motor_read(MotorBase *me) {
  assert(me->ops->read);
  return me->ops->read(me);
}
