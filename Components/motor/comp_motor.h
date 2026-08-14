// 电机驱动平台层 —— 抽象基类
// 子类: TIM PWM + AB 编码器 (motor_tim)
//
// 适配: STM32_OOP 框架 (this → me, 统一 include 路径)

#ifndef COMP_MOTOR_H
#define COMP_MOTOR_H

#include <stdint.h>

// 电机实例名
typedef enum {
  MotA, // 电机 A
  MotB  // 电机 B
} MotName;

typedef struct MotorBase MotorBase;

// 虚函数指针类型
typedef void    (*motor_w_fn)(MotorBase *me, int16_t d);    // 写 PWM
typedef void    (*motor_e_fn)(MotorBase *me, uint16_t pin); // 喂入编码器边沿
typedef int16_t (*motor_r_fn)(MotorBase *me);               // 读取编码器增量并清零

// 虚函数表 (ops)
typedef struct {
  motor_w_fn write;   // [必须] 设置 PWM
  motor_e_fn encode;  // [必须] 编码器 EXTI 回调
  motor_r_fn read;    // [必须] 返回编码器增量并清零
} MotorOps;

// 基类结构体
struct MotorBase {
  const MotorOps *ops;
  MotName name;
};

void    motor_init  (MotorBase *me);
void    motor_write (MotorBase *me, int16_t d);
void    motor_encode(MotorBase *me, uint16_t pin);
int16_t motor_read (MotorBase *me);

#endif
