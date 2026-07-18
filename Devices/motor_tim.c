// 电机驱动 —— MotorBase 子类, TIMx PWM + AB 相编码器
// TB6612 双路 H 桥: CCR1/CCR2 控制 A 路, CCR3/CCR4 控制 B 路
// inv 字段同时翻转 PWM 方向和编码器方向, 使应用层两电机完全对称
// 编码器 4 倍频鉴相: AB 相每个边沿产生中断, 查 3-bit 状态表判断方向
//
// 来源: LitteCar 项目 (SmCar/LitteCar_STM32)
// 适配: STM32_OOP 框架 (this → me, 统一 include 路径)

#include "motor_tim.h"
#include "../BSP/container_of.h"
#include "../Components/comp_math.h"

#define PWM_MAX 7200  // TIMx ARR=7199, >ARR 即 100% 占空比

// -------- ops 实现 --------

// 写入 PWM: 限幅 ±4000 → inv 方向翻转 → 设置 CCR1/CCR2 差动输出
static void write_impl(MotorBase *base, int16_t d) {
  MotTim *me = container_of(base, MotTim, base);
  d *= me->inv;
  math_constrain_i16(&d, -7200, 7200);
  if (d < 0) { *me->pwm.ccr1 = PWM_MAX; *me->pwm.ccr2 = PWM_MAX + d; }
  else       { *me->pwm.ccr2 = PWM_MAX; *me->pwm.ccr1 = PWM_MAX - d; }
}

// 编码器 4 倍频鉴相: AB 相每个边沿产生中断，读当前两路电平组成 3-bit 状态
//   位0: 是否 A 相触发   位1: A 电平   位2: B 电平
// 查表: t 为奇数 → 反转, t 为偶数 → 正转; 每个 AB 周期产生 4 次计数 (4 倍频)
static void encode_impl(MotorBase *base, uint16_t pin) {
  MotTim *me = container_of(base, MotTim, base);

  int is_a = (pin == me->enc.a_pin);
  int is_b = (pin == me->enc.b_pin);
  if (!(is_a | is_b)) return;

  int t = is_a
        + (HAL_GPIO_ReadPin(me->enc.a_port, me->enc.a_pin) == GPIO_PIN_SET)
        + (HAL_GPIO_ReadPin(me->enc.b_port, me->enc.b_pin) == GPIO_PIN_SET);
  if (t & 1) me->cnt -= me->inv; else me->cnt += me->inv;
}

// 读取并清零编码器速度: 返回上一周期脉冲增量, 用于 motor_tim 每 10ms 采样
static int16_t read_impl(MotorBase *base) {
  MotTim *me = container_of(base, MotTim, base);
  int16_t v = me->cnt;
  me->cnt = 0;
  return v;
}

// 电机驱动虚函数表
static const MotorOps mt_ops = {
  .write  = write_impl,
  .encode = encode_impl,
  .read   = read_impl,
};

// -------- 构造 --------

// 初始化电机驱动: 调基类构造 → 绑定 PWM 通道和编码器引脚 → 注册 ops
// inv: 方向翻转系数 (1=正常, -1=物理反向, 驱动层消化两电机安装方向差异)
void mt_init(MotTim *me, MotName name, const PwmCh *pwm, const AbEnc *enc, int8_t inv) {
  motor_init(&me->base);
  me->base.name = name;
  me->pwm = *pwm;
  me->enc = *enc;
  me->cnt = 0;
  me->inv = inv;
  me->base.ops = &mt_ops;
}
