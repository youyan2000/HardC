// 角度+速度级联 PID —— 组合两个独立 PID 控制器
//
// 每次 tick 流程:
//   1. 外环: pid_compute(outer, target, outer_fb)
//      → 角度误差经外环 PID 计算 → 外环输出 (PWM 域)
//      → 外环的限幅/抗饱和由 pid_compute 自动处理
//   2. 缩放: inner_target = outer_output / scale
//      → 将 PWM 域输出转换为速度域目标
//   3. 限幅: inner_target clamp ±inner_limit
//      → 防止内环接收超范围速度目标
//   4. 内环: pid_compute(inner, inner_target, inner_fb)
//      → 速度误差经内环 PID 计算 → 最终 PWM 输出
//      → 内环的限幅/抗饱和由 pid_compute 自动处理

#include "pid_cascade.h"
#include "comp_math.h"

// ======== 构造 ========

void pid_cascade_init(PidCascade *me, PidBase *outer, PidBase *inner,
                      float scale, float inner_limit) {
  me->outer       = outer;
  me->inner       = inner;
  me->scale       = scale;
  me->inner_limit = inner_limit;
}

// ======== 级联计算 ========

float pid_cascade_compute(PidCascade *me, float target,
                          float outer_fb, float inner_fb) {
  // 1. 外环: 计算角度/位置 PID 输出 (含限幅+抗饱和)
  float outer_out = pid_compute(me->outer, target, outer_fb);

  // 2. 缩放: PWM 域 → 速度域
  float inner_target = outer_out / me->scale;

  // 3. 内环目标限幅: 防止请求超范围的电机速度
  math_constrain_f(&inner_target, -me->inner_limit, me->inner_limit);

  // 4. 内环: 计算速度 PID 输出 (含限幅+抗饱和), 返回最终 PWM
  return pid_compute(me->inner, inner_target, inner_fb);
}

// ======== 清零 ========

void pid_cascade_reset(PidCascade *me) {
  pid_reset(me->outer);
  pid_reset(me->inner);
}
