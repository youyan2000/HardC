// 纯积分 (I) 控制器实现 —— 仅 Ki × dt × err 累加
//
// 算法:
//   Out += Ki × dt × (Ref − Fbk)   ← 积分累积
// 抗积分饱和: 采用 clamping(截幅) 抗饱和 —— 基类检测到输出截断时回调把积分器
//   钳到限幅边界, 不继续 windup.
//
// 不变式: 本类的输出在数学上 == 积分累积值 (integral). 抗饱和回调依赖此等价,
//   `integral = clamped`. 若将来在此类上叠加 P 项(变成 PI), 必须重写抗饱和.

#include "pid_i.h"
#include "container_of.h"

// ======== ops 实现 ========

// 前向声明 (供 on_saturation 抗饱和钳位使用)
static void i_on_saturation(PidBase *base, float raw, float clamped);

static float i_compute(PidBase *base, float target, float measure) {
  PidI *me = container_of(base, PidI, base);

  // 积分累加: 由基类输出限幅决定是否 freeze/clamp
  me->integral += me->cfg.ki * base->dt * (target - measure);

  return me->integral;
}

// 抗积分饱和: 输出被限幅时把积分器钳到限幅边界 (clamping 抗饱和)
//   由于纯积分器的输出==integral, 当输出被钳到 clamped 时 integral 也钳到 clamped,
//   下拍在限制边界继续积分 (可自然退出饱和), 不会 windup 失控.
static void i_on_saturation(PidBase *base, float raw, float clamped) {
  PidI *me = container_of(base, PidI, base);
  (void) raw;
  me->integral = clamped;   // 钳到限幅边界 (依赖"输出==积分"不变式)
}

// 清零内部状态
static void i_reset(PidBase *base) {
  PidI *me = container_of(base, PidI, base);
  me->integral = 0.0f;
}

// ======== 虚表 ========
static const PidOps i_ops = {
  .compute        = i_compute,
  .reset          = i_reset,
  .on_saturation  = i_on_saturation,
};

// ======== 构造 ========

void pid_i_init(PidI *me, float dt, float out_min, float out_max,
                const PidIConfig *cfg) {
  pid_base_init(&me->base);

  me->base.dt           = dt;
  me->base.out_min      = out_min;
  me->base.out_max      = out_max;
  me->base.anti_windup  = true;  // 需要抗饱和: 限幅时冻结积分
  me->base.ops          = &i_ops;

  me->integral = 0.0f;

  if (cfg) {
    me->cfg = *cfg;
  }
}

void pid_i_update_config(PidI *me, const PidIConfig *cfg) {
  me->cfg = *cfg;
}

void pid_i_set_ki(PidI *me, float ki) {
  me->cfg.ki = ki;
}
