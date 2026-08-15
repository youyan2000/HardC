// 纯比例 (P) 控制器实现 —— 仅 Kp × err
//
// 算法:
//   Out = Kp × (Ref − Fbk)   ← 无限幅、无历史
// 由基类 pid_compute 负责物理限幅

#include "pid_p.h"
#include "container_of.h"

// ======== ops 实现 ========

static float p_compute(PidBase *base, float target, float measure) {
  PidP *me = container_of(base, PidP, base);
  return me->cfg.kp * (target - measure);
}

static void p_reset(PidBase *base) {
  (void) base;  // 纯比例无内部状态可清零
}

// ======== 虚表 ========
static const PidOps p_ops = {
  .compute        = p_compute,
  .reset          = p_reset,
  .on_saturation  = 0,   // 纯比例不需要抗饱和回调
};

// ======== 构造 ========

void pid_p_init(PidP *me, float dt, float out_min, float out_max,
                const PidPConfig *cfg) {
  pid_base_init(&me->base);

  me->base.dt           = dt;
  me->base.out_min      = out_min;
  me->base.out_max      = out_max;
  me->base.anti_windup  = false;  // 纯比例无积分, 不需要抗饱和
  me->base.ops          = &p_ops;

  if (cfg) {
    me->cfg = *cfg;
  }
}

void pid_p_update_config(PidP *me, const PidPConfig *cfg) {
  me->cfg = *cfg;
}

void pid_p_set_kp(PidP *me, float kp) {
  me->cfg.kp = kp;
}
