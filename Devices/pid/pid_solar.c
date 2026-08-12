// 太阳能 PI 控制器实现 — 条件积分抗饱和
//
// 来源: TI CNTL_PI_F (solar/v1.2/float)
// 算法:
//   up = Kp * (Ref - Fbk)
//   ui = (!prev_saturated) ? (Ki * up + ui_prev) : ui_prev   ← 冻结积分
//   v1 = up + ui
//   Out = clamp(v1, Umin, Umax)
//   记录 prev_output = Out → 下拍判断 prev_saturated = (Out != v1)

#include "pid_solar.h"
#include "container_of.h"

// ======== ops 实现 ========

static float solar_compute(PidBase *base, float target, float measure) {
  PidSolar *me = container_of(base, PidSolar, base);

  // 比例项
  me->up = me->cfg.kp * (target - measure);

  // 积分项 — 条件积分: 上拍饱和则冻结
  // prev_saturated 由上拍的 on_saturation 回调 (或本函数末尾) 更新
  if (!me->prev_saturated) {
    me->ui = me->cfg.ki * me->up + me->ui_prev;
  } else {
    me->ui = me->ui_prev;  // 冻结: 保持上拍积分值
  }
  me->ui_prev = me->ui;

  // 预饱和输出
  me->v1 = me->up + me->ui;

  // 预测本拍是否饱和 (用于下拍的积分冻结判断)
  // 注: 基类 pid_compute 在返回后会再次限幅, 但限幅规则相同
  float predicted_clamped = me->v1;
  if (predicted_clamped > base->out_max) predicted_clamped = base->out_max;
  else if (predicted_clamped < base->out_min) predicted_clamped = base->out_min;
  me->prev_output = predicted_clamped;
  me->prev_saturated = (predicted_clamped != me->v1);

  return me->v1;  // 基类 pid_compute 负责限幅
}

static void solar_reset(PidBase *base) {
  PidSolar *me = container_of(base, PidSolar, base);
  me->up = 0.0f;
  me->ui = 0.0f;
  me->ui_prev = 0.0f;
  me->v1 = 0.0f;
  me->prev_output = 0.0f;
  me->prev_saturated = false;
}

// ======== 虚表 ========
static const PidOps solar_ops = {
  .compute = solar_compute,
  .reset   = solar_reset,
  .on_saturation = 0,   // 条件积分在 compute 中自己判断, 不依赖回调
};

// ======== 构造 ========

void pid_solar_init(PidSolar *me, float dt, float out_min, float out_max,
                    const PidSolarConfig *cfg) {
  pid_base_init(&me->base);

  me->base.dt           = dt;
  me->base.out_min      = out_min;
  me->base.out_max      = out_max;
  me->base.anti_windup  = false;  // 内部自己做条件积分, 不依赖基类回调
  me->base.ops          = &solar_ops;

  me->up      = 0.0f;
  me->ui      = 0.0f;
  me->ui_prev = 0.0f;
  me->v1      = 0.0f;
  me->prev_output = 0.0f;
  me->prev_saturated = false;

  if (cfg) {
    me->cfg = *cfg;
  }
}

void pid_solar_update_config(PidSolar *me, const PidSolarConfig *cfg) {
  me->cfg = *cfg;   // cfg 是独立成员, 不影响 up/ui/ui_prev/v1 等状态
}

void pid_solar_set_kp(PidSolar *me, float kp) {
  me->cfg.kp = kp;
}

void pid_solar_set_ki(PidSolar *me, float ki) {
  me->cfg.ki = ki;
}
