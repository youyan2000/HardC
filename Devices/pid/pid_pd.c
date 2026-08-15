// 比例+微分 (PD) 控制器实现 —— Kp×err + Kd×diff
//
// 算法:
//   P = Kp × (Ref − Fbk)
//   D = Kd × diff  , 其中
//       d_on_measurement=false → diff = (err − err_prev) / dt       (标准D)
//       d_on_measurement=true  → diff = −(meas − meas_prev) / dt    (D先行, 取负与err同向)
//   Out = P + D
// 由基类 pid_compute 负责物理限幅
// 首拍 / reset 后微分项为 0 (无上一拍样本, 避免微分冲击)

#include "pid_pd.h"
#include "container_of.h"

// ======== ops 实现 ========

static float pd_compute(PidBase *base, float target, float measure) {
  PidPd *me = container_of(base, PidPd, base);

  float error = target - measure;

  // 比例项
  float p_term = me->cfg.kp * error;

  // 微分项 (差商) — 选微分先行或标准 D
  float d_term = 0.0f;
  if (me->initialized) {
    if (me->cfg.d_on_measurement) {
      // 微分先行: 对测量值求导. 注意与 err 同方向需取负 (err=target-meas)
      d_term = -(measure - me->meas_prev) / base->dt;
    } else {
      d_term = (error - me->err_prev) / base->dt;
    }
    d_term *= me->cfg.kd;
  } else {
    // 首拍: 无上一拍样本, 微分项置 0 (避免微分冲击)
    me->initialized = true;
  }

  // 更新历史
  me->err_prev  = error;
  me->meas_prev = measure;

  return p_term + d_term;
}

// 清零内部状态 (历史差分清零 + 复位首拍标志)
static void pd_reset(PidBase *base) {
  PidPd *me = container_of(base, PidPd, base);
  me->err_prev  = 0.0f;
  me->meas_prev = 0.0f;
  me->initialized = false;
}

// ======== 虚表 ========
static const PidOps pd_ops = {
  .compute        = pd_compute,
  .reset          = pd_reset,
  .on_saturation  = 0,   // PD 无积分, 不需要抗饱和回调
};

// ======== 构造 ========

void pid_pd_init(PidPd *me, float dt, float out_min, float out_max,
                 const PidPdConfig *cfg) {
  pid_base_init(&me->base);

  me->base.dt           = dt;
  me->base.out_min      = out_min;
  me->base.out_max      = out_max;
  me->base.anti_windup  = false;  // PD 无积分, 不需要抗饱和
  me->base.ops          = &pd_ops;

  me->err_prev  = 0.0f;
  me->meas_prev = 0.0f;
  me->initialized = false;

  if (cfg) {
    me->cfg = *cfg;
  }
}

void pid_pd_update_config(PidPd *me, const PidPdConfig *cfg) {
  me->cfg = *cfg;
}

void pid_pd_set_kp(PidPd *me, float kp) {
  me->cfg.kp = kp;
}

void pid_pd_set_kd(PidPd *me, float kd) {
  me->cfg.kd = kd;
}
