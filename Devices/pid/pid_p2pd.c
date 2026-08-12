// P2PD 非线性 PID —— 无积分项, 平方项提供自动增益调度
//
// 算法: output = sign(err) × err² × Kpp + err × Kp + Δerr × Kd
//
// 增益调度特性:
//   - |err| 小 (直线): err² ≈ 0, Kp 主导温和修正, 不会 Z 字振荡
//   - |err| 大 (弯道): err² 急剧增长, 平方项主导强力修正
//
// 无积分器 → 没有积分饱和问题 → on_saturation = NULL
// reset 只清 prev_error

#include "pid_p2pd.h"
#include "container_of.h"

// ======== ops 实现 ========

// 核心计算: sign(err) × err² × Kpp + err × Kp + Δerr × Kd
static float p2pd_compute(PidBase *base, float target, float measure) {
  PidP2PD *me = container_of(base, PidP2PD, base);
  float err = target - measure;
  float out;

  // 平方项符号 = err 符号, 保持输出方向正确
  if (err >= 0) {
    out =  err * err * me->cfg.kpp + err * me->cfg.kp
        + (err - me->prev_error) * me->cfg.kd;
  } else {
    out = -err * err * me->cfg.kpp + err * me->cfg.kp
        + (err - me->prev_error) * me->cfg.kd;
  }

  me->prev_error = err;  // 保存本次误差, 供下拍 D 项
  return out;              // 限幅由基类 pid_compute 统一处理
}

// 清零: 只重置历史误差 (无积分器)
static void p2pd_reset(PidBase *base) {
  PidP2PD *me = container_of(base, PidP2PD, base);
  me->prev_error = 0;
}

// 无积分器 → 不需要抗饱和回调
static const PidOps p2pd_ops = {
  .compute       = p2pd_compute,
  .reset         = p2pd_reset,
  .on_saturation = NULL,        // P2PD 没有积分器, 不会饱和
};

// ======== 构造 ========

void pid_p2pd_init(PidP2PD *me, float dt, float out_min, float out_max,
                   const PidP2PDConfig *cfg) {
  pid_base_init(&me->base);
  me->cfg        = *cfg;
  me->prev_error = 0;

  me->base.dt          = dt;
  me->base.out_min     = out_min;
  me->base.out_max     = out_max;
  me->base.anti_windup = false;   // 无积分器, 不需要抗饱和
  me->base.ops         = &p2pd_ops;
}

// ======== 运行时调参 ========

void pid_p2pd_update_config(PidP2PD *me, const PidP2PDConfig *cfg) {
  me->cfg = *cfg;
}

void pid_p2pd_set_kpp(PidP2PD *me, float kpp) { me->cfg.kpp = kpp; }
void pid_p2pd_set_kp(PidP2PD *me, float kp)   { me->cfg.kp  = kp; }
void pid_p2pd_set_kd(PidP2PD *me, float kd)   { me->cfg.kd  = kd; }

void pid_p2pd_set_prev_error(PidP2PD *me, float val) {
  me->prev_error = val;
}
