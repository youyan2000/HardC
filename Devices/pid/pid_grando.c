// PID_GRANDO 全功能 PID 实现
//
// 来源: TI PID_GRANDO macro (motor_control/math_blocks/v4.3/pid_grando.h)
// 翻译为 HardC 纯C float 版本
//
// 算法阶段:
//   1. 比例: up = Kr*Ref - Fbk
//   2. 微分: ud = c1*Kd*(Km*Ref - Fbk) - d2,  d2 = ud*c2
//   3. 积分: ui = ui_prev + Ki*w1*(Ref - Fbk)
//   4. 合成: v1 = Kp*(up + ui + ud)
//   5. 限幅: Out = clamp(v1, Umin, Umax), w1 = (Out==v1)?1:0

#include "pid_grando.h"
#include "container_of.h"
#include <math.h>
#include "comp_math.h"

// ======== ops 实现 ========

static float grando_compute(PidBase *base, float target, float measure) {
  PidGrando *me = container_of(base, PidGrando, base);

  float error = target - measure;

  // ---- 阶段 1: 比例支路 (设定点权重) ----
  // up = Kr * Ref - Fbk
  me->up = me->cfg.kr * target - measure;

  // ---- 阶段 2: 微分支路 (D 滤波) ----
  // ud = c1 * Kd * (Km*Ref - Fbk) - d2   ← 一阶 IIR 差分
  // d2 = ud * c2                           ← 反馈延迟
  float ud_input = me->cfg.c1 * me->cfg.kd * (me->cfg.km * target - measure);
  me->ud = ud_input - me->d2;
  me->d2 = me->ud * me->cfg.c2;

  // ---- 阶段 3: 积分支路 (回算法抗饱和) ----
  // ui = ui_prev + Ki * w1 * error
  // w1=1.0 正常积分, w1=0.0 冻结 (由上拍 on_saturation 设定)
  me->ui = me->ui_prev + me->cfg.ki * me->w1 * error;
  me->ui_prev = me->ui;

  // 重置 w1 为本拍的乐观预设 (下拍默认允许积分)
  // 若本拍饱和, on_saturation 将覆盖 w1=0 冻结下拍积分
  me->w1 = 1.0f;

  // ---- 阶段 4: 合成 (Kp 作用于三路之和) ----
  float v1 = me->cfg.kp * (me->up + me->ui + me->ud);

  return v1;  // 基类 pid_compute 负责限幅
}

static void grando_on_saturation(PidBase *base, float raw, float clamped) {
  (void)raw; (void)clamped;
  PidGrando *me = container_of(base, PidGrando, base);

  // 回算法: 本拍饱和 → 下拍冻结积分 (w1=0)
  me->w1 = 0.0f;
}

static void grando_reset(PidBase *base) {
  PidGrando *me = container_of(base, PidGrando, base);
  me->up = 0.0f;
  me->ui = 0.0f;
  me->ui_prev = 0.0f;
  me->ud = 0.0f;
  me->d2 = 0.0f;
  me->w1 = 1.0f;
}

// ======== 虚表 ========
static const PidOps grando_ops = {
  .compute       = grando_compute,
  .reset         = grando_reset,
  .on_saturation = grando_on_saturation,
};

// ======== 构造 ========

void pid_grando_init(PidGrando *me, float dt,
                     const PidGrandoConfig *cfg) {
  pid_base_init(&me->base);

  me->base.dt           = dt;
  me->base.out_min      = (cfg ? cfg->out_min : -1.0f);
  me->base.out_max      = (cfg ? cfg->out_max : 1.0f);
  me->base.anti_windup  = true;
  me->base.ops          = &grando_ops;

  me->up      = 0.0f;
  me->ui      = 0.0f;
  me->ui_prev = 0.0f;
  me->ud      = 0.0f;
  me->d2      = 0.0f;
  me->w1      = 1.0f;

  if (cfg) {
    me->cfg = *cfg;
  }
}

void pid_grando_update_config(PidGrando *me, const PidGrandoConfig *cfg) {
  float ui_saved = me->ui;
  float ui_prev_saved = me->ui_prev;
  float d2_saved = me->d2;

  me->cfg = *cfg;

  me->ui = ui_saved;
  me->ui_prev = ui_prev_saved;
  me->d2 = d2_saved;
}

void pid_grando_set_kp(PidGrando *me, float kp) {
  me->cfg.kp = kp;
}

void pid_grando_set_ki(PidGrando *me, float ki) {
  me->cfg.ki = ki;
}

void pid_grando_set_kd(PidGrando *me, float kd) {
  me->cfg.kd = kd;
}

void pid_grando_set_kr(PidGrando *me, float kr) {
  me->cfg.kr = kr;
}

void pid_grando_set_dfilt_freq(PidGrando *me, float fc_hz) {
  float dt = me->base.dt;
  float wc_t = M_2PI * fc_hz * dt;
  float den = 1.0f / (1.0f + wc_t);

  me->cfg.c1 = wc_t * den;
  me->cfg.c2 = den;
}
