// 非线性 PID —— PidBase 子类实现 (TI DCL NLPID)

#include "pid_nl.h"
#include "container_of.h"

// ======== ops 实现 ========

// 核心计算: 并行式非线性 PID
static float nl_compute(PidBase *base, float target, float measure) {
  PidNl *me = container_of(base, PidNl, base);
  const PidNlConfig *cfg = &me->cfg;

  // 误差折半 (输入归一化 ±1 时保持整形有效域)
  float v1 = (target - measure) * 0.5f;
  float v2 = (v1 < 0.0f) ? -1.0f : 1.0f;
  float v3 = MATH_ABS(v1);

  // 非线性整形: |e|>δ → 幂律 ; |e|≤δ → 线性
  float v4 = (v3 > cfg->delta_p) ? v2 * powf(v3, cfg->alpha_p) : v1 * cfg->gamma_p;
  float v5 = (v3 > cfg->delta_i) ? v2 * powf(v3, cfg->alpha_i) : v1 * cfg->gamma_i;
  float v9 = (v3 > cfg->delta_d) ? v2 * powf(v3, cfg->alpha_d) : v1 * cfg->gamma_d;

  // 积分通路 (抗饱和: i16=0 停止积分)
  me->i7 = (v5 * cfg->kp * cfg->ki * me->i16) + me->i7;
  float ui = me->i7;

  // 微分通路 (二阶滤波)
  float v10 = v9 * cfg->kd * cfg->c1;
  float v12 = v10 - me->d2 - me->d3;
  me->d2 = v10;
  me->d3 = v12 * cfg->c2;
  float ud = v12;

  // 输出合成 + 限幅 (物理限幅由基类 pid_compute 负责, 这里返回原始)
  float v13 = cfg->kp * (v4 + v12) + me->i7;

  // 记录钳位状态供基类抗饱和 (与原 NL 一致: 输出越界 → 停积分)
  if (v13 > base->out_max || v13 < base->out_min) {
    me->i16 = 0.0f;   // 钳位 → 停积分
  }
  (void)ui; (void)ud;

  return v13;
}

// 清零内部状态
static void nl_reset(PidBase *base) {
  PidNl *me = container_of(base, PidNl, base);
  me->d2 = 0.0f;
  me->d3 = 0.0f;
  me->i7 = 0.0f;
  me->i16 = 1.0f;   // 未饱和
}

// 抗积分饱和回调 (原 NL 以 i16 标志停积分, 已在 compute 内处理; 此处保持 flag 复位逻辑)
static void nl_on_saturation(PidBase *base, float raw, float clamped) {
  PidNl *me = container_of(base, PidNl, base);
  (void)raw; (void)clamped;
  me->i16 = 0.0f;   // 饱和 → 停积分
}

static const PidOps nl_ops = {
  .compute       = nl_compute,
  .reset         = nl_reset,
  .on_saturation = nl_on_saturation,
};

// ======== 构造 ========
void pid_nl_init(PidNl *me, float dt, float out_min, float out_max,
                 const PidNlConfig *cfg) {
  pid_base_init(&me->base);
  me->cfg          = *cfg;
  me->d2 = 0.0f; me->d3 = 0.0f; me->i7 = 0.0f; me->i16 = 1.0f;
  me->base.dt      = dt;
  me->base.out_min = out_min;
  me->base.out_max = out_max;
  me->base.anti_windup = true;
  me->base.ops = &nl_ops;
}

void pid_nl_update_config(PidNl *me, const PidNlConfig *cfg) {
  me->cfg = *cfg;
}

void pid_nl_set_kp(PidNl *me, float v) { me->cfg.kp = v; }
void pid_nl_set_ki(PidNl *me, float v) { me->cfg.ki = v; }
void pid_nl_set_kd(PidNl *me, float v) { me->cfg.kd = v; }

void pid_nl_set_filter_bw(PidNl *me, float fc, float dt) {
  float tau = 1.0f / (M_2PI * fc);
  me->cfg.c1 = 2.0f / (dt + 2.0f * tau);
  me->cfg.c2 = (dt - 2.0f * tau) / (dt + 2.0f * tau);
}

void pid_nl_set_gamma_from_delta(PidNl *me) {
  me->cfg.gamma_p = powf(me->cfg.delta_p, me->cfg.alpha_p - 1.0f);
  me->cfg.gamma_i = powf(me->cfg.delta_i, me->cfg.alpha_i - 1.0f);
  me->cfg.gamma_d = powf(me->cfg.delta_d, me->cfg.alpha_d - 1.0f);
}
