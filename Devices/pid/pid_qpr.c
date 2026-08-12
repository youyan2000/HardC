// QPR (准比例谐振) 控制器
//
// 算法:
//   1. 死区检测 (可选)
//   2. P 项: Kp × error
//   3. 阻尼谐振项: 对每个谐波用 biquad DF1:
//        y = b0*error + b1*x1 + b2*x2 - a1*y1 - a2*y2
//   4. 输出 = P项 + Σy_res[n]
//
// 系数推导 (Tustin 变换):
//   R(s) = Ki * ω_c * s / (s² + ω_c*s + ω²)
//   s = K*(z-1)/(z+1), K = 2/dt
//
//   分子: Ki*ω_c*K*(z²-1) / D
//   分母: (K²+ω_c*K+ω²)*z² + (-2K²+2ω²)*z + (K²-ω_c*K+ω²)
//   归一化:
//     den = K² + ω_c*K + ω²
//     b0 = Ki * ω_c * K / den
//     b1 = 0
//     b2 = -b0
//     a1 = 2*(ω² - K²) / den
//     a2 = (K² - ω_c*K + ω²) / den
//
// PR vs QPR 关键差异:
//   - QPR 的 a2 < 1 (极点位于单位圆内) → 数值稳定、有限增益
//   - PR 的 a2 = 1 (极点恰在单位圆上) → 理论上无限增益但数值敏感
//   - 加大 bandwidth (ω_c) → 增益降低、带宽增大、鲁棒性提高

#include "pid_qpr.h"
#include "container_of.h"
#include "comp_math.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ======== 内部: 预计算单个谐振器的 biquad 系数 ========
static void qpr_calc_coeffs(QPRResonator *r, float w, float wc, float ki, float dt) {
  float K   = 2.0f / dt;
  float K2  = K * K;
  float w2  = w * w;
  float den = K2 + wc * K + w2;

  r->b0 =  ki * wc * K / den;
  r->b1 =  0;
  r->b2 = -r->b0;                    // 分子: ω_c*K*(z²-1) → b2 = -b0
  r->a1 =  2.0f * (w2 - K2) / den;
  r->a2 =  (K2 - wc * K + w2) / den; // QPR: a2 < 1 (稳定), PR 极限: a2 = 1

  r->ki = ki;
  r->w  = w;
}

// ======== ops 实现 ========

// 核心计算: P项 + 各谐波阻尼谐振器输出
static float qpr_compute(PidBase *base, float target, float measure) {
  PidQPR *me = container_of(base, PidQPR, base);
  float error = target - measure;

  // 1. 死区
  if (me->cfg.deadzone > 0 && math_abs_f(error) < me->cfg.deadzone) {
    error = 0;
  }

  // 2. P 项
  float output = me->cfg.kp * error;

  // 3. 各谐波阻尼谐振器: biquad DF1
  for (int n = 0; n < me->cfg.num_harmonics; n++) {
    QPRResonator *r = &me->res[n];

    // y = b0*x - a1*y1 - a2*y2 + b1*x1 + b2*x2
    float y = r->b0 * error
            - r->a1 * r->y1
            - r->a2 * r->y2
            + r->b1 * r->x1
            + r->b2 * r->x2;

    // 移位
    r->x2 = r->x1;
    r->x1 = error;
    r->y2 = r->y1;
    r->y1 = y;

    output += y;
  }

  return output;  // 限幅由基类 pid_compute 统一处理
}

// 清零: 重置所有谐振器状态
static void qpr_reset(PidBase *base) {
  PidQPR *me = container_of(base, PidQPR, base);
  for (int n = 0; n < QPR_MAX_HARMONICS; n++) {
    me->res[n].x1 = 0;
    me->res[n].x2 = 0;
    me->res[n].y1 = 0;
    me->res[n].y2 = 0;
  }
}

// QPR 无积分器, 不需要抗饱和
static const PidOps qpr_ops = {
  .compute       = qpr_compute,
  .reset         = qpr_reset,
  .on_saturation = NULL,
};

// ======== 构造 ========

void pid_qpr_init(PidQPR *me, float dt, float out_min, float out_max,
                  const QPRConfig *cfg) {
  pid_base_init(&me->base);
  me->cfg = *cfg;
  me->wc  = cfg->bandwidth;

  // 预计算所有谐波 biquad 系数
  memset(me->res, 0, sizeof(me->res));
  for (int n = 0; n < cfg->num_harmonics && n < QPR_MAX_HARMONICS; n++) {
    float w = cfg->harmonics[n].order * 2.0f * M_PI * cfg->f0;
    qpr_calc_coeffs(&me->res[n], w, me->wc, cfg->harmonics[n].ki, dt);
  }

  me->base.dt          = dt;
  me->base.out_min     = out_min;
  me->base.out_max     = out_max;
  me->base.anti_windup = false;
  me->base.ops         = &qpr_ops;
}

// ======== 运行时调参 ========

// 整体替换配置 (重算所有 biquad)
void pid_qpr_update_config(PidQPR *me, const QPRConfig *cfg) {
  me->cfg = *cfg;
  me->wc  = cfg->bandwidth;

  memset(me->res, 0, sizeof(me->res));
  float dt = me->base.dt;
  for (int n = 0; n < cfg->num_harmonics && n < QPR_MAX_HARMONICS; n++) {
    float w = cfg->harmonics[n].order * 2.0f * M_PI * cfg->f0;
    qpr_calc_coeffs(&me->res[n], w, me->wc, cfg->harmonics[n].ki, dt);
  }
}

void pid_qpr_set_kp(PidQPR *me, float kp) {
  me->cfg.kp = kp;
}

// 运行时修改单个谐波的 ki (重置该谐振器 + 重算系数)
void pid_qpr_set_ki(PidQPR *me, uint8_t harmonic_order, float ki) {
  float dt = me->base.dt;
  for (int n = 0; n < me->cfg.num_harmonics && n < QPR_MAX_HARMONICS; n++) {
    if (me->cfg.harmonics[n].order == harmonic_order) {
      me->cfg.harmonics[n].ki = ki;
      memset(&me->res[n], 0, sizeof(QPRResonator));
      float w = harmonic_order * 2.0f * M_PI * me->cfg.f0;
      qpr_calc_coeffs(&me->res[n], w, me->wc, ki, dt);
      break;
    }
  }
}

// 修改带宽 ω_c (影响所有谐波 → 全部重算)
void pid_qpr_set_bandwidth(PidQPR *me, float bandwidth_rad_s) {
  me->cfg.bandwidth = bandwidth_rad_s;
  me->wc = bandwidth_rad_s;

  memset(me->res, 0, sizeof(me->res));
  float dt = me->base.dt;
  for (int n = 0; n < me->cfg.num_harmonics && n < QPR_MAX_HARMONICS; n++) {
    float w = me->cfg.harmonics[n].order * 2.0f * M_PI * me->cfg.f0;
    qpr_calc_coeffs(&me->res[n], w, me->wc, me->cfg.harmonics[n].ki, dt);
  }
}

// 重调基波频率: 用于电网频率锁相环 (PLL) 检测到频率偏移时动态调整
void pid_qpr_retune_f0(PidQPR *me, float f0_new) {
  me->cfg.f0 = f0_new;

  memset(me->res, 0, sizeof(me->res));
  float dt = me->base.dt;
  for (int n = 0; n < me->cfg.num_harmonics && n < QPR_MAX_HARMONICS; n++) {
    float w = me->cfg.harmonics[n].order * 2.0f * M_PI * f0_new;
    qpr_calc_coeffs(&me->res[n], w, me->wc, me->cfg.harmonics[n].ki, dt);
  }
}
