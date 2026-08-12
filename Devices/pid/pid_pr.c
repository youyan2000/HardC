// PR (比例谐振) 控制器
//
// 算法:
//   1. 死区检测 (可选)
//   2. P 项: Kp × error  (全频段比例响应)
//   3. 谐振项: 对每个谐波, 用 biquad DF1 计算 y_res[n]:
//        y = b0*error + b1*x1 + b2*x2 - a1*y1 - a2*y2
//        (b1=0, b2=-b0, a2=1.0 因为纯谐振器极点恰好在单位圆上)
//   4. 输出 = P项 + Σy_res[n]
//
// 系数推导 (Tustin 变换):
//   R(s) = Ki * s/(s² + ω²)
//   s = K*(z-1)/(z+1), K = 2/dt
//
//   归一化后 biquad:
//     b0 = Ki * K / (K² + ω²)
//     b1 = 0
//     b2 = -b0
//     a1 = 2*(ω² - K²) / (K² + ω²)
//     a2 = 1.0
//
// 关键特性:
//   - 在 ω 处增益 → ∞ (理论上零静差)
//   - 极点恰好在单位圆上 → 对频率漂移敏感, 适合电网频率稳定的场景
//   - reset 后谐振器需要数个周期重新建立稳态

#include "pid_pr.h"
#include "container_of.h"
#include "comp_math.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ======== 内部: 预计算单个谐振器的 biquad 系数 ========
// 基于 Tustin 变换, 在 init 和 update_config 时调用
static void pr_calc_coeffs(PRResonator *r, float w, float ki, float dt) {
  float K   = 2.0f / dt;       // Tustin 频率翘曲因子
  float K2  = K * K;
  float w2  = w * w;
  float den = K2 + w2;

  r->b0 =  ki * K / den;
  r->b1 =  0;
  r->b2 = -r->b0;              // b2 = -b0 (纯谐振器: 分子为 K*(z²-1))
  r->a1 =  2.0f * (w2 - K2) / den;
  r->a2 =  1.0f;               // 纯谐振器极点恰好在单位圆上

  r->ki = ki;
  r->w  = w;
}

// ======== ops 实现 ========

// 核心计算: P项 + 各谐波谐振器输出
static float pr_compute(PidBase *base, float target, float measure) {
  PidPR *me = container_of(base, PidPR, base);
  float error = target - measure;

  // 1. 死区
  if (me->cfg.deadzone > 0 && math_abs_f(error) < me->cfg.deadzone) {
    error = 0;
  }

  // 2. P 项: 全频段比例响应
  float output = me->cfg.kp * error;

  // 3. 各谐波谐振器: biquad DF1
  for (int n = 0; n < me->cfg.num_harmonics; n++) {
    PRResonator *r = &me->res[n];

    // y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2
    float y = r->b0 * error
            + r->b1 * r->x1
            + r->b2 * r->x2
            - r->a1 * r->y1
            - r->a2 * r->y2;

    // 移位: 输入历史
    r->x2 = r->x1;
    r->x1 = error;
    // 移位: 输出历史
    r->y2 = r->y1;
    r->y1 = y;

    output += y;
  }

  return output;  // 限幅由基类 pid_compute 统一处理
}

// 清零: 重置所有谐振器状态 (x1/x2/y1/y2 = 0)
static void pr_reset(PidBase *base) {
  PidPR *me = container_of(base, PidPR, base);
  for (int n = 0; n < PR_MAX_HARMONICS; n++) {
    me->res[n].x1 = 0;
    me->res[n].x2 = 0;
    me->res[n].y1 = 0;
    me->res[n].y2 = 0;
  }
}

// PR 没有积分器, 不需要抗饱和回调 (饱和由输出限幅处理即可)
static const PidOps pr_ops = {
  .compute       = pr_compute,
  .reset         = pr_reset,
  .on_saturation = NULL,    // 无积分器, 不积分的系统不会饱和
};

// ======== 构造 ========

void pid_pr_init(PidPR *me, float dt, float out_min, float out_max,
                 const PRConfig *cfg) {
  pid_base_init(&me->base);
  me->cfg = *cfg;

  // 预计算所有谐波的 biquad 系数
  memset(me->res, 0, sizeof(me->res));
  for (int n = 0; n < cfg->num_harmonics && n < PR_MAX_HARMONICS; n++) {
    float w = cfg->harmonics[n].order * 2.0f * M_PI * cfg->f0;
    pr_calc_coeffs(&me->res[n], w, cfg->harmonics[n].ki, dt);
  }

  me->base.dt          = dt;
  me->base.out_min     = out_min;
  me->base.out_max     = out_max;
  me->base.anti_windup = false;   // 无积分器, 不需抗饱和
  me->base.ops         = &pr_ops;
}

// ======== 运行时调参 ========

void pid_pr_update_config(PidPR *me, const PRConfig *cfg) {
  me->cfg = *cfg;

  // 重算所有谐振器系数
  memset(me->res, 0, sizeof(me->res));
  float dt = me->base.dt;
  for (int n = 0; n < cfg->num_harmonics && n < PR_MAX_HARMONICS; n++) {
    float w = cfg->harmonics[n].order * 2.0f * M_PI * cfg->f0;
    pr_calc_coeffs(&me->res[n], w, cfg->harmonics[n].ki, dt);
  }
}

void pid_pr_set_kp(PidPR *me, float kp) {
  me->cfg.kp = kp;
}

// 运行时修改单个谐波的 ki (重算对应 biquad 系数)
void pid_pr_set_ki(PidPR *me, uint8_t harmonic_order, float ki) {
  float dt = me->base.dt;
  for (int n = 0; n < me->cfg.num_harmonics && n < PR_MAX_HARMONICS; n++) {
    if (me->cfg.harmonics[n].order == harmonic_order) {
      me->cfg.harmonics[n].ki = ki;
      // 重置该谐振器状态 + 重算系数
      memset(&me->res[n], 0, sizeof(PRResonator));
      float w = harmonic_order * 2.0f * M_PI * me->cfg.f0;
      pr_calc_coeffs(&me->res[n], w, ki, dt);
      break;
    }
  }
}
