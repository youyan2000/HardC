// DDSRF-PLL —— PllBase 子类实现
// 鉴相(PD): 正负序双 dq 交叉解耦 → 4 路 LPF → 正序 q 轴 = v_q (锁相误差)
// 环路滤波(LF) + 振荡器(VCO): 复用基类 pll_base_lf_vco
//
// 解耦网络方程 (复域):
//   dq_p_decoup = dq_p_raw - dq_n_lpf × (cos2θ + j·sin2θ)
//   dq_n_decoup = dq_n_raw - dq_p_lpf × (cos2θ - j·sin2θ)

#include "pll_ddsrf.h"
#include "container_of.h"
#include "comp_math.h"
#include <string.h>   // memset

// 一阶 LPF: y[k] = k1·x[k] + k2·y[k-1]
static inline float ddsrf_lpf(float *state, float k1, float k2, float x) {
  float y = k1 * x + k2 * state[0];
  state[1] = state[0];
  state[0] = y;
  return y;
}

// ======== PD: DDSRF 正负序解耦鉴相 ========
static void ddsrf_run(PllBase *base, const PllInput *in) {
  PllDdsrf *me = container_of(base, PllDdsrf, base);

  // 存取输入
  me->d_p = in->d_p;
  me->d_n = in->d_n;
  me->q_p = in->q_p;
  me->q_n = in->q_n;

  // ---- 阶段 1: 解耦网络 (用基类反馈 θ) ----
  // cos(2θ) = cos²-sin², sin(2θ) = 2·sin·cos
  me->cos_2theta = base->cos_val * base->cos_val - base->sin_val * base->sin_val;
  me->sin_2theta = 2.0f * base->sin_val * base->cos_val;

  // 正序解耦
  me->d_p_decoupl = me->d_p - (me->d_n_lpf * me->cos_2theta + me->q_n_lpf * me->sin_2theta);
  me->q_p_decoupl = me->q_p - (me->q_n_lpf * me->cos_2theta - me->d_n_lpf * me->sin_2theta);

  // 负序解耦
  me->d_n_decoupl = me->d_n - (me->d_p_lpf * me->cos_2theta - me->q_p_lpf * me->sin_2theta);
  me->q_n_decoupl = me->q_n - (me->q_p_lpf * me->cos_2theta + me->d_p_lpf * me->sin_2theta);

  // ---- 阶段 2: 4 路 LPF (对解耦后的 dq 分别滤波) ----
  me->d_p_lpf = ddsrf_lpf(me->y, me->k1, me->k2, me->d_p_decoupl);
  me->q_p_lpf = ddsrf_lpf(me->x, me->k1, me->k2, me->q_p_decoupl);
  me->d_n_lpf = ddsrf_lpf(me->w, me->k1, me->k2, me->d_n_decoupl);
  me->q_n_lpf = ddsrf_lpf(me->z, me->k1, me->k2, me->q_n_decoupl);

  // ---- 阶段 3: 正序 q 轴 → 基类 LF + VCO ----
  pll_base_lf_vco(base, me->q_p_lpf);
}

// 清零: 重置解耦网络 + 4 路 LPF + 基类运行时状态
static void ddsrf_reset(PllBase *base) {
  PllDdsrf *me = container_of(base, PllDdsrf, base);
  me->y[0] = me->y[1] = 0.0f;
  me->x[0] = me->x[1] = 0.0f;
  me->w[0] = me->w[1] = 0.0f;
  me->z[0] = me->z[1] = 0.0f;
  me->d_p_lpf = me->d_n_lpf = me->q_p_lpf = me->q_n_lpf = 0.0f;
  pll_base_reset_state(&me->base);
}

static const PllOps ddsrf_ops = {
  .run   = ddsrf_run,
  .reset = ddsrf_reset,
};

// ======== 构造 ========
void pll_ddsrf_init(PllDdsrf *me, float grid_freq_hz, float dt,
                    float kp, float ki) {
  memset(me, 0, sizeof(PllDdsrf));   // 清零子类 (含 4 路 LPF 状态)
  pll_base_init(&me->base);

  me->cfg.lpf_fc = 50.0f;   // 解耦 LPF 截止 ~50Hz

  me->base.fn      = grid_freq_hz;
  me->base.delta_t = dt;
  me->base.ops     = &ddsrf_ops;

  // 解耦网络 LPF 系数 (一阶低通)
  float wc = M_2PI * me->cfg.lpf_fc;
  float den_lpf = 1.0f / (1.0f + wc * dt);
  me->k2 = den_lpf;              // 反馈 = 1/(1+ωc·T)
  me->k1 = wc * dt * den_lpf;    // 输入 = ωc·T/(1+ωc·T)

  pll_base_set_pi(&me->base, kp, ki);
}
