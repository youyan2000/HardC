// Notch-PLL —— PllBase 子类实现
// 鉴相(PD): 积型检测器 upd = v×cos → 陷波滤波(2f0) → 送入基类 LF+VCO
// 环路滤波(LF) + 振荡器(VCO): 复用基类 pll_base_lf_vco
//
// 与旧 comp_pll.c 的 NotchPll 算法等价; LF+VCO 已抽到基类实现。
// 注意: 基类 VCO 用欧拉积分, 此处不重复精确离散振荡器 (保留 woo 输出供参考)。

#include "pll_notch.h"
#include "container_of.h"
#include "comp_math.h"
#include <string.h>   // memset

// 计算陷波滤波器系数 (双线性变换, 2 倍频陷波)
static void notch_coeff_calc(PllNotch *me) {
  float dt = me->base.delta_t;
  float wn = me->base.fn * M_2PI;     // 标称角频率
  float w2 = 2.0f * wn;               // 陷波频率 = 2×f0
  float c = 2.0f / dt;
  float c2 = c * c;
  float w2_sq = w2 * w2;
  float c1_damp = me->cfg.notch_depth;
  (void)me->cfg.notch_bw;   // notch_bw 保留为配置项 (原 TI SPLL 演化为带宽阻尼)

  float den = c2 + 2.0f * c1_damp * w2 * c + w2_sq;

  me->notch_b0 = (c2 + w2_sq) / den;
  me->notch_b1 = (2.0f * w2_sq - 2.0f * c2) / den;
  me->notch_b2 = (c2 + w2_sq) / den;
  me->notch_a1 = (2.0f * c2 - 2.0f * w2_sq) / den;
  me->notch_a2 = (c2 - 2.0f * c1_damp * w2 * c + w2_sq) / den;
}

// ======== PD: 积型鉴相 + 陷波 ========
static void notch_run(PllBase *base, const PllInput *in) {
  PllNotch *me = container_of(base, PllNotch, base);

  // ---- 阶段 1: 积型鉴相器 (product detector) ----
  // upd = v × cos(θ) — 含 2f0 纹波
  me->upd[2] = me->upd[1];
  me->upd[1] = me->upd[0];
  me->upd[0] = in->v * base->cos_val;

  // ---- 阶段 2: 陷波滤波器 (去除 2f0 纹波) ----
  float ynotch_new = me->notch_b0 * me->upd[0]
                   + me->notch_b1 * me->upd[1]
                   + me->notch_b2 * me->upd[2]
                   + me->notch_a1 * me->ynotch[1]
                   + me->notch_a2 * me->ynotch[2];

  me->ynotch[2] = me->ynotch[1];
  me->ynotch[1] = me->ynotch[0];
  me->ynotch[0] = ynotch_new;
  me->out_ynotch = ynotch_new;

  // ---- 阶段 3: 基类 LF + VCO ----
  me->wo = me->base.fn * M_2PI + base->ylf * M_2PI;
  pll_base_lf_vco(base, ynotch_new);
}

// 清零: 重置鉴相/陷波历史 + 基类运行时状态
static void notch_reset(PllBase *base) {
  PllNotch *me = container_of(base, PllNotch, base);
  me->upd[0] = me->upd[1] = me->upd[2] = 0.0f;
  me->ynotch[0] = me->ynotch[1] = me->ynotch[2] = 0.0f;
  me->out_ynotch = 0.0f;
  me->wo = me->base.fn * M_2PI;
  pll_base_reset_state(&me->base);
}

static const PllOps notch_ops = {
  .run   = notch_run,
  .reset = notch_reset,
};

// ======== 构造 ========
void pll_notch_init(PllNotch *me, float grid_freq_hz, float dt,
                    float kp, float ki) {
  memset(me, 0, sizeof(PllNotch));   // 清零子类 (含鉴相/陷波历史)
  pll_base_init(&me->base);

  me->cfg.notch_depth = 0.01f;
  me->cfg.notch_bw    = 0.1f;

  me->base.fn       = grid_freq_hz;
  me->base.delta_t  = dt;
  me->base.ops      = &notch_ops;
  me->wo            = me->base.fn * M_2PI;

  notch_coeff_calc(me);
  pll_base_set_pi(&me->base, kp, ki);
}

// 用新中心角频率重算陷波系数
void pll_notch_coeff_update(PllNotch *me, float wn) {
  me->base.fn = wn / M_2PI;
  notch_coeff_calc(me);
}
