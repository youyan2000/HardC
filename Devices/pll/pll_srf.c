// SRF-PLL —— PllBase 子类实现
// 鉴相(PD): Park 旋转坐标投影取 q 轴 = v_q (锁相误差)
//   v_q = v_β·cosθ − v_α·sinθ ; 锁定时 v_q → 0
// 环路滤波(LF) + 振荡器(VCO): 复用基类 pll_base_lf_vco
//
// 与旧 comp_pll.c 的 SrfPll 算法等价; LF+VCO 已抽到基类实现。

#include "pll_srf.h"
#include "container_of.h"
#include "comp_math.h"
#include <string.h>   // memset

// ======== PD: Park 旋转坐标鉴相 ========
// 输入帧 v_alpha/v_beta (外部 Clarke 得到) → q 轴投影 → 基类 LF+VCO
static void srf_run(PllBase *base, const PllInput *in) {
  PllSrf *me = container_of(base, PllSrf, base);

  // Park 旋转投影 (用基类缓存的 sin/cos 反馈)
  float v_q = in->v_beta * base->cos_val - in->v_alpha * base->sin_val;
  me->v_q = v_q;

  // 基类 LF + VCO (内含 freq_lim 限幅)
  pll_base_lf_vco(base, v_q);
}

// 清零: 重置基类 LF/VCO 运行时状态 (保留配置/ops)
static void srf_reset(PllBase *base) {
  PllSrf *me = container_of(base, PllSrf, base);
  me->v_q = 0.0f;
  pll_base_reset_state(&me->base);
}

static const PllOps srf_ops = {
  .run   = srf_run,
  .reset = srf_reset,
};

// ======== 构造 ========
void pll_srf_init(PllSrf *me, float grid_freq_hz, float dt,
                  float kp, float ki) {
  memset(me, 0, sizeof(PllSrf));   // 清零子类 (含 v_q)
  pll_base_init(&me->base);

  me->cfg.freq_lim = 200.0f;   // 默认 ±200 Hz

  me->base.fn       = grid_freq_hz;
  me->base.delta_t  = dt;
  me->base.freq_lim = me->cfg.freq_lim;
  me->base.ops      = &srf_ops;

  pll_base_set_pi(&me->base, kp, ki);
}
