// SOGI-FLL —— PllBase 子类实现
// 鉴相(PD): SOGI-QSG 正交 → Park 投影取 q 轴 = v_q
// FLL:      用正交误差驱动频率自适应, 每拍重算 SOGI 系数与 fn
// 环路滤波(LF) + 振荡器(VCO): 复用基类 pll_base_lf_vco
//
// 与旧 comp_sogi_fll.h 的 SogiFll 算法等价; 区别是 LF(Tustin) 系数由构造直接传入 (TI 风格),
// fn 被 FLL 自适应更新, 基类 VCO 每拍用刷新后的 fn 积分相位。

#include "pll_sogi_fll.h"
#include "container_of.h"
#include "comp_math.h"
#include <string.h>   // memset

// 从自适应角频率 w_dash 用双线性变换计算 SOGI 系数 (不碰 FLL 积分器 x3)
static inline void sogi_fll_coeff_recalc(PllSogiFll *me) {
  float osgx, osgy, temp;

  osgx = 2.0f * me->cfg.k_damp * me->w_dash * me->base.delta_t;
  osgy = (me->w_dash * me->base.delta_t) * (me->w_dash * me->base.delta_t);
  temp = 1.0f / (osgx + osgy + 4.0f);

  me->osg_b0 = osgx * temp;
  me->osg_b2 = -me->osg_b0;
  me->osg_a1 = 2.0f * (4.0f - osgy) * temp;
  me->osg_a2 = (osgx - osgy - 4.0f) * temp;

  me->osg_qb0 = me->cfg.k_damp * osgy * temp;
  me->osg_qb1 = 2.0f * me->osg_qb0;
  me->osg_qb2 = me->osg_qb0;
}

// ======== PD: SOGI 鉴相 + FLL 频率自适 ========
static void sogi_fll_run(PllBase *base, const PllInput *in) {
  PllSogiFll *me = container_of(base, PllSogiFll, base);

  me->u[0] = in->v;

  // ---- SOGI-QSG 同相输出 (α) ----
  me->osg_u[0] = me->osg_b0 * (me->u[0] - me->u[2])
               + me->osg_a1 * me->osg_u[1]
               + me->osg_a2 * me->osg_u[2];
  me->osg_u[2] = me->osg_u[1];
  me->osg_u[1] = me->osg_u[0];

  // ---- SOGI-QSG 正交输出 (β) ----
  me->osg_qu[0] = me->osg_qb0 * me->u[0]
                + me->osg_qb1 * me->u[1]
                + me->osg_qb2 * me->u[2]
                + me->osg_a1 * me->osg_qu[1]
                + me->osg_a2 * me->osg_qu[2];
  me->osg_qu[2] = me->osg_qu[1];
  me->osg_qu[1] = me->osg_qu[0];

  me->u[2] = me->u[1];
  me->u[1] = me->u[0];

  // ---- Park 投影 (α=osg_u, β=osg_qu) ----
  me->u_q = base->cos_val * me->osg_u[0] + base->sin_val * me->osg_qu[0];
  me->u_d = base->cos_val * me->osg_qu[0] - base->sin_val * me->osg_u[0];

  // ---- FLL: 频率锁定环 —— 用正交误差驱动频率自适应 ----
  //   ef2 = −(u − u_α)·u_β·γ·dt (正交分量与误差相乘, 正比于频率偏差)
  me->ef2 = ((me->u[0] - me->osg_u[0]) * me->osg_qu[0])
            * me->cfg.gamma * me->base.delta_t * -1.0f;

  me->x3[0] = me->x3[1] + me->ef2;
  me->x3[1] = me->x3[0];

  me->w_dash = me->wc + me->x3[0];          // FLL 基准是固定名义中心 wc
  me->base.fn = me->w_dash / M_2PI;         // 自适应频率喂给 VCO 作为中心

  // ---- 用自适应频率重算 SOGI 系数 (保持 90° 正交性, 不清零 FLL 积分器) ----
  sogi_fll_coeff_recalc(me);

  // ---- 基类 LF + VCO (用刷新后的 fn + 直接传入的 b0/b1 系数) ----
  pll_base_lf_vco(base, me->u_q);
}

// 清零: 重置 SOGI/FLL 历史 + 基类运行时状态 (保留自适应 w_dash)
static void sogi_fll_reset(PllBase *base) {
  PllSogiFll *me = container_of(base, PllSogiFll, base);
  me->u[0] = me->u[1] = me->u[2] = 0.0f;
  me->osg_u[0] = me->osg_u[1] = me->osg_u[2] = 0.0f;
  me->osg_qu[0] = me->osg_qu[1] = me->osg_qu[2] = 0.0f;
  me->u_q = 0.0f;
  me->u_d = 0.0f;
  me->ef2 = 0.0f;
  me->x3[0] = me->x3[1] = 0.0f;
  pll_base_reset_state(&me->base);
}

static const PllOps sogi_fll_ops = {
  .run   = sogi_fll_run,
  .reset = sogi_fll_reset,
};

// ======== 构造 ========
// LF 系数 lpf_b0/lpf_b1 直接传入 (TI 风格, 由外部按带宽设计); 基类 b0/b1 同步。
void pll_sogi_fll_init(PllSogiFll *me, float grid_freq_hz, float isr_freq_hz,
                       float lpf_b0, float lpf_b1, float k_damp, float gamma) {
  memset(me, 0, sizeof(PllSogiFll));   // 清零子类 (含 SOGI/FLL 历史)
  pll_base_init(&me->base);

  me->cfg.k_damp = k_damp;
  me->cfg.gamma  = gamma;

  me->base.fn      = grid_freq_hz;
  me->base.delta_t = 1.0f / isr_freq_hz;
  me->base.ops     = &sogi_fll_ops;
  me->wc           = grid_freq_hz * M_2PI;  // 固定名义中心
  me->w_dash       = me->wc;

  // SOGI 初始化系数 + 清零 FLL 积分器
  sogi_fll_coeff_recalc(me);
  me->x3[0] = me->x3[1] = 0.0f;

  // LF 系数直接赋值
  me->base.b0 = lpf_b0;
  me->base.b1 = lpf_b1;
  me->base.kp = lpf_b0 + lpf_b1;
  me->base.ki = (lpf_b0 - lpf_b1) / me->base.delta_t;
}
