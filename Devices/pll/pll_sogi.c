// SOGI-PLL —— PllBase 子类实现
// 鉴相(PD): SOGI-QSG 生成正交 αβ → Park 投影取 q 轴 = v_q (锁相误差)
// 环路滤波(LF) + 振荡器(VCO): 复用基类 pll_base_lf_vco
//
// 与旧 comp_pll.c 的 SogiPll 算法等价 (双线性离散化 + Park 乘法投影),
// 只是把一致的 LF+VCO 抽到基类实现。

#include "pll_sogi.h"
#include "container_of.h"
#include "comp_math.h"
#include <string.h>   // memset

// 预计算 SOGI-QSG 系数 (双线性变换离散化)
// 连续传递函数:
//   同相: H_d(s) = k·ω·s / (s² + k·ω·s + ω²)
//   正交: H_q(s) = k·ω²  / (s² + k·ω·s + ω²)
static void sogi_coeff_update(PllSogi *me) {
  float omega = M_2PI * me->base.fn;  // 角频率 (rad/s)
  float t = me->base.delta_t;
  float c = 2.0f / t;                 // 双线性变换常数
  float k = me->cfg.k_damp;

  float c2 = c * c;
  float k_w_c = k * omega * c;
  float w2 = omega * omega;

  float den = c2 + k_w_c + w2;        // 分母公共项

  // 同相通道 (带通): b0*(u[k] - u[k-2])
  me->osg_b0 = k_w_c / den;
  me->osg_b2 = -me->osg_b0;
  me->osg_a1 = (2.0f * c2 - 2.0f * w2) / den;
  me->osg_a2 = -(c2 - k_w_c + w2) / den;

  // 正交通道 (低通)
  me->osg_qb0 = k * w2 / den;
  me->osg_qb1 = 2.0f * me->osg_qb0;
  me->osg_qb2 = me->osg_qb0;
}

// ======== PD: SOGI 鉴相 ========
// 阶段: 历史移位 → SOGI-QSG 双通道 → Park 投影取 q 轴 → 基类 LF+VCO
static void sogi_run(PllBase *base, const PllInput *in) {
  PllSogi *me = container_of(base, PllSogi, base);

  // ---- 历史移位 ----
  me->u[2] = me->u[1];
  me->u[1] = me->u[0];
  me->u[0] = in->v;

  // ---- SOGI-QSG 同相输出 (带通) ----
  float osg_u_new = me->osg_b0 * (me->u[0] - me->u[2])
                  + me->osg_a1 * me->osg_u[1]
                  + me->osg_a2 * me->osg_u[2];

  // ---- SOGI-QSG 正交输出 (90° 移相) ----
  float osg_qu_new = me->osg_qb0 * me->u[0]
                   + me->osg_qb1 * me->u[1]
                   + me->osg_qb2 * me->u[2]
                   + me->osg_a1 * me->osg_qu[1]
                   + me->osg_a2 * me->osg_qu[2];

  // SOGI 历史移位
  me->osg_u[2] = me->osg_u[1];
  me->osg_u[1] = me->osg_u[0];
  me->osg_u[0] = osg_u_new;

  me->osg_qu[2] = me->osg_qu[1];
  me->osg_qu[1] = me->osg_qu[0];
  me->osg_qu[0] = osg_qu_new;

  // ---- Park 投影 → q 轴 = 锁相误差 (α=osg_u, β=osg_qu) ----
  // q = cosθ·α + sinθ·β ; 锁定时 q → 0
  float v_q = base->cos_val * me->osg_u[0] + base->sin_val * me->osg_qu[0];
  me->u_q = v_q;

  // ---- 基类 LF + VCO ----
  pll_base_lf_vco(base, v_q);
}

// 清零: 重置 SOGI 历史 + 基类 LF/VCO 运行时状态 (保留配置/ops)
static void sogi_reset(PllBase *base) {
  PllSogi *me = container_of(base, PllSogi, base);
  me->u[0] = me->u[1] = me->u[2] = 0.0f;
  me->osg_u[0] = me->osg_u[1] = me->osg_u[2] = 0.0f;
  me->osg_qu[0] = me->osg_qu[1] = me->osg_qu[2] = 0.0f;
  me->u_q = 0.0f;
  pll_base_reset_state(&me->base);
}

static const PllOps sogi_ops = {
  .run   = sogi_run,
  .reset = sogi_reset,
};

// ======== 构造 ========
void pll_sogi_init(PllSogi *me, float grid_freq_hz, float dt,
                   float kp, float ki, float k_damp) {
  memset(me, 0, sizeof(PllSogi));   // 清零子类 (含历史缓冲), 防未初始化引发发散
  pll_base_init(&me->base);

  me->cfg.k_damp = k_damp;
  me->base.fn      = grid_freq_hz;
  me->base.delta_t = dt;
  me->base.freq_lim = 0.0f;  // SOGI 不限制 (锁定速度快)
  me->base.ops     = &sogi_ops;

  sogi_coeff_update(me);
  pll_base_set_pi(&me->base, kp, ki);
}

void pll_sogi_coeff_update(PllSogi *me) {
  sogi_coeff_update(me);
}
