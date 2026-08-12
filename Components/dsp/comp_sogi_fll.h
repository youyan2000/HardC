// 单相锁相环 — SOGI-QSG + 频率锁定环 (FLL) 自动适应电网频率
//
// 来源: TI C2000Ware Digital Power SDK libraries/spll/include/spll_1ph_sogi_fll.h
// 翻译为 C-OOP 纯C float 版本
//
// 与 comp_pll.h 的 SogiPll (SOGI-PLL) 区别:
//   SogiPll   — 中心频率固定为标称值 (fn), 仅用环路滤波器微调相位
//   SogiFll   — 增加频率锁定环 (FLL): 用正交输出误差驱动频率积分器,
//               w_dash = wc + x3 实时跟踪电网频率漂移, 并每拍重算 SOGI 系数
//   FLL 适用于弱电网/频率漂移场景 (柴油发电机、微电网、变速发电机接口)
//
// 算法:
//   1. SOGI-QSG: 从单相电压生成正交信号 (αβ)
//   2. Park: u_D = cos·qu − sin·u ; u_Q = cos·u + sin·qu
//   3. 环路滤波器 (积分型): ylf += b0·u_Q[n] + b1·u_Q[n-1]
//   4. VCO: theta += (fn + ylf)·dt·2π ; fn = 标称频率 + 滤波输出
//   5. FLL: ef2 = −(u − osg_u)·osg_qu·γ·dt ; x3 += ef2 ; w_dash = wc + x3
//   6. 用自适应 w_dash 重算 SOGI 双线性系数 (每拍)
//
// 输入电压需归一化到标幺 (pu), 角度输出 0~2π

#ifndef COMP_SOGI_FLL_H
#define COMP_SOGI_FLL_H

#include <math.h>

// ======================= SOGI-QSG 双线性系数 (随自适应频率每拍重算) =======================

typedef struct {
  float osg_b0;    // 同相 IIR 分子
  float osg_b2;    // 同相 IIR 分子 (延迟2拍, = −b0)
  float osg_a1;    // 同相 IIR 分母
  float osg_a2;    // 同相 IIR 分母
  float osg_qb0;   // 正交 IIR 分子
  float osg_qb1;   // 正交 IIR 分子 (延迟1拍, = 2·qb0)
  float osg_qb2;   // 正交 IIR 分子 (延迟2拍, = qb0)
} SogiFllOsgCoeff;

// ======================= 环路滤波器系数 (频率校正积分器) =======================

typedef struct {
  float b1;        // 输入延迟1拍系数
  float b0;        // 输入当前拍系数
} SogiFllLpfCoeff;

// ======================= SOGI-FLL 主结构体 =======================

typedef struct {
  // 输入/输出
  float u[3];          // 输入电压缓冲 (当前+2拍历史)
  float osg_u[3];      // 同相输出缓冲 (SOGI α)
  float osg_qu[3];     // 正交输出缓冲 (SOGI β)
  float u_q[2];        // q 轴分量 (锁相误差)
  float u_d[2];        // d 轴分量
  float ylf[2];        // 环路滤波器状态

  float fo;            // 输出: 当前频率 (Hz)
  float fn;            // 参数: 标称频率 (Hz, 也随 FLL 更新)
  float wc;            // 标称中心角频率 (rad/s) = 2π·fn0
  float theta;         // 输出: 锁相角 (0~2π)
  float cosine;        // 输出: cos(θ)
  float sine;          // 输出: sin(θ)
  float delta_t;       // 参数: 采样周期 (s) = 1/f_isr

  // FLL 状态
  float ef2;           // FLL 误差项
  float x3[2];         // FLL 频率积分器状态
  float w_dash;        // 自适应中心角频率 (rad/s) = wc + x3
  float gamma;         // 参数: FLL 增益 (γ)
  float k;             // 参数: SOGI 阻尼系数 (k, 典型 1.414)

  // 系数
  SogiFllOsgCoeff osg; // SOGI-QSG 双线性系数
  SogiFllLpfCoeff lpf; // 环路滤波器系数
} SogiFll;

// ======================= 系数计算 =======================

// 从自适应角频率 w_dash 用双线性变换计算 SOGI 系数 (不碰 FLL 积分器 x3)
// 运行中每拍随 w_dash 更新调用
static inline void sogi_fll_coeff_recalc(SogiFll *me) {
  float osgx, osgy, temp;

  osgx = 2.0f * me->k * me->w_dash * me->delta_t;
  osgy = (me->w_dash * me->delta_t) * (me->w_dash * me->delta_t);
  temp = 1.0f / (osgx + osgy + 4.0f);

  me->osg.osg_b0 = osgx * temp;
  me->osg.osg_b2 = -me->osg.osg_b0;
  me->osg.osg_a1 = 2.0f * (4.0f - osgy) * temp;
  me->osg.osg_a2 = (osgx - osgy - 4.0f) * temp;

  me->osg.osg_qb0 = me->k * osgy * temp;
  me->osg.osg_qb1 = 2.0f * me->osg.osg_qb0;
  me->osg.osg_qb2 = me->osg.osg_qb0;
}

// 初始化用系数计算 (与 TI coeff_calc 一致: 同时清零 FLL 积分器 x3)
// 仅在初始化/复位时调用, 不要在 run() 中调用 (会破坏 FLL 累加)
static inline void sogi_fll_coeff_calc(SogiFll *me) {
  sogi_fll_coeff_recalc(me);

  me->x3[0] = 0.0f;
  me->x3[1] = 0.0f;
}

// ======================= 初始化/配置 =======================

// 配置 SOGI-FLL
//   grid_freq_hz — 电网标称频率 (Hz, 如 50)
//   isr_freq_hz  — 采样频率 (Hz, ISR 调用频率)
//   lpf_b0/lpf_b1 — 环路滤波器系数 (频率校正积分器, 由外部按带宽设计)
//   k            — SOGI 阻尼系数 (典型 √2)
//   gamma        — FLL 增益 (γ, 决定频率跟踪收敛速度)
static inline void sogi_fll_init(SogiFll *me, float grid_freq_hz,
                                 float isr_freq_hz, float lpf_b0, float lpf_b1,
                                 float k, float gamma) {
  const float two_pi = 6.28318530718f;

  me->fn = grid_freq_hz;
  me->w_dash = two_pi * grid_freq_hz;
  me->wc = two_pi * grid_freq_hz;
  me->delta_t = 1.0f / isr_freq_hz;
  me->k = k;
  me->gamma = gamma;

  sogi_fll_coeff_calc(me);

  me->lpf.b0 = lpf_b0;
  me->lpf.b1 = lpf_b1;

  me->fo = 0.0f;
  me->theta = 0.0f;
  me->cosine = 1.0f;
  me->sine = 0.0f;
  me->ef2 = 0.0f;

  me->u[0] = me->u[1] = me->u[2] = 0.0f;
  me->osg_u[0] = me->osg_u[1] = me->osg_u[2] = 0.0f;
  me->osg_qu[0] = me->osg_qu[1] = me->osg_qu[2] = 0.0f;
  me->u_q[0] = me->u_q[1] = 0.0f;
  me->u_d[0] = me->u_d[1] = 0.0f;
  me->ylf[0] = me->ylf[1] = 0.0f;
  me->x3[0] = me->x3[1] = 0.0f;
}

// 复位内部状态 (不改变配置参数, 保留自适应频率 w_dash)
static inline void sogi_fll_reset(SogiFll *me) {
  me->u[0] = me->u[1] = me->u[2] = 0.0f;
  me->osg_u[0] = me->osg_u[1] = me->osg_u[2] = 0.0f;
  me->osg_qu[0] = me->osg_qu[1] = me->osg_qu[2] = 0.0f;
  me->u_q[0] = me->u_q[1] = 0.0f;
  me->u_d[0] = me->u_d[1] = 0.0f;
  me->ylf[0] = me->ylf[1] = 0.0f;
  me->ef2 = 0.0f;
  me->x3[0] = me->x3[1] = 0.0f;
  me->fo = 0.0f;
  me->theta = 0.0f;
  me->cosine = 1.0f;
  me->sine = 0.0f;
  sogi_fll_coeff_calc(me);  // 用当前自适应频率重算 SOGI 系数
}

// ======================= 单步运行 (ISR 每采样周期调用) =======================

static inline void sogi_fll_run(SogiFll *me, float ac_value) {
  const float two_pi = 6.28318530718f;

  me->u[0] = ac_value;

  // ---- SOGI-QSG: 同相输出 (α) ----
  me->osg_u[0] = me->osg.osg_b0 * (me->u[0] - me->u[2])
               + me->osg.osg_a1 * me->osg_u[1]
               + me->osg.osg_a2 * me->osg_u[2];
  me->osg_u[2] = me->osg_u[1];
  me->osg_u[1] = me->osg_u[0];

  // ---- SOGI-QSG: 正交输出 (β) ----
  me->osg_qu[0] = me->osg.osg_qb0 * me->u[0]
                + me->osg.osg_qb1 * me->u[1]
                + me->osg.osg_qb2 * me->u[2]
                + me->osg.osg_a1 * me->osg_qu[1]
                + me->osg.osg_a2 * me->osg_qu[2];
  me->osg_qu[2] = me->osg_qu[1];
  me->osg_qu[1] = me->osg_qu[0];

  me->u[2] = me->u[1];
  me->u[1] = me->u[0];

  // ---- Park 变换 (αβ → dq) ----
  me->u_q[0] = me->cosine * me->osg_u[0] + me->sine * me->osg_qu[0];
  me->u_d[0] = me->cosine * me->osg_qu[0] - me->sine * me->osg_u[0];

  // ---- 环路滤波器 (频率校正积分) ----
  me->ylf[0] = me->ylf[1] + me->lpf.b0 * me->u_q[0] + me->lpf.b1 * me->u_q[1];
  me->ylf[1] = me->ylf[0];
  me->u_q[1] = me->u_q[0];

  // ---- VCO: 频率 = 标称 + 滤波输出, 积分相位 ----
  me->fo = me->fn + me->ylf[0];
  me->theta += me->fo * me->delta_t * two_pi;

  if (me->theta > two_pi) {
    me->theta -= two_pi;
  }

  me->sine = sinf(me->theta);
  me->cosine = cosf(me->theta);

  // ---- FLL: 频率锁定环 — 用正交误差驱动频率自适应 ----
  //   ef2 = −(u − u_α)·u_β·γ·dt   (正交分量与误差相乘, 正比于频率偏差)
  me->ef2 = ((me->u[0] - me->osg_u[0]) * me->osg_qu[0])
            * me->gamma * me->delta_t * -1.0f;

  me->x3[0] = me->x3[1] + me->ef2;
  me->x3[1] = me->x3[0];

  me->w_dash = me->wc + me->x3[0];
  me->fn = me->w_dash / two_pi;

  // ---- 用自适应频率重算 SOGI 系数 (保持 90° 正交性, 不清零 FLL 积分器) ----
  sogi_fll_coeff_recalc(me);
}

#endif  // COMP_SOGI_FLL_H
