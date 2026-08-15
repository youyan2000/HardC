// SOGI-FLL —— PllBase 子类 (SOGI 正交 + Park 投影鉴相 + FLL 频率自适)
//
// 算法链: SOGI-QSG(单相→αβ) → Park 投影(取 q 轴 = 锁相误差) → 基类 LF(PI) → 基类 VCO
//        + FLL(query-error): ef2 = −(u−u_α)·u_β·γ·dt 驱动频率积分器, 实时自适应电网频率
// 同构于旧 comp_sogi_fll.h 的 SogiFll。
// 与 PllSogi (SOGI-PLL 固定标称频率) 的区别: 增加频率锁定环 FLL, w_dash 随电网漂移,
// 并每拍用自适应频率重算 SOGI 双线性系数。
// 适用: 弱电网/频率漂移场景 (柴油发电机、微电网、变速发电机接口)。
//
// 输入帧 (PllInput): 使用 v (归一化到标幺 pu 的单相电压)。

#ifndef PLL_SOGI_FLL_H
#define PLL_SOGI_FLL_H

#include "comp_pll_base.h"

// ======== 配置 POD ========
typedef struct {
  float k_damp;   // SOGI 阻尼系数 (典型 √2)
  float gamma;    // FLL 收敛增益 (γ, 决定频率跟踪速度)
} PllSogiFllCfg;

// ======== 子类结构体 —— 基类第一成员 ========
typedef struct {
  PllBase base;   // 基类 (含 LF+VCO; fn 会被 FLL 自适应更新)

  PllSogiFllCfg cfg;  // 配置

  // SOGI-QSG 双线性系数
  float osg_b0, osg_b2;
  float osg_a1, osg_a2;
  float osg_qb0, osg_qb1, osg_qb2;

  // SOGI-QSG 运行时状态
  float u[3];
  float osg_u[3];
  float osg_qu[3];

  float u_q;    // 锁相误差 v_q (本拍)
  float u_d;    // d 轴分量 (幅值估计)

  // FLL 状态
  float wc;           // 标称中心角频率 (rad/s) = 2π·grid_freq_hz (固定基准)
  float ef2;          // FLL 误差项
  float x3[2];        // FLL 频率积分器状态
  float w_dash;       // 自适应角频率 (rad/s) = wc + x3
} PllSogiFll;

// ======== 构造 ========
//   grid_freq_hz: 标称电网频率 (50/60)
//   isr_freq_hz:  采样频率 (Hz) = 1/dt
//   lpf_b0, lpf_b1: LF(Tustin) 分子系数 —— 由外部按带宽设计
//   k_damp:       SOGI 阻尼系数 (典型 √2)
//   gamma:        FLL 增益 (频率跟踪收敛速度)
void pll_sogi_fll_init(PllSogiFll *me, float grid_freq_hz, float isr_freq_hz,
                       float lpf_b0, float lpf_b1, float k_damp, float gamma);

#endif  // PLL_SOGI_FLL_H
