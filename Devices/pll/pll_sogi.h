// SOGI-PLL —— PllBase 子类 (SOGI 正交积分器 + Park 乘法投影鉴相)
//
// 算法链: SOGI-QSG(单相→αβ 正交) → Park 投影(取 q 轴 = 锁相误差) → 基类 LF(PI) → 基类 VCO
// 同构于旧 comp_pll.h 的 SogiPll; 也是 SSRF-SPLL (SOGI-QSG + 同步旋转坐标系) 的 OOP 变体。
// 特性: 单相高性能, 电网谐波畸变下比纯乘法 Notch 鉴相精度高。
// 适用: 单相并网逆变器 / UPS / APF 的电网同步。

#ifndef PLL_SOGI_H
#define PLL_SOGI_H

#include "comp_pll_base.h"

// ======== 配置 POD ========
typedef struct {
  float k_damp;   // SOGI 阻尼系数 (典型 √2 ≈ 1.4142)
} PllSogiCfg;

// ======== 子类结构体 —— 基类第一成员 (container_of 依赖) ========
typedef struct {
  PllBase base;   // 基类 (含 LF+VCO 字段)

  PllSogiCfg cfg; // 配置

  // SOGI-QSG 双线性系数
  float osg_b0, osg_b2;  // 同相 IIR 分子 (b2 = -b0)
  float osg_a1, osg_a2;  // 同相/正交共用分母
  float osg_qb0, osg_qb1, osg_qb2;  // 正交 IIR 分子

  // SOGI-QSG 运行时状态
  float u[3];         // 输入电压历史 [k, k-1, k-2]
  float osg_u[3];     // 同相输出 (滤波后 α)
  float osg_qu[3];    // 正交输出 (90° 滞后 β)

  float u_q;          // 锁相误差 v_q (本拍)
} PllSogi;

// ======== 构造 ========
//   grid_freq_hz: 标称电网频率 (50/60)
//   dt:            控制周期 (s)
//   kp, ki:        LF 环路滤波 PI 增益
//   k_damp:        SOGI 阻尼系数 (0.5~2.0, 默认 √2)
void pll_sogi_init(PllSogi *me, float grid_freq_hz, float dt,
                   float kp, float ki, float k_damp);

// 按当前标称频率重算 SOGI-QSG 双线性系数 (电网频率变化后调用)
void pll_sogi_coeff_update(PllSogi *me);

#endif  // PLL_SOGI_H
