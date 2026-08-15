// SRF-PLL —— PllBase 子类 (同步旋转坐标系, Park 乘法投影鉴相)
//
// 算法链: Clarke(外部)→ αβ → Park 旋转投影(取 q 轴 = 锁相误差) → 基类 LF(PI) → 基类 VCO
// 同构于旧 comp_pll.h 的 SrfPll。
// 特性: 三相标准锁相, dq 定向。
// 适用: 三相并网逆变器 / 电机矢量控制的 dq 定向。
//
// 输入帧 (PllInput): 使用 v_alpha / v_beta (需先用 Clarke 变换得到 αβ)。

#ifndef PLL_SRF_H
#define PLL_SRF_H

#include "comp_pll_base.h"

// ======== 配置 POD ========
typedef struct {
  float freq_lim;   // 频率偏差限幅 (±Hz), 0=不限, 典型 ±200
} PllSrfCfg;

// ======== 子类结构体 —— 基类第一成员 ========
typedef struct {
  PllBase base;   // 基类 (含 LF+VCO)

  PllSrfCfg cfg;  // 配置

  float v_q;      // 本拍锁相误差 (Park q 轴)
} PllSrf;

// ======== 构造 ========
//   grid_freq_hz: 标称电网频率
//   dt:            控制周期 (s)
//   kp, ki:        LF 环路滤波 PI 增益
void pll_srf_init(PllSrf *me, float grid_freq_hz, float dt,
                  float kp, float ki);

#endif  // PLL_SRF_H
