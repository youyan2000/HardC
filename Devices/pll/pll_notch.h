// Notch-PLL —— PllBase 子类 (纯乘法累积型鉴相 + 陷波滤 2f0 + 精确离散 VCO)
//
// 算法链: 积型鉴相器(v×cos → 含 2f0 纹波) → 陷波滤波器(2f0 抑制) → 基类 LF(PI) → 基类 VCO
// 同构于旧 comp_pll.h 的 NotchPll。
// 优势: 比 SOGI-PLL 省 CPU 和内存 (无需正交发生器)。
// 劣势: 谐波畸变时锁相精度不如 SOGI。
// 适用: 轻量单相应用。
//
// 输入帧 (PllInput): 使用 v (单相电压采样)。

#ifndef PLL_NOTCH_H
#define PLL_NOTCH_H

#include "comp_pll_base.h"

// ======== 配置 POD ========
typedef struct {
  float notch_depth;  // 陷波深度阻尼 (默认 0.01)
  float notch_bw;     // 陷波带宽阻尼 (默认 0.1)
} PllNotchCfg;

// ======== 子类结构体 —— 基类第一成员 ========
typedef struct {
  PllBase base;   // 基类 (含 LF+VCO)

  PllNotchCfg cfg;  // 配置

  // 积型鉴相器输出历史
  float upd[3];      // [k, k-1, k-2]

  // 陷波滤波器 (2f0 纹波抑制) 状态 + 系数
  float ynotch[3];   // 陷波输出历史
  float notch_b2, notch_b1, notch_b0;   // 陷波分子
  float notch_a2, notch_a1;             // 陷波分母

  float out_ynotch;  // 陷波输出 (本拍送入 LF)
  float wo;          // 当前角频率 (rad/s)
} PllNotch;

// ======== 构造 ========
//   grid_freq_hz: 标称电网频率 (50/60)
//   dt:            控制周期 (s)
//   kp, ki:        LF 环路滤波 PI 增益
void pll_notch_init(PllNotch *me, float grid_freq_hz, float dt,
                    float kp, float ki);

// 按新中心角频率重算陷波系数 (电网频率变化后调用; wn 单位 rad/s)
void pll_notch_coeff_update(PllNotch *me, float wn);

#endif  // PLL_NOTCH_H
