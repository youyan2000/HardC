// DDSRF-PLL —— PllBase 子类 (解耦双同步旋转坐标系, 正负序分离)
//
// 算法链: 外部输入正负序 dq → 交叉解耦(消除负序干扰) → 4 路 LPF → 正序 q 轴(锁相误差)
//        → 基类 LF(PI) → 基类 VCO
// 同构于旧 comp_pll.h 的 DdsrfPll。
// 优势: 电网不平衡时仍能精确锁相 (分离正负序, 消除负序 2 倍频扰动)。
// 适用: 三相并网逆变器在不平衡/谐波电网下的鲁棒锁相。
//
// 输入帧 (PllInput): 使用 d_p/d_n/q_p/q_n (需外部 Park 变换到正负序 dq)。

#ifndef PLL_DDSRF_H
#define PLL_DDSRF_H

#include "comp_pll_base.h"

// ======== 配置 POD ========
typedef struct {
  float lpf_fc;   // 解耦网络 4 路 LPF 截止频率 (Hz, 默认 50)
} PllDdsrfCfg;

// ======== 子类结构体 —— 基类第一成员 ========
typedef struct {
  PllBase base;   // 基类 (含 LF+VCO)

  PllDdsrfCfg cfg;  // 配置

  // 输入正负序 dq (外部填入)
  float d_p, d_n, q_p, q_n;

  // 解耦后 dq
  float d_p_decoupl, d_n_decoupl;
  float q_p_decoupl, q_n_decoupl;

  // 2 倍频项
  float cos_2theta, sin_2theta;

  // 4 路 LPF 状态
  float y[2];   // d_p
  float x[2];   // q_p
  float w[2];   // d_n
  float z[2];   // q_n

  // 解耦 LPF 输出
  float d_p_lpf, d_n_lpf, q_p_lpf, q_n_lpf;

  // LPF 系数
  float k1, k2;
} PllDdsrf;

// ======== 构造 ========
//   grid_freq_hz: 标称频率
//   dt:            控制周期 (s)
//   kp, ki:        LF 环路滤波 PI 增益
void pll_ddsrf_init(PllDdsrf *me, float grid_freq_hz, float dt,
                    float kp, float ki);

#endif  // PLL_DDSRF_H
