// 软件锁相环库 — SOGI-PLL (单相) / SRF-PLL (三相)
//
// 来源: TI controlSUITE solar/v1.2/float (SPLL_1ph_SOGI_F, SPLL_3ph_SRF_F)
// 翻译为 HardC 纯C float 版本 (去掉 TI 定点依赖, 保留算法核心)
//
// 应用场景:
//   SOGI-PLL — 单相并网逆变器, UPS, APF 的电网同步
//   SRF-PLL — 三相并网逆变器, 电机矢量控制的 dq 定向
//
// 调用方式 (ISR 中每控制周期调用一次):
//   sogi_pll_run(&pll, v_grid);           // 单相: 输入电网电压, 输出锁相角+频率
//   srf_pll_run_ab(&pll, v_alpha, v_beta, &sin, &cos); // 三相: 先 Clarke 变换, 再 PLL

#ifndef COMP_PLL_H
#define COMP_PLL_H

#include <math.h>
#include <stdbool.h>
#include "comp_math.h"

// ======================= SOGI-PLL (单相正交积分器锁相环) =======================

// SOGI-QSG 系数 — 二阶广义积分器正交信号发生器
// 将单相输入信号 u 分解为正交双相 u_α (同相) + u_β (滞后90°)
typedef struct {
  float b0;       // 同相通道分子 b0
  float b2;       // 同相通道分子 b2
  float a1;       // 共用分母 a1
  float a2;       // 共用分母 a2
  float qb0;      // 正交通道分子 b0
  float qb1;      // 正交通道分子 b1
  float qb2;      // 正交通道分子 b2
  float k;        // 阻尼系数 (默认 √2 ≈ 1.4142)
} SogiOsgCoeff;

// 环路滤波器系数 — 一阶 IIR 低通 (PI 等效)
typedef struct {
  float b1;       // 分子 B1 (Z^-1)
  float b0;       // 分子 B0 (Z^0)
  float a1;       // 分母 A1 (Z^-1)
} SogiLpfCoeff;

// SOGI-PLL 主结构体
typedef struct {
  // 输入历史
  float u[3];           // 电网电压输入 [k, k-1, k-2]

  // SOGI-QSG 状态
  float osg_u[3];       // 同相输出 (滤波后的 α)
  float osg_qu[3];      // 正交输出 (90°滞后的 β)
  float u_d;            // d 轴分量 (电网电压幅值)
  float u_q[2];         // q 轴分量 (锁相误差) [k, k-1]

  // 环路滤波器
  float ylf[2];         // 滤波器输出 [k, k-1]

  // 压控振荡器 VCO
  float fo;             // 输出: 估算频率 (Hz)
  float fn;             // 参数: 标称频率 (Hz, 如 50.0)
  float theta[2];       // 输出: 锁相角 (rad) [k, k-1]
  float sin_val;        // 输出: sin(θ)
  float cos_val;        // 输出: cos(θ)

  // 配置
  float delta_t;        // 采样周期 (s)
  SogiOsgCoeff osg;     // SOGI 系数
  SogiLpfCoeff lpf;     // 环路滤波器系数
} SogiPll;

// 初始化 SOGI-PLL
//   grid_freq_hz: 标称电网频率 (50 或 60)
//   dt:           控制周期 (s), 如 1e-4 = 100µs
//   kp, ki:       环路滤波器 PI 增益
//   k_damp:       SOGI 阻尼系数 (0.5~2.0, 默认 √2)
void sogi_pll_init(SogiPll *me, float grid_freq_hz, float dt,
                   float kp, float ki, float k_damp);

// SOGI-PLL 单步运行 (ISR 中每控制周期调用)
//   v_grid: 当前电网电压采样值
//   返回:    估算频率 fo (Hz)
float sogi_pll_run(SogiPll *me, float v_grid);

// 重置 PLL 状态 (故障恢复 / 重新启动时调用)
void sogi_pll_reset(SogiPll *me);

// ======================= SRF-PLL (三相同步旋转坐标系锁相环) =======================

// SRF-PLL 主结构体
// 使用者需先做 Clarke 变换得到 v_alpha/v_beta,
// 再调 srf_pll_run_ab(&pll, v_alpha, v_beta, &sin, &cos)
typedef struct {
  // q 轴 (Park 变换结果, 外部填入或内部计算)
  float v_q[2];         // q 轴电压 [k, k-1] — 锁相误差

  // 环路滤波器
  float ylf[2];         // 滤波器输出 [k, k-1]
  float b0_lf;          // 滤波器系数 B0
  float b1_lf;          // 滤波器系数 B1

  // 压控振荡器 VCO
  float fo;             // 输出: 估算频率 (Hz)
  float fn;             // 参数: 标称频率 (Hz)
  float theta[2];       // 输出: 锁相角 (rad) [k, k-1]

  // 配置
  float delta_t;        // 采样周期 (s)
  float freq_lim;       // 频率偏差限幅 (Hz, 默认 ±200)
} SrfPll;

// 初始化 SRF-PLL
//   grid_freq_hz: 标称电网频率
//   dt:           控制周期 (s)
//   kp, ki:       环路滤波器 PI 增益
void srf_pll_init(SrfPll *me, float grid_freq_hz, float dt,
                  float kp, float ki);

// SRF-PLL 单步运行 — v_α/v_β 版本 (内置 Park 变换)
//   v_alpha, v_beta: Clarke 变换后的 αβ 电压
//   out_sin, out_cos: 输出 sin(θ)/cos(θ), 供外部 Park 变换使用
//   返回: 估算频率 fo (Hz)
float srf_pll_run_ab(SrfPll *me, float v_alpha, float v_beta,
                     float *out_sin, float *out_cos);

// SRF-PLL 单步运行 — v_q 版本 (Park 变换由外部完成, 只做环路滤波+VCO)
//   v_q: Park 变换后的 q 轴分量 (应为 0 表示锁相)
//   返回: 估算频率 fo (Hz)
float srf_pll_run_q(SrfPll *me, float v_q);

// 重置 PLL 状态
void srf_pll_reset(SrfPll *me);

// ======================= 陷波型单相 PLL (非 SOGI, 适合轻量应用) =======================

// 来源: TI controlSUITE SPLL_1ph_F (solar/v1.2/float)
// 机制: 积型鉴相器 (AC_input × cos) → 陷波滤波器 (2×f0 纹波抑制) → PI → VCO
// 优势: 比 SOGI-PLL 省 CPU 和内存 (无需正交发生器)
// 劣势: 谐波畸变时锁相精度不如 SOGI

typedef struct {
  // 鉴相器
  float ac_input;               // 输入: 电网电压采样值
  float upd[3];                 // 鉴相器输出历史 [k, k-1, k-2]

  // 陷波滤波器 (抑制 2 倍频纹波)
  float ynotch[3];              // 陷波输出历史
  float notch_b2, notch_b1, notch_b0;  // 陷波分子
  float notch_a2, notch_a1;            // 陷波分母

  // 环路滤波器
  float ylf[2];                 // LPF 输出 [k, k-1]
  float lpf_b1, lpf_b0;         // LPF 分子
  float lpf_a1;                 // LPF 分母

  // VCO — 精确离散时间振荡器 (比欧拉积分法更精确)
  float sin_val[2];             // sin(θ) [k, k-1]
  float cos_val[2];             // cos(θ) [k, k-1]
  float theta[2];               // 相位 (标幺, 0~1)
  float wo;                     // 输出: 角频率 (rad/s)
  float wn;                     // 参数: 标称角频率 (rad/s)

  // 配置
  float delta_t;                // 采样周期 (s)
} NotchPll;

// 初始化陷波型 PLL
//   grid_freq_hz: 标称电网频率 (50 或 60)
//   dt:           控制周期 (s)
//   kp, ki:       环路滤波器 PI 增益
void notch_pll_init(NotchPll *me, float grid_freq_hz, float dt,
                    float kp, float ki);

// 陷波型 PLL 单步运行
//   返回: 当前锁相频率 fo (Hz)
float notch_pll_run(NotchPll *me, float v_grid);

// 陷波系数更新 (电网频率变化后调用)
void notch_pll_coeff_update(NotchPll *me, float wn);

// 重置
void notch_pll_reset(NotchPll *me);

// ======================= DDSRF-PLL (解耦双同步旋转坐标系 PLL) =======================

// 来源: TI controlSUITE SPLL_3ph_DDSRF_F (solar/v1.2/float)
// 机制: 正序+负序双 dq 解耦 → 交叉耦合补偿 → 正序 q 轴 → PI → VCO
// 优势: 电网不平衡时仍能精确锁相 (分离正负序)
// 应用: 三相并网逆变器在不平衡/谐波电网下的鲁棒锁相

typedef struct {
  // 正负序 dq 分量 (外部 Park 变换填入)
  float d_p;                    // 输入: 正序 d 轴
  float d_n;                    // 输入: 负序 d 轴
  float q_p;                    // 输入: 正序 q 轴
  float q_n;                    // 输入: 负序 q 轴

  // 解耦后的 dq 分量
  float d_p_decoupl;            // 正序 d (解耦后)
  float d_n_decoupl;            // 负序 d (解耦后)
  float q_p_decoupl;            // 正序 q (解耦后)
  float q_n_decoupl;            // 负序 q (解耦后)

  // 2 倍频分量
  float cos_2theta;             // cos(2θ)
  float sin_2theta;             // sin(2θ)

  // 4 路 LPF 状态 (对 d_p, q_p, d_n, q_n 分别低通)
  float y[2];                   // d_p LPF 状态 [k, k-1]
  float x[2];                   // q_p LPF 状态
  float w[2];                   // d_n LPF 状态
  float z[2];                   // q_n LPF 状态

  // 解耦 LPF 输出
  float d_p_lpf;                // d_p 低通输出
  float d_n_lpf;                // d_n 低通输出
  float q_p_lpf;                // q_p 低通输出
  float q_n_lpf;                // q_n 低通输出

  // LPF 系数
  float k1;                     // LPF 输入系数
  float k2;                     // LPF 反馈系数

  // 环路滤波器 + VCO
  float v_q[2];                 // 误差信号 [k, k-1] (正序 q 轴)
  float ylf[2];                 // 环路滤波器输出 [k, k-1]
  float b0_lf;                  // PI 离散化 B0 系数
  float b1_lf;                  // PI 离散化 B1 系数
  float fo;                     // 输出: 估算频率
  float fn;                     // 参数: 标称频率
  float theta[2];               // 输出: 锁相角 (rad) [k, k-1]
  float delta_t;                // 采样周期 (s)
} DdsrfPll;

// 初始化 DDSRF-PLL
//   grid_freq_hz: 标称频率
//   dt:           控制周期 (s)
//   kp, ki:       环路滤波器 PI 增益
void ddsrf_pll_init(DdsrfPll *me, float grid_freq_hz, float dt,
                    float kp, float ki);

// DDSRF-PLL 单步运行 — 调用前需外部计算 sin(θ)/cos(θ) 和 d_p/d_n/q_p/q_n
//   返回: 估算频率 fo (Hz)
float ddsrf_pll_run(DdsrfPll *me, float d_p, float d_n, float q_p, float q_n);

// 重置
void ddsrf_pll_reset(DdsrfPll *me);

#endif  // COMP_PLL_H
