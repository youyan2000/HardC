// 电机控制 — 滑模观测器 (Sliding Mode Observer for PMSM)
//
// 来源: TI controlSUITE motor_control/math_blocks/v4.3 (smopos.h, smopos_const.h)
// 翻译为 HardC 纯C float inline 版本
//
// 算法:
//   I_est(k+1) = Fsmopos × I_est(k) + Gsmopos × (V - E - Z)
//   I_error = I_est - I_real
//   Z = sat(I_error, ±E0) × 2 × Kslide     ← 滑模控制律
//   E(k+1) = E(k) + Kslf × (Z(k) - E(k))   ← 反电动势提取 (一阶 LPF)
//   θ = atan2(-Eα, Eβ)                      ← 转子角度
//
// 调用方式 (ISR 中每控制周期调用):
//   smo_run(&obs, v_alpha, v_beta, i_alpha, i_beta);
//   float theta = obs.theta;

#ifndef COMP_SMO_H
#define COMP_SMO_H

#include <math.h>

// ======================= SmoObs (滑模观测器) =======================

typedef struct {
  // 输入
  float v_alpha;          // 输入: α 轴定子电压 (标幺)
  float v_beta;           // 输入: β 轴定子电压 (标幺)
  float i_alpha;          // 输入: α 轴定子电流 (标幺)
  float i_beta;           // 输入: β 轴定子电流 (标幺)

  // 估计状态
  float est_i_alpha;      // 估计 α 轴电流
  float est_i_beta;       // 估计 β 轴电流
  float i_alpha_error;    // α 轴电流误差 = 估计 - 实际
  float i_beta_error;     // β 轴电流误差

  // 滑模控制
  float z_alpha;          // 输出: α 轴滑模控制量 (开关信号)
  float z_beta;           // 输出: β 轴滑模控制量

  // 反电动势 (从 Z 中低通滤波提取)
  float e_alpha;          // α 轴反电动势
  float e_beta;           // β 轴反电动势

  // 输出
  float theta;            // 输出: 转子角度 (rad)

  // 参数
  float f_smopos;         // 电机 RL 离散化系数 F = exp(-Rs/Ls * Ts)
  float g_smopos;         // 电机 RL 离散化系数 G = (Vb/Ib)*(1/Rs)*(1-F)
  float kslide;           // 滑模增益 (越大→切换越快, 但抖振越大)
  float kslf;             // 滑模滤波器增益 (一阶 LPF 截止频率系数)
  float e0;               // 饱和边界 (±E0, 通常 0.5)
} SmoObs;

#define SMO_OBS_DEFAULTS { 0,0,0,0, 0,0,0,0, 0,0, 0,0, 0, 0,0,0,0, 0.5f }

// 初始化 — 电机参数和采样周期
//   rs:      定子电阻 (Ω)
//   ls:      定子电感 (H)
//   ib:      电流基值 (A)
//   vb:      电压基值 (V)
//   ts:      采样周期 (s)
//   kslide:  滑模增益
//   kslf:    反电动势滤波器增益 (更小→更平滑, 更慢)
static inline void smo_obs_init(SmoObs *me, float rs, float ls,
                                 float ib, float vb, float ts,
                                 float kslide, float kslf) {
  // 连续时间 RL: L di/dt = V - Ri - E
  // 离散化: i(k+1) = F×i(k) + G×(V(k) - E(k) - Z(k))
  me->f_smopos = expf((-rs / ls) * ts);
  me->g_smopos = (vb / ib) * (1.0f / rs) * (1.0f - me->f_smopos);
  me->kslide = kslide;
  me->kslf = kslf;
  me->e0 = 0.5f;

  // 初始化状态
  me->est_i_alpha = 0.0f;
  me->est_i_beta = 0.0f;
  me->e_alpha = 0.0f;
  me->e_beta = 0.0f;
  me->z_alpha = 0.0f;
  me->z_beta = 0.0f;
  me->theta = 0.0f;
}

// SMO 单步运行 — 每控制周期在 ISR 中调用
static inline float smo_obs_run(SmoObs *me, float v_alpha, float v_beta,
                                 float i_alpha, float i_beta) {
  me->v_alpha = v_alpha;
  me->v_beta = v_beta;
  me->i_alpha = i_alpha;
  me->i_beta = i_beta;

  // ---- 阶段 1: 滑模电流观测器 ----
  // I_est(k+1) = F × I_est(k) + G × (V(k) - E(k) - Z(k))
  me->est_i_alpha = me->f_smopos * me->est_i_alpha
                  + me->g_smopos * (me->v_alpha - me->e_alpha - me->z_alpha);
  me->est_i_beta  = me->f_smopos * me->est_i_beta
                  + me->g_smopos * (me->v_beta  - me->e_beta  - me->z_beta);

  // ---- 阶段 2: 电流误差 ----
  me->i_alpha_error = me->est_i_alpha - me->i_alpha;
  me->i_beta_error  = me->est_i_beta  - me->i_beta;

  // ---- 阶段 3: 滑模控制律 (减函数, E0=0.5 为边界) ----
  // Z = sat(I_error, +E0, -E0) × 2 × Kslide
  float e0 = me->e0;

  // α 轴
  float d_alpha;
  if (me->i_alpha_error > e0) {
    d_alpha = e0;
  } else if (me->i_alpha_error < -e0) {
    d_alpha = -e0;
  } else {
    d_alpha = me->i_alpha_error;
  }
  me->z_alpha = d_alpha * 2.0f * me->kslide;

  // β 轴
  float d_beta;
  if (me->i_beta_error > e0) {
    d_beta = e0;
  } else if (me->i_beta_error < -e0) {
    d_beta = -e0;
  } else {
    d_beta = me->i_beta_error;
  }
  me->z_beta = d_beta * 2.0f * me->kslide;

  // ---- 阶段 4: 滑模控制滤波 → 反电动势提取 ----
  // E(k+1) = E(k) + Kslf × (Z(k) - E(k))  — 一阶低通滤波器
  me->e_alpha += me->kslf * (me->z_alpha - me->e_alpha);
  me->e_beta  += me->kslf * (me->z_beta  - me->e_beta);

  // ---- 阶段 5: 转子角度计算 ----
  // θ = atan2(-Eα, Eβ)
  me->theta = atan2f(-me->e_alpha, me->e_beta);

  return me->theta;
}

// 重置观测器
static inline void smo_obs_reset(SmoObs *me) {
  me->est_i_alpha = 0.0f;
  me->est_i_beta = 0.0f;
  me->e_alpha = 0.0f;
  me->e_beta = 0.0f;
  me->z_alpha = 0.0f;
  me->z_beta = 0.0f;
  me->theta = 0.0f;
}

#endif  // COMP_SMO_H
