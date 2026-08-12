// 电机控制 — 增强型滑模观测器 (Enhanced Sliding Mode Observer for PMSM/BLDC)
//
// 来源: TI controlSUITE motor_control/libs/eSMO/v100 (esmopos.h)
// 翻译为 C-OOP 纯C float inline 版本
//
// 相比基础 SMO (comp_smo.h) 的改进:
//   - PLL 锁相环替代直接 arctan → 角度更平滑, 低转速更好
//   - 反电动势低通滤波 → 减少滑模抖振 (chattering)
//   - 速度直接由 PLL 输出 → 无需微分角度, 省去后处理
//
// 算法流程 (每控制周期 ISR 中):
//   1. 电流估计:   i_est += (dt/Ls) × (V - Rs×i_est - E_filt - Z)
//   2. 滑模控制:   Z = clamp(I_err × K_slide, ±1)     ← 饱和函数替代符号函数
//   3. 反电动势提取: E_filt += dt×ωc × (Z - E_filt)     ← 一阶 IIR 低通
//   4. PLL 鉴相:   ε = Eα_filt×cos(θ) - Eβ_filt×sin(θ)
//   5. PI → VCO:   积分 += Ki×ε×dt, ω = Kp×ε + 积分, θ += ω×dt
//   6. 角度归一化: θ ∈ [-π, π]
//
// 调用方式 (ISR 中每控制周期调用):
//   esmo_run(&obs, &cfg, v_alpha, v_beta, i_alpha, i_beta);
//   float theta = esmo_get_theta(&obs);
//   float speed = esmo_get_speed(&obs);

#ifndef COMP_ESMO_H
#define COMP_ESMO_H

#include <math.h>

#define ESMO_PI 3.14159265f

// ======================= EsmoCfg (eSMO 配置参数) =======================

typedef struct {
  float ls;           // 定子电感 (H)
  float rs;           // 定子电阻 (Ω)
  float dt;           // 控制周期 (s)
  float smo_gain;     // 滑模增益 (越大→切换越快, 但抖振越大)
  float pll_kp;       // PLL 比例增益
  float pll_ki;       // PLL 积分增益
  float bemf_fc;      // 反电动势滤波截止频率 (Hz)
} EsmoCfg;

// ======================= EsmoState (eSMO 运行状态) =======================

typedef struct {
  // 电流估计
  float i_alpha_est;        // 估计 α 轴电流
  float i_beta_est;         // 估计 β 轴电流

  // 电压 (输入缓存, 调试用)
  float v_alpha_filt;       // α 轴电压
  float v_beta_filt;        // β 轴电压

  // 滑模控制量 (开关信号 Z)
  float smo_alpha;          // α 轴滑模输出
  float smo_beta;           // β 轴滑模输出

  // 反电动势
  float bemf_alpha;         // α 轴反电动势 (滑模输出原始值)
  float bemf_beta;          // β 轴反电动势 (滑模输出原始值)
  float bemf_alpha_filt;    // α 轴反电动势 (低通滤波后)
  float bemf_beta_filt;     // β 轴反电动势 (低通滤波后)

  // PLL 锁相环
  float theta_pll;          // 输出: PLL 锁相角 (rad)
  float speed_pll;          // 输出: PLL 估算电角速度 (rad/s)
  float pll_integral;       // PLL 积分器状态
  float pll_err;            // PLL 鉴相误差 (调试用)
} EsmoState;

// 初始化 — 清零所有状态
// 在电机启动前调用, 确保观测器从零初始条件开始
static inline void esmo_init(EsmoState *me) {
  me->i_alpha_est = 0.0f;
  me->i_beta_est = 0.0f;
  me->v_alpha_filt = 0.0f;
  me->v_beta_filt = 0.0f;
  me->smo_alpha = 0.0f;
  me->smo_beta = 0.0f;
  me->bemf_alpha = 0.0f;
  me->bemf_beta = 0.0f;
  me->bemf_alpha_filt = 0.0f;
  me->bemf_beta_filt = 0.0f;
  me->theta_pll = 0.0f;
  me->speed_pll = 0.0f;
  me->pll_integral = 0.0f;
  me->pll_err = 0.0f;
}

// eSMO 单步运行 — 每控制周期在 ISR 中调用
//   v_alpha, v_beta: αβ 轴定子电压
//   i_alpha, i_beta: αβ 轴定子电流 (采样值)
//   更新内部状态 + theta_pll + speed_pll
static inline void esmo_run(EsmoState *me, const EsmoCfg *cfg,
                             float v_alpha, float v_beta,
                             float i_alpha, float i_beta) {
  // ---- 缓存输入电压 (调试用) ----
  me->v_alpha_filt = v_alpha;
  me->v_beta_filt = v_beta;

  // ---- 阶段 1: 电流估计 (基于电机 RL 模型, 欧拉前向离散化) ----
  // i_est(k+1) = i_est(k) + (dt/Ls) × (V - Rs×i_est - E_filt - Z)
  // 其中 E_filt 是滤波后的反电动势 (作为反馈补偿项)
  // Z 是滑模控制量 (迫使估计电流跟踪实际电流)
  float dt_over_ls = cfg->dt / cfg->ls;
  me->i_alpha_est += dt_over_ls * (v_alpha - cfg->rs * me->i_alpha_est
                                   - me->bemf_alpha_filt - me->smo_alpha);
  me->i_beta_est  += dt_over_ls * (v_beta  - cfg->rs * me->i_beta_est
                                   - me->bemf_beta_filt  - me->smo_beta);

  // ---- 阶段 2: 电流误差 ----
  // 误差 = 估计电流 - 实际电流
  // 作为滑模面的趋近律输入
  float i_err_alpha = me->i_alpha_est - i_alpha;
  float i_err_beta  = me->i_beta_est  - i_beta;

  // ---- 阶段 3: 滑模控制律 (饱和函数 sat, 抑制抖振) ----
  // Z = sat(I_err × K_slide, ±1)
  // 用饱和函数替代符号函数 sign(), 在边界层内线性, 边界层外饱和
  // 相比纯符号函数: 高频抖振大幅减少, 无需低通后处理

  // α 轴饱和
  float z_raw_alpha = i_err_alpha * cfg->smo_gain;
  if (z_raw_alpha > 1.0f) {
    me->smo_alpha = 1.0f;
  } else if (z_raw_alpha < -1.0f) {
    me->smo_alpha = -1.0f;
  } else {
    me->smo_alpha = z_raw_alpha;
  }

  // β 轴饱和
  float z_raw_beta = i_err_beta * cfg->smo_gain;
  if (z_raw_beta > 1.0f) {
    me->smo_beta = 1.0f;
  } else if (z_raw_beta < -1.0f) {
    me->smo_beta = -1.0f;
  } else {
    me->smo_beta = z_raw_beta;
  }

  // ---- 阶段 4: 反电动势提取 (一阶 IIR 低通滤波) ----
  // 从滑模开关信号 Z 中提取平滑的反电动势
  // E_filt(k+1) = E_filt(k) + dt × ωc × (Z(k) - E_filt(k))
  // ωc = 2π × fc (fc 为截止频率, 典型值 100~500 Hz)
  // 截止频率越高 → 响应越快但滤波越弱; 越低 → 越平滑但相位滞后越大
  float lpf_coeff = cfg->dt * 2.0f * ESMO_PI * cfg->bemf_fc;

  // 保存原始值 (调试用)
  me->bemf_alpha = me->smo_alpha;
  me->bemf_beta  = me->smo_beta;

  // 一阶 IIR 低通滤波
  me->bemf_alpha_filt += lpf_coeff * (me->smo_alpha - me->bemf_alpha_filt);
  me->bemf_beta_filt  += lpf_coeff * (me->smo_beta  - me->bemf_beta_filt);

  // ---- 阶段 5: PLL 锁相环角度/速度跟踪 ----
  // 鉴相器 (Phase Detector):
  //   ε = Eα_filt × cos(θ_pll) - Eβ_filt × sin(θ_pll)
  // 原理: 将滤波后的反电动势投影到估计旋转坐标系
  //       锁定时 ε → 0, θ_pll → 真实转子角度
  me->pll_err = me->bemf_alpha_filt * cosf(me->theta_pll)
              - me->bemf_beta_filt  * sinf(me->theta_pll);

  // PI 环路滤波器
  // 积分项累积稳态误差 → 消除静差
  // 比例项响应瞬时误差 → 提高动态响应
  me->pll_integral += cfg->pll_ki * me->pll_err * cfg->dt;
  me->speed_pll = cfg->pll_kp * me->pll_err + me->pll_integral;

  // VCO 积分 → 角度 (θ_pll = ∫ ω dt)
  me->theta_pll += me->speed_pll * cfg->dt;

  // ---- 阶段 6: 角度归一化到 [-π, π] ----
  // 防止角度无限增长导致浮点精度丢失
  while (me->theta_pll > ESMO_PI) {
    me->theta_pll -= 2.0f * ESMO_PI;
  }
  while (me->theta_pll < -ESMO_PI) {
    me->theta_pll += 2.0f * ESMO_PI;
  }
}

// 获取 PLL 锁相角 — 用于 FOC Park/InvPark 变换
static inline float esmo_get_theta(const EsmoState *me) {
  return me->theta_pll;
}

// 获取 PLL 估算电角速度 (rad/s) — 用于速度环反馈
// 注意: 这是电角速度, 机械角速度需除以极对数
static inline float esmo_get_speed(const EsmoState *me) {
  return me->speed_pll;
}

#endif  // COMP_ESMO_H
