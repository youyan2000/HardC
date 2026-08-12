// 电机控制 — 高频注入 (High Frequency Injection for PMSM 低速/零速无感控制)
//
// 来源: TI controlSUITE motor_control/libs/HFI
// 翻译为 C-OOP 纯C float static inline 版本
//
// 原理:
//   脉振高频正弦电压注入 (Pulsating HF Injection):
//   在估计 d 轴注入高频余弦电压 v_h·cos(ω_h·t), 利用转子
//   结构凸极性 (saliency) 在 q 轴电流中感应出包含位置误差
//   信息的高频分量。通过同步解调 + PLL 提取转子角度。
//
//   适用: IPMSM (内嵌式永磁), 凸极比 Lq/Ld > 1.5
//   不适用: SPMSM (表贴式, 凸极比 ≈ 1), 除非有饱和凸极性
//
// 算法流程 (每控制周期 ISR 中):
//   1. hfi_inject():    DDS 余弦注入 → v_d_inj (叠加到电流环 PI 输出)
//   2. 电流环 PI 执行后, 采样 i_q 实际值
//   3. hfi_demodulate(): BPF (HPF级联LPF) → 同步解调 → LPF → PLL → θ, ω
//      a) BPF 带通滤波: 提取 f_h 附近的分量, 滤除基频和 PWM 谐波
//         - HPF (截止 f_h/2): bpf_hp = α·(bpf_hp_prev + i_q - i_q_prev)
//         - LPF (截止 f_h):   i_q_bpf = bpf_lp_prev + bp_k·(bpf_hp - bpf_lp_prev)
//      b) 解调: ε_raw = i_q_bpf · sin(θ_inj)  ← 同源 DDS 相位
//      c) LPF:  pos_err += lpf_k · (ε_raw - pos_err)
//      d) PLL:  PI + VCO → theta, speed
//      e) 角度归一化到 [-π, π]
//
// 使用要点:
//   - 仅低速/零速使用 (通常 < 5% 额定转速)
//   - 中高速切换至 eSMO/滑模观测器
//   - v_h 需权衡: 太小 SNR 差, 太大转矩脉动/噪声
//   - 切换策略: 混合/滞后/加权过渡 (避免角度跳变)
//
// 调用方式 (ISR 中每控制周期):
//   hfi_inject(&hfi, &cfg);                    // 1. 计算注入电压
//   v_d_ref = pi_d_output + hfi.v_d_inj;       // 2. 叠加到 d 轴电压
//   // ... FOC 电流环 + SVPWM 输出 ...
//   hfi_demodulate(&hfi, &cfg, i_q_measured);  // 3. 解调提取角度
//   float theta = hfi_get_theta(&hfi);
//   float speed = hfi_get_speed(&hfi);

#ifndef COMP_HFI_H
#define COMP_HFI_H

#include <math.h>
#include <stdbool.h>

#define HFI_PI  3.14159265f
#define HFI_2PI 6.28318531f

// ======================= HfiCfg (HFI 配置参数) =======================
// 不可变参数 — 编译期或 YAML 注入填充
// 调用 hfi_cfg_default() 填充安全默认值后按需覆盖

typedef struct {
  float dt;         // ISR 周期 (s)
  float v_h;        // 注入电压幅值 (pu, 典型 0.02~0.05)
  float f_h;        // 注入频率 (Hz, 典型 500~2000)
  float bp_k;       // BPF 带通滤波系数 (LPF段: k = 2π·fc·dt, fc 建议 f_h)
  float lpf_k;      // 解调 LPF 系数 (fc 典型 50~200 Hz)
  float pll_kp;     // PLL 比例增益
  float pll_ki;     // PLL 积分增益
} HfiCfg;

// ======================= HfiState (HFI 运行时状态) =======================
// 调用 hfi_init() 清零所有字段

typedef struct {
  float inject_phase;   // DDS 相位累加器 (rad), 折叠在 [0, 2π)
  float v_d_inj;        // 输出: d 轴注入电压 (叠加到电流环 PI 输出)
  float i_q_bpf;        // BPF 输出: 带通滤波后的 q 轴电流
  float demod_raw;      // 解调原始值 ε_raw = i_q_bpf · sin(phase)
  float pos_err;        // 位置误差 (LPF 解调后的 DC 分量)
  float theta;          // 估计转子角度 (rad), PLL 输出
  float speed;          // 估计电角速度 (rad/s), PLL 输出
  float pll_integral;   // PLL PI 积分器状态
  float bpf_hp_prev;    // BPF 高通段前一拍输出 (滤波器历史)
  float bpf_lp_prev;    // BPF 低通段前一拍输出 (滤波器历史)
  float i_q_prev;       // 前一拍 q 轴电流 (HPF 差分用)
  bool  enabled;        // true=注入激活中
} HfiState;

// ======================= API 函数 =======================

// 初始化 — 清零所有运行时状态
// 电机启动前调用, 确保滤波器/PLL 从零初始条件开始
static inline void hfi_init(HfiState *me) {
  me->inject_phase = 0.0f;
  me->v_d_inj = 0.0f;
  me->i_q_bpf = 0.0f;
  me->demod_raw = 0.0f;
  me->pos_err = 0.0f;
  me->theta = 0.0f;
  me->speed = 0.0f;
  me->pll_integral = 0.0f;
  me->bpf_hp_prev = 0.0f;
  me->bpf_lp_prev = 0.0f;
  me->i_q_prev = 0.0f;
  me->enabled = false;
}

// 填充安全默认值 — 基于 dt 计算所有系数
// v_h=0.03 (3% 注入), f_h=1000 Hz, LPF 100 Hz
// PLL 带宽约 40 Hz (需根据电机凸极比微调)
static inline void hfi_cfg_default(HfiCfg *cfg, float dt) {
  cfg->dt = dt;
  cfg->v_h = 0.03f;
  cfg->f_h = 1000.0f;
  // BPF 系数 (LPF 段, 截止频率 = f_h): k = 2π · fc · dt
  cfg->bp_k = HFI_2PI * cfg->f_h * dt;
  // 解调 LPF 系数 (截止频率 = 100 Hz): k = 2π · fc · dt
  cfg->lpf_k = HFI_2PI * 100.0f * dt;
  // PLL 增益 (自然频率 ωn ≈ 2π·40 Hz)
  cfg->pll_kp = 2.0f;
  cfg->pll_ki = 80.0f;
}

// DDS 正弦注入 — 每控制周期在电流环 PI 之前调用
// 更新 me->v_d_inj, 叠加到 d 轴电流环 PI 输出
static inline void hfi_inject(HfiState *me, const HfiCfg *cfg) {
  if (!me->enabled) {
    me->v_d_inj = 0.0f;
    return;
  }

  // 阶段 1: DDS 相位累加 — Δθ = 2π · f_h · dt
  me->inject_phase += HFI_2PI * cfg->f_h * cfg->dt;

  // 阶段 2: 相位折叠到 [0, 2π) — 防止浮点无限增长
  while (me->inject_phase >= HFI_2PI) {
    me->inject_phase -= HFI_2PI;
  }
  while (me->inject_phase < 0.0f) {
    me->inject_phase += HFI_2PI;
  }

  // 阶段 3: 余弦注入 — v_d_inj = v_h · cos(θ_inj)
  // 脉振高频注入在估计 d 轴, 产生交变磁场探测凸极性
  me->v_d_inj = cfg->v_h * cosf(me->inject_phase);
}

// 解调 + PLL — 每控制周期在电流采样后调用
// i_q: q 轴电流采样值 (来自 Clarke/Park 变换)
// 更新 me->theta 和 me->speed
static inline void hfi_demodulate(HfiState *me, const HfiCfg *cfg, float i_q) {
  if (!me->enabled) {
    return;
  }

  // ---- 阶段 1: BPF 带通滤波 (一阶 HPF 级联一阶 LPF) ----
  // 目的: 提取 f_h 附近的 q 轴电流分量, 滤除基频和 PWM 谐波
  //
  // HPF (高通截止 = f_h/2):
  //   α_hp = exp(-2π · (f_h/2) · dt)
  //   bpf_hp = α_hp · (bpf_hp_prev + i_q - i_q_prev)
  // 差分方程源自一阶模拟 HPF 的双线性变换离散化
  float alpha_hp = expf(-HFI_PI * cfg->f_h * cfg->dt);
  float bpf_hp = alpha_hp * (me->bpf_hp_prev + i_q - me->i_q_prev);

  // LPF (低通截止 = f_h):
  //   i_q_bpf = bpf_lp_prev + bp_k · (bpf_hp - bpf_lp_prev)
  // 一阶 IIR 低通: y += k · (x - y)
  float i_q_bpf = me->bpf_lp_prev + cfg->bp_k * (bpf_hp - me->bpf_lp_prev);

  // 保存滤波器历史值 (供下一周期使用)
  me->i_q_prev = i_q;
  me->bpf_hp_prev = bpf_hp;
  me->bpf_lp_prev = i_q_bpf;
  me->i_q_bpf = i_q_bpf;

  // ---- 阶段 2: 同步解调 ----
  // ε_raw = i_q_bpf · sin(θ_inj)  ← 同源 DDS 相位
  // 原理: i_q 高频分量幅值 ∝ ΔL·sin(2·Δθ) ≈ 2ΔL·Δθ (小误差)
  //       乘以 sin(θ_inj) 后得到包含 DC 位置误差的分量
  me->demod_raw = i_q_bpf * sinf(me->inject_phase);

  // ---- 阶段 3: 低通滤波提取位置误差 DC 分量 ----
  // 一阶 IIR: pos_err += lpf_k · (ε_raw - pos_err)
  // 截止频率典型 50~200 Hz, 滤除 2×f_h 的解调谐波
  me->pos_err += cfg->lpf_k * (me->demod_raw - me->pos_err);

  // ---- 阶段 4: PLL 锁相环 (PI 环路滤波器 + VCO) ----
  // PI 积分器: 累积稳态误差, 消除静差
  me->pll_integral += cfg->pll_ki * me->pos_err * cfg->dt;
  // 速度输出 = 比例项 (瞬态) + 积分项 (稳态)
  me->speed = cfg->pll_kp * me->pos_err + me->pll_integral;
  // VCO: 角度 = ∫ ω dt
  me->theta += me->speed * cfg->dt;

  // ---- 阶段 5: 角度归一化到 [-π, π] ----
  // 防止角度无限增长导致浮点精度丢失
  while (me->theta > HFI_PI) {
    me->theta -= HFI_2PI;
  }
  while (me->theta < -HFI_PI) {
    me->theta += HFI_2PI;
  }
}

// 获取估计转子角度 (rad) — FOC Park/InvPark 变换用
static inline float hfi_get_theta(const HfiState *me) {
  return me->theta;
}

// 获取估计电角速度 (rad/s) — 速度环反馈用
// 注意: 这是电角速度, 机械角速度需除以极对数
static inline float hfi_get_speed(const HfiState *me) {
  return me->speed;
}

// 使能注入 — 低速/零速区激活
static inline void hfi_enable(HfiState *me) {
  me->enabled = true;
}

// 禁用注入 — 中高速切换至 eSMO 前调用
// 同时清零注入电压, 避免叠加到电流环输出
static inline void hfi_disable(HfiState *me) {
  me->enabled = false;
  me->v_d_inj = 0.0f;
}

#endif  // COMP_HFI_H
