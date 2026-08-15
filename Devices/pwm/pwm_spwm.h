// 正弦脉宽调制 SPWM —— PwmBase 子类, 单相/三相正弦式逆变器
//
// 拓扑: 每相一个互补半桥 (上下管 + 死区), 共 ≤3 桥臂:
//   单相全桥 (N=2):  A 臂 0°  + B 臂 180°  → 双极性正弦波
//   三相 (N=3):      A 臂 0° / B 臂 120° / C 臂 240°
//
// 调制原理 (正弦调制波 × 三角载波):
//   上管占空比  duty_i = 0.5 × (1 + m × sin(θ_i))
//   m = 调制比 [0,1], θ_i = 第 i 桥臂的调制波瞬时相位
//   m=1 时线性调制区边界 (三相为 1.15 若注入三次谐波)
//
// 三次谐波注入 (THI, 仅三相有效):
//   注入 ⅙ × sin(3·θ) 零序分量 → 线调制区从 1.0 提升到 2/√3 ≈ 1.1547
//   上管占空比  duty_i = 0.5 × (1 + m × (sin(θ_i) + thi × sin(3θ_i)))
//
// 接口:
//   spwm_set_point(&spwm, m, theta_deg);   // ISR 热路径: 调制比+电角度 → 写全部桥臂
//   spwm_set_thi(&spwm, enable, coeff);     // 三次谐波注入开关 (仅三相, 默认关)
//
// 与 SVPWM 的关系: 均为逆变器调制, SPWM 逐相正弦调制更简单、天然支持
//   单相(N=2)与两电平三相(N=3), 输出正弦; SVPWM 只做三相、电压利用率更高.
//
// 调用链 (典型 ISR):
//   1. 控制环产生: 调制比 m + 电角度 θ (可由 comp_sgen.h SgenFixed 累加)
//   2. SPWM: m, θ → N 桥臂占空比 (本文件)
//   3. BSP: 每桥臂占空比 → 互补半桥 CMP 寄存器 (BSP/bsp_pwm.h, 死区硬件生成)

#ifndef PWM_SPWM_H
#define PWM_SPWM_H

#include "comp_pwm.h"
#include "bsp_pwm.h"

// ======== 常量 ========
#define PWM_SPWM_MAX_ARMS 3   // 最大桥臂数 (6 开关 → 3 个互补半桥)
#define PWM_SPWM_THI_6th    0.1666667f  // 三次谐波注入系数 ⅙ (标准值)

// ======== 每桥臂配置 (init 时传入) ========
typedef struct {
  BspPwmTimer timer;        // 互补半桥定时器 (如 BSP_TIMER_A)
  uint32_t    output_mask;  // 互补输出掩码 (如 BSP_OUT_TIMER_A_PAIR)
} PwmSpwmArmCfg;

// ======== 运行时桥臂状态 (public, host 测试可读) ========
typedef struct {
  BspPwmTimer timer;        // 硬件定时器
  uint32_t    output_mask;  // 输出掩码
  float       phase_deg;    // 调制波相位偏移 (0° / 120° / 240°, 自动生成)
  float       duty;         // 当前上管占空比 (正弦调制律 × 调制比)
} PwmSpwmArm;

// ======== 子类结构体 —— base 必须是第一个成员 ========
typedef struct {
  PwmBase  base;               // 基类 (必须为第一个成员, container_of 依赖)

  // BSP 硬件绑定 (App 注入 handle + clk_hz)
  BspPwmConfig bsp_cfg;

  // 电力电子参数
  uint8_t  num_arms;           // 桥臂数 (2=单相全桥, 3=三相)
  float    mod_index;          // 调制比 m [0,1]
  uint32_t deadtime_ns;        // 死区 (互补半桥)
  uint32_t period;             // 载波周期 (BSP 计数值, 由频率换算)
  bool     center_aligned;     // true=中心对齐 (默认, 对称 PWM)

  // 三次谐波注入 (THI) — 仅三相有效, 提高电压利用率
  bool     thi_enable;         // true=注入三次谐波 (默认关)
  float    thi_coeff;          // 注入系数 (默认 1/6)

  // 每桥臂独立状态
  PwmSpwmArm arms[PWM_SPWM_MAX_ARMS];
} PwmSpwm;

// ======== 构造 ========

// 初始化 SPWM:
//   freq_hz:     载波 (开关) 频率
//   deadtime_ns: 死区 (ns)
//   num_arms:    桥臂数 (2=单相全桥, 3=三相)
//   cfg:         每桥臂定时器+掩码数组 (长度 num_arms)
void pwm_spwm_init(PwmSpwm *me, uint32_t freq_hz, uint32_t deadtime_ns,
                   uint8_t num_arms, const PwmSpwmArmCfg *cfg);

// 反初始化: 停止输出 → 清除 BSP 配置
void pwm_spwm_deinit(PwmSpwm *me);

// ======== 核心调制接口 ========

// 正弦调制 (ISR 热路径): 由调制比 m + 调制波电角度 θ 重算全部桥臂占空比并写硬件
//   m:         调制比 [0, 1] (内部钳位); 若调制峰值触及 duty_max 会被安全削顶
//   theta_deg: 调制波瞬时电角度 (度), 可自增或来自 PLL/SgenFixed
void spwm_set_point(PwmSpwm *me, float m, float theta_deg);

// 设置三相三次谐波注入 (仅 num_arms==3 生效)
void spwm_set_thi(PwmSpwm *me, bool enable, float coeff);

// ======== 运行时调参 (沿用 PwmBase 统一接口) ========

// 兼容入口: 写某桥臂上管绝对占空比 (调试/开环用, 正常闭环用 spwm_set_point)
void pwm_spwm_set_duty(PwmSpwm *me, uint8_t ch, float duty);

// 设置载波频率 (所有桥臂)
void pwm_spwm_set_freq(PwmSpwm *me, uint32_t freq_hz);

// 设置死区 (所有桥臂)
void pwm_spwm_set_deadtime(PwmSpwm *me, uint32_t deadtime_ns);

// 获取桥臂数
uint8_t pwm_spwm_get_num_arms(const PwmSpwm *me);

#endif  // PWM_SPWM_H
