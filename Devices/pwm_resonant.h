#ifndef PWM_RESONANT_H
#define PWM_RESONANT_H

// 谐振变换器变频 PWM —— PwmBase 子类
//
// 拓扑: 通过调节开关频率改变谐振腔阻抗, 从而控制输出电压/功率
// 占空比固定 50% (上下管各半周期), 死区防止直通
//
// 应用:
//   LLC 谐振变换器
//   双有源桥 (DAB) — 变频 + 移相组合控制
//   LCC / CLLC 谐振
//
// 控制原理:
//   频率 ↑ → 谐振腔增益 ↓ → 输出电压 ↓
//   频率 ↓ → 谐振腔增益 ↑ → 输出电压 ↑ (接近谐振点时增益最大)
//
// 可调参数:
//   - 开关频率 (Hz, 核心控制量, 在 [freq_min, freq_max] 范围内调节)
//   - 死区时间 (ns, 固定值, 保证 ZVS 软开关)
//   - 占空比 (固定 50%, 如需微调则走基类 set_duty 接口)
//
// 变频约束:
//   频率范围受限于: 谐振腔参数 (Lr, Cr, Lm) + 磁性元件饱和 + 开关损耗
//   典型范围: 50kHz ~ 300kHz (LLC), 100kHz ~ 500kHz (高频 GaN)

#include "comp_pwm.h"
#include "bsp_pwm.h"

// ======== 子类结构体 —— base 必须是第一个成员 ========
typedef struct {
  PwmBase  base;               // 基类 (必须为第一个成员)

  // BSP 硬件绑定
  BspPwmConfig bsp_cfg;
  BspPwmTimer  timer;          // 硬件定时器编号
  uint32_t     output_mask;    // 输出通道掩码 (互补对)

  // 电力电子参数 (变频是核心)
  uint32_t freq_hz;            // 当前开关频率
  uint32_t freq_min;           // 最低频率 (Hz, 物理约束: 变压器饱和 / 听觉噪声)
  uint32_t freq_max;           // 最高频率 (Hz, 物理约束: 开关损耗 / 死区占比)
  uint32_t deadtime_ns;        // 死区 (ns, 保证 ZVS 软开关的最小死区)
  uint32_t period;             // PWM 周期 (BSP 计数值)

  // 谐振参数 (可选 —— 用于自适应频率计算)
  float    resonant_freq;      // 谐振频率 (Hz, fr = 1 / (2π√LrCr))
  bool     below_resonant;     // true=低于谐振点工作 (ZCS区), false=高于谐振点 (ZVS区)
} PwmResonant;

// ======== 构造 ========

// 初始化谐振变频 PWM:
//   freq_start:  起始开关频率 (通常略高于谐振频率)
//   freq_min:    最低频率限制
//   freq_max:    最高频率限制
//   deadtime_ns: 死区 (ns, 保证 ZVS 的最小值)
//   timer:       硬件定时器
//   output_mask: 输出通道
void pwm_res_init(PwmResonant *me,
                  uint32_t freq_start, uint32_t freq_min, uint32_t freq_max,
                  uint32_t deadtime_ns,
                  BspPwmTimer timer, uint32_t output_mask);

void pwm_res_deinit(PwmResonant *me);

// ======== 运行时调参 (变频为主) ========

// 设置开关频率 (核心控制接口)
void pwm_res_set_freq(PwmResonant *me, uint32_t freq_hz);

// 设置死区
void pwm_res_set_deadtime(PwmResonant *me, uint32_t deadtime_ns);

// 获取当前频率
uint32_t pwm_res_get_freq(PwmResonant *me);

// 获取频率范围
void pwm_res_get_freq_range(PwmResonant *me, uint32_t *freq_min, uint32_t *freq_max);

// 设置谐振参数 (用于自适应控制)
void pwm_res_set_resonant_params(PwmResonant *me, float resonant_freq, bool below_resonant);

#endif
