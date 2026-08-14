#ifndef PWM_INTERLEAVED_H
#define PWM_INTERLEAVED_H

// 多相交错并联 PWM —— PwmBase 子类
//
// 拓扑: N 个相同的半桥并联, 每相之间均匀错相 360°/N, 降低输入/输出电流纹波
//
// 应用:
//   2相交错 Buck → 输入电流纹波减半
//   3相交错 Boost (120° 错相)
//   6相交错 VRM  → CPU 核心供电
//
// 可调参数:
//   - 开关频率 (Hz, 所有相共用)
//   - 占空比 (0.0~1.0, 各相可独立设置以实现均流)
//   - 死区时间 (ns, 所有相共用)
//   - 相数 N (1~6, 对应 HRTIM 最多 6 个定时器)
//
// 相位分配:
//   第 0 相: 0°    (定时器复位触发 = Master PERIOD)
//   第 1 相: 360/N ° (定时器复位触发 = Master CMP 偏移)
//   第 2 相: 720/N °
//   ...
//
// 均流: 各相占空比独立可调, 上层 PID 计算出 per-phase duty 后分别写入

#include "comp_pwm.h"
#include "bsp_pwm.h"

#define PWM_INTERLEAVED_MAX_PHASES 6   // 最多 6 相 (受限于 HRTIM A~F)

// ======== 每相配置 ========
typedef struct {
  BspPwmTimer timer;         // 硬件定时器编号
  uint32_t    output_mask;   // 输出通道掩码
  float       duty;          // 当前占空比
  float       phase_deg;     // 相位偏移 (度, 相对于 0° 参考)
} PwmInterleavedPhase;

// ======== 子类结构体 —— base 必须是第一个成员 ========
typedef struct {
  PwmBase  base;               // 基类 (必须为第一个成员)

  // BSP 硬件绑定
  BspPwmConfig bsp_cfg;

  // 电力电子参数
  uint8_t  num_phases;         // 实际使用的相数 (1~6)
  uint32_t deadtime_ns;        // 死区 (所有相共用)
  uint32_t period;             // PWM 周期 (所有相共用)

  // 每相独立配置
  PwmInterleavedPhase phases[PWM_INTERLEAVED_MAX_PHASES];

  // 工作模式
  bool     center_aligned;
} PwmInterleaved;

// ======== 构造 ========

// 初始化交错并联 PWM:
//   freq_hz:     开关频率
//   deadtime_ns: 死区
//   num_phases:  相数 (1~6)
//   timers:      每相绑定的硬件定时器数组 (长度 = num_phases)
//   out_masks:   每相输出通道掩码数组
void pwm_il_init(PwmInterleaved *me, uint32_t freq_hz, uint32_t deadtime_ns,
                 uint8_t num_phases,
                 const BspPwmTimer *timers,
                 const uint32_t *out_masks);

void pwm_il_deinit(PwmInterleaved *me);

// ======== 运行时调参 ========

// 设置某一相占空比 (phase_idx: 0 ~ num_phases-1)
void pwm_il_set_duty(PwmInterleaved *me, uint8_t phase_idx, float duty);

// 设置所有相统一占空比 (无均流需求时使用)
void pwm_il_set_duty_all(PwmInterleaved *me, float duty);

// 设置开关频率
void pwm_il_set_freq(PwmInterleaved *me, uint32_t freq_hz);

// 设置死区
void pwm_il_set_deadtime(PwmInterleaved *me, uint32_t deadtime_ns);

// 获取当前总相数
uint8_t pwm_il_get_num_phases(PwmInterleaved *me);

#endif
