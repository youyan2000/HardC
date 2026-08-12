#ifndef PWM_BUCKBOOST_H
#define PWM_BUCKBOOST_H

// 单路 Buck/Boost PWM —— PwmBase 子类
//
// 最简单的电力电子 PWM: 一路输出, 可带同步整流 (互补+死区可选)
//
// 应用:
//   Buck 降压 (异步/同步整流)
//   Boost 升压 (异步/同步整流)
//   Buck-Boost 升降压 (单管 + 二极管, 非四开关)
//
// 可调参数:
//   - 开关频率 (Hz)
//   - 占空比 (0.0~1.0)
//   - 死区时间 (ns, 仅同步整流模式)
//
// 同步整流 vs 异步整流:
//   sync_rect = true  → 互补输出 + 死区 (效率更高)
//   sync_rect = false → 单管输出, 另一路固定关闭 (简单可靠)

#include "comp_pwm.h"
#include "bsp_pwm.h"

// ======== 子类结构体 —— base 必须是第一个成员 ========
typedef struct {
  PwmBase  base;               // 基类 (必须为第一个成员)

  // BSP 硬件绑定
  BspPwmConfig bsp_cfg;
  BspPwmTimer  timer;          // 硬件定时器编号
  uint32_t     output_mask;    // 输出通道掩码

  // 电力电子参数
  float    duty;               // 当前占空比
  uint32_t deadtime_ns;        // 死区 (仅同步整流有效)
  uint32_t period;             // PWM 周期 (BSP 计数值)

  // 工作模式
  PwmMode  mode;               // Buck / Boost / BuckBoost
  bool     sync_rect;          // true=同步整流 (互补输出)
  bool     center_aligned;     // true=中心对齐
} PwmBuckBoost;

// ======== 构造 ========

// 初始化单路 PWM:
//   freq_hz:     开关频率
//   timer:       硬件定时器编号
//   output_mask: 输出通道掩码
//   mode:        工作模式 (Buck/Boost/BuckBoost)
//   sync_rect:   是否启用同步整流 (互补输出 + 死区)
void pwm_bb_init(PwmBuckBoost *me, uint32_t freq_hz,
                 BspPwmTimer timer, uint32_t output_mask,
                 PwmMode mode, bool sync_rect);

void pwm_bb_deinit(PwmBuckBoost *me);

// ======== 运行时调参 ========
void pwm_bb_set_duty(PwmBuckBoost *me, float duty);
void pwm_bb_set_freq(PwmBuckBoost *me, uint32_t freq_hz);
void pwm_bb_set_deadtime(PwmBuckBoost *me, uint32_t deadtime_ns);
void pwm_bb_set_mode(PwmBuckBoost *me, PwmMode mode);

#endif
