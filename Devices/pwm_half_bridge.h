#ifndef PWM_HALF_BRIDGE_H
#define PWM_HALF_BRIDGE_H

// 半桥 PWM —— PwmBase 子类
//
// 拓扑: 两个开关管串联 (上管+下管), 互补导通, 死区插入防止直通
// 应用: Buck/Boost 同步整流、全桥的一条腿、三相逆变器的一个桥臂
//
// 可调参数:
//   - 开关频率 (Hz)
//   - 占空比 (0.0~1.0, 上管导通占比)
//   - 死区时间 (ns)
//
// 输出波形: 中心对齐 (上下计数), CMP1=上升沿, CMP3=下降沿
//   - TA1 (高侧): SET=CMP1, RESET=CMP3
//   - TA2 (低侧): 死区单元自动生成互补波形

#include "comp_pwm.h"
#include "bsp_pwm.h"

// ======== 子类结构体 —— base 必须是第一个成员 ========
typedef struct {
  PwmBase  base;               // 基类 (必须为第一个成员, container_of 依赖)

  // BSP 硬件绑定
  BspPwmConfig bsp_cfg;        // BSP 顶层配置 (时钟 / DLL / 句柄)
  BspPwmTimer  timer;          // 绑定的定时器编号 (如 BSP_TIMER_A)
  uint32_t     output_mask;    // 使能的输出通道 (如 TA1 | TA2)

  // 电力电子参数
  float    duty;               // 当前占空比 (0.0~1.0, 上管导通占比)
  uint32_t deadtime_ns;        // 死区 (ns)
  uint32_t period;             // PWM 周期 (BSP 计数值, 由频率换算)

  // 工作模式
  bool     center_aligned;     // true=中心对齐 (默认), false=边沿对齐
  bool     active_high;        // true=高有效 (默认), false=低有效
} PwmHalfBridge;

// ======== 构造 ========

// 初始化半桥 PWM:
//   freq_hz:     开关频率 (如 100000 = 100kHz)
//   deadtime_ns: 死区 (如 200 = 200ns)
//   timer:       硬件定时器编号
//   output_mask: 输出通道掩码 (通常为 BSP_OUT_TIMER_X_PAIR)
void pwm_hb_init(PwmHalfBridge *me, uint32_t freq_hz, uint32_t deadtime_ns,
                 BspPwmTimer timer, uint32_t output_mask);

// 反初始化: 停止输出 → 清除 BSP 配置
void pwm_hb_deinit(PwmHalfBridge *me);

// ======== 运行时调参 ========
void pwm_hb_set_duty(PwmHalfBridge *me, float duty);
void pwm_hb_set_freq(PwmHalfBridge *me, uint32_t freq_hz);
void pwm_hb_set_deadtime(PwmHalfBridge *me, uint32_t deadtime_ns);
void pwm_hb_set_alignment(PwmHalfBridge *me, bool center_aligned);

#endif
