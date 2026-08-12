#ifndef PWM_FULL_BRIDGE_H
#define PWM_FULL_BRIDGE_H

// 全桥移相 PWM —— PwmBase 子类
//
// 拓扑: 两个半桥 (A腿 + B腿) 组成 H 桥, 通过调节两腿之间的相位差控制功率传输
// 应用: 移相全桥 DC/DC (PSFB)、双有源桥 (DAB)、无线电能传输 (WPT)
//
// 可调参数:
//   - 开关频率 (Hz)
//   - A腿占空比 (0.0~1.0)
//   - B腿占空比 (0.0~1.0)
//   - 死区时间 (ns, 每腿独立或共用)
//   - 移相角 (度, A腿 vs B腿, 0~180)
//
// 控制原理:
//   移相角 = 0   → 同相, 零功率传输 (输出电压 = 0)
//   移相角 = 90  → 最大功率传输
//   移相角 = 180 → 反向最大功率
//
// 输出:
//   ch=0 → A腿 (TA1/TA2, 互补+死区)
//   ch=1 → B腿 (TB1/TB2, 互补+死区)

#include "comp_pwm.h"
#include "bsp_pwm.h"

// 来源: TI controlSUITE PWMDRV_PSFB
// PSFB ZVS 状态
typedef enum {
  PsfbZvsState_Unknown,
  PsfbZvsState_Achieved,   // 四管均实现 ZVS
  PsfbZvsState_LeadingLost,// 超前腿 ZVS 丢失 (轻载 — 能量不足)
  PsfbZvsState_LaggingLost,// 滞后腿 ZVS 丢失 (重载 — di/dt 过大)
  PsfbZvsState_AllLost,    // 两腿均丢失 ZVS
} PsfbZvsState;
typedef struct {
  PwmBase  base;               // 基类 (必须为第一个成员)

  // BSP 硬件绑定 (两腿各占一个定时器)
  BspPwmConfig bsp_cfg;
  BspPwmTimer  timer_a;        // A腿定时器
  BspPwmTimer  timer_b;        // B腿定时器
  uint32_t     output_mask_a;  // A腿输出掩码
  uint32_t     output_mask_b;  // B腿输出掩码

  // 电力电子参数
  float    duty_a;             // A腿占空比
  float    duty_b;             // B腿占空比
  uint32_t deadtime_ns;        // 死区 (两腿共用)
  float    phase_deg;          // 移相角 (B腿滞后 A腿的角度, 0~180)
  uint32_t period;             // PWM 周期 (BSP 计数值)

  // 工作模式
  bool     center_aligned;     // true=中心对齐

  // === PSFB ZVS 自适应 (控制层访问, 来源: TI controlSUITE PWMDRV_PSFB) ===
  bool  zvs_adaptive_enable;   // 使能 ZVS 自适应死区
  float zvs_min_deadtime_ns;   // 最小死区 (ns)
  float zvs_max_deadtime_ns;   // 最大死区 (ns)
  float zvs_current_threshold; // 进入 ZVS 的负载电流阈值 (A)
  float duty_loss_comp;        // 占空比丢失补偿系数 (pu, 典型 0.02~0.08)
  PsfbZvsState zvs_state;     // 当前 ZVS 状态 (ISR 更新)
  float zvs_margin_pu;         // ZVS 裕量 (0~1, 1=裕量充足, 0=完全丢失)
} PwmFullBridge;

// ======== 构造 ========

// 初始化全桥 PWM:
//   freq_hz:     开关频率
//   deadtime_ns: 死区
//   timer_a/b:   A/B 腿硬件定时器编号
//   out_mask_a/b: A/B 腿输出通道掩码
void pwm_fb_init(PwmFullBridge *me, uint32_t freq_hz, uint32_t deadtime_ns,
                 BspPwmTimer timer_a, BspPwmTimer timer_b,
                 uint32_t out_mask_a, uint32_t out_mask_b);

void pwm_fb_deinit(PwmFullBridge *me);

// ======== 运行时调参 ========

// 设置 A 腿占空比
void pwm_fb_set_duty_a(PwmFullBridge *me, float duty);

// 设置 B 腿占空比
void pwm_fb_set_duty_b(PwmFullBridge *me, float duty);

// 设置移相角 (B 腿相对于 A 腿的滞后角度)
void pwm_fb_set_phase(PwmFullBridge *me, float phase_deg);

// 设置开关频率
void pwm_fb_set_freq(PwmFullBridge *me, uint32_t freq_hz);

// 设置死区
void pwm_fb_set_deadtime(PwmFullBridge *me, uint32_t deadtime_ns);

// 配置 ZVS 自适应死区
void pwm_fb_set_zvs_adaptive(PwmFullBridge *me, bool enable,
                              float min_ns, float max_ns, float i_threshold_a);

// 设置占空比丢失补偿
void pwm_fb_set_duty_loss_comp(PwmFullBridge *me, float comp);

// ISR 调用: 根据负载电流自适应调整死区
// i_load: 当前负载电流 (A) — 绝对值
// 返回: 本周期应使用的死区 (ns)
float pwm_fb_adaptive_deadtime(PwmFullBridge *me, float i_load);

// ISR 调用: ZVS 裕量估计 — 检查超前腿和滞后腿的 ZVS 状态
// i_leading:  超前腿电流 (A, 开关时刻)
// i_lagging:  滞后腿电流 (A, 开关时刻)
// vds_sample: Vds 采样值 (V, 开通前瞬间)
// 返回: ZVS 状态
PsfbZvsState pwm_fb_zvs_margin_update(PwmFullBridge *me,
                                       float i_leading, float i_lagging,
                                       float vds_sample);

// 占空比丢失补偿 — 根据负载电流修正实际占空比
// duty_target: 目标有效占空比 (补偿前)
// i_load:      负载电流 (A)
// 返回: 补偿后的占空比 (已 clamp 到 duty_min/duty_max)
float pwm_fb_duty_loss_compensate(PwmFullBridge *me, float duty_target, float i_load);

#endif
