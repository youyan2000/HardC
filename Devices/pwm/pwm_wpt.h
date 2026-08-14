#ifndef PWM_WPT_H
#define PWM_WPT_H

// 无线充电 (WPT) 线圈驱动 —— PwmBase 子类, 单相半桥
//
// 拓扑: E 侧半桥 (互补对输出, 如 BSP_OUT_TIMER_E_PAIR), 中心对齐对称双沿,
//   驱动线圈 (LCL 谐振网络). 状态机: WPT_OFF → WPT_CHARGING → WPT_FINISHED;
//   故障走 WPT_ERROR (急停).
//
// 占空比钳位 [max(duty_emin, vb_limit_by_duty), 0.99]: duty_emin 防线圈驱动过弱,
//   vb_limit_by_duty 由高压侧 (VB) 上限动态抬升 duty_min (0=禁用).
//
// 线圈频率: 主频 freq_hz 经 freq_div4 分频 (WPT 用 4: 52kHz 主频 → 13kHz 线圈).
//   分频值由本设备持有并提供 coil_freq 查询; 实际 DMA/CMP 周期分频调度是
//   BSP/工具链 AI 缺口 (见 AGENT-SYNC), 本层不直接写分频寄存器.
//
// 接线契约: init 不写硬件 (bsp_cfg.handle 为 NULL). App 注入 handle + clk_hz
//   后调 pwm_wpt_set_freq → set_duty → start. 字段集固定, 不增 deadtime 字段:
//   start 时死区=0, App 需要死区时在 start 后调 set_deadtime (直接转发 BSP).

#include "comp_pwm.h"
#include "bsp_pwm.h"

// ======== 状态机 ========
typedef enum {
  WPT_ERROR = 0,  // 错误 (故障急停)
  WPT_OFF,        // 关闭 (等待充电请求)
  WPT_CHARGING,   // 充电中 (E 侧半桥输出)
  WPT_FINISHED,   // 充满 (停充等待)
} WptSt;

// ======== 子类结构体 —— base 必须是第一个成员 ========
typedef struct {
  PwmBase base;            // 基类 (必须为第一个成员)
  BspPwmConfig bsp_cfg;    // BSP 硬件绑定 (App 注入 handle + clk_hz)
  BspPwmTimer timer;       // E 侧半桥定时器 (单相)
  uint32_t output_mask;    // 输出掩码 (互补对, 如 BSP_OUT_TIMER_E_PAIR)
  float duty_emin;         // 占空比硬下限 (防线圈驱动过弱, init 传入, 如 0.1)
  float duty;              // 当前占空比 (运行态, public)
  float vb_limit_by_duty;  // 高压侧 (VB) 上限对应的最小占空比, 动态抬升 duty_min (0=禁用)
  uint8_t freq_div4;       // DMA 周期分频: 线圈有效频率 = freq_hz / freq_div4
  WptSt st;                // 状态机 (public, host 测试可读)
} PwmWpt;

// ======== 构造 ========

// 构造: 绑定 ops + 存 timer/mask/duty_emin + base.duty_min=duty_emin, duty_max=0.99f
//   base.mode=HalfBridge, num_ch=1, freq_div4=1, st=WPT_OFF, duty=0, vb_limit_by_duty=0
//   不写硬件 (handle 由 App 经 bsp_cfg 注入后, set_freq 才触碰寄存器).
void pwm_wpt_init(PwmWpt *me, uint32_t freq_hz, BspPwmTimer timer, uint32_t output_mask, float duty_emin);

// ======== 运行时调参 ========

// 写占空比: 经 pwm_set_duty(&me->base, 0, duty) 钳位到 [max(duty_emin, vb_limit_by_duty), 0.99]
void pwm_wpt_set_duty(PwmWpt *me, float duty);

// 动态抬升占空比下限 (VB_LIMIT_BY_DUTY): vb_limit_by_duty = duty_lo;
//   base.duty_min = max(duty_emin, duty_lo); 若当前 duty < 新下限, 重新写硬件钳位
void pwm_wpt_set_vb_limit(PwmWpt *me, float duty_lo);

// 线圈频率分频: div 默认 1; WPT 用 4 (52kHz 主频 → 13kHz 线圈). DMA 分频是 BSP/工具链责任
void pwm_wpt_set_freq_div(PwmWpt *me, uint8_t div);

// 线圈有效频率 = base.freq_hz / freq_div4 (div==0 时返回 base.freq_hz)
uint32_t pwm_wpt_coil_freq_hz(const PwmWpt *me);

// ======== 状态机 ========

// 显式置状态 (host 测试/业务模块注入)
void pwm_wpt_set_state(PwmWpt *me, WptSt st);
WptSt pwm_wpt_state(const PwmWpt *me);

// ======== 生命周期 (薄包装: 底层启停 + 同步状态机) ========
void pwm_wpt_start(PwmWpt *me);      // pwm_start + st=WPT_CHARGING
void pwm_wpt_stop(PwmWpt *me);       // pwm_stop  + st=WPT_OFF
void pwm_wpt_emergency(PwmWpt *me);  // pwm_emergency_stop + st=WPT_ERROR

// ======== 频率/死区 (PwmBase 统一接口包装) ========
void pwm_wpt_set_freq(PwmWpt *me, uint32_t freq_hz);
void pwm_wpt_set_deadtime(PwmWpt *me, uint32_t deadtime_ns);

#endif  // PWM_WPT_H
