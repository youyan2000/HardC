// PWM 平台层 —— 电力电子 PWM 输出抽象基类
//
// 基类只定义电力电子 PWM 的共性操作:
//   启停 / 调占空比 / 调频率 / 调死区 / 调相位 / 急停
//
// 子类按功率拓扑分 (不同拓扑 = 不同可调参数组合):
//   PwmBuckBoost   — 单路 PWM (降压/升压),      参数: 频率 + 占空比
//   PwmHalfBridge  — 半桥互补 PWM,               参数: 频率 + 占空比 + 死区
//   PwmFullBridge  — 全桥移相 PWM,               参数: 频率 + 占空比 + 死区 + 移相角
//   PwmInterleaved — 多相交错并联 PWM,            参数: 频率 + 占空比 + 死区 + 相数
//   PwmResonant    — 谐振变换器变频 PWM (LLC/DAB), 参数: 频率范围 + 死区 (占空比固定 50%)
//
// 硬件适配在 BSP 层 (BSP/bsp_pwm.h), 同时支持:
//   STM32 HRTIM (G4/H7/F3) — 高精度定时器, ~184ps 分辨率
//   TI C2000 ePWM          — 增强型 PWM, ~150ps 分辨率
// 本层 (Components + Devices) 不看寄存器, 只管理 PWM 参数和拓扑逻辑

#ifndef COMP_PWM_H
#define COMP_PWM_H

#include <stdint.h>
#include <stdbool.h>

// ======== PWM 工作模式 ========
typedef enum {
  PwmMode_Buck,          // 降压模式
  PwmMode_Boost,         // 升压模式
  PwmMode_BuckBoost,     // 升降压模式 (四开关)
  PwmMode_HalfBridge,    // 半桥模式
  PwmMode_FullBridge,    // 全桥移相模式
  PwmMode_Interleaved,   // 交错并联模式
  PwmMode_Resonant,      // 谐振变频模式
} PwmMode;

// ======== 前向声明 ========
typedef struct PwmBase PwmBase;

// ======== 虚函数签名 (子类各自实现) ========

// 启动 PWM 输出 (先启计数器 → 等波形对齐 → 再开输出)
typedef void (*pwm_start_fn)(PwmBase *me);

// 停止 PWM 输出 (先关输出 → 再停计数器, 防止关断瞬间电平不确定)
typedef void (*pwm_stop_fn)(PwmBase *me);

// 设置指定通道占空比 [0.0, 1.0]
// ch: 通道号 (半桥用 0=N, 全桥用 0=A腿 1=B腿, 交错用 0~N-1)
typedef void (*pwm_set_duty_fn)(PwmBase *me, uint8_t ch, float duty);

// 设置开关频率 (Hz), 如 100000 = 100kHz
// 谐振模式时此值为变频核心参数
typedef void (*pwm_set_freq_fn)(PwmBase *me, uint32_t freq_hz);

// 设置死区时间 (ns), 互补输出的上升沿和下降沿延迟
// 如 200 = 200ns 死区, 防止上下管直通
typedef void (*pwm_set_deadtime_fn)(PwmBase *me, uint32_t deadtime_ns);

// 设置通道间相位差 (度), 0~360
// 全桥: A腿 vs B腿相位; 交错: 相邻相之间相位
typedef void (*pwm_set_phase_fn)(PwmBase *me, uint8_t ch, float phase_deg);

// 紧急停机: 立即封波, 输出置为安全电平 (高阻 / 下拉), 不经过软件判断
typedef void (*pwm_emergency_stop_fn)(PwmBase *me);

// ======== 虚函数表 ========
typedef struct {
  pwm_start_fn            start;
  pwm_stop_fn             stop;
  pwm_set_duty_fn         set_duty;
  pwm_set_freq_fn         set_freq;
  pwm_set_deadtime_fn     set_deadtime;
  pwm_set_phase_fn        set_phase;
  pwm_emergency_stop_fn   emergency_stop;
} PwmOps;

// ======== 基类结构体 —— 只包含电力电子 PWM 共性字段 ========
struct PwmBase {
  const PwmOps *ops;       // 虚表指针 (子类 init 时绑定)
  PwmMode       mode;      // 当前工作模式
  uint8_t       num_ch;    // 独立输出通道数 (1=单路, 2=全桥, N=交错)
  uint32_t      freq_hz;   // 当前开关频率
  float         duty_min;  // 占空比下限 (物理约束, 如 0.05 = 最小 5%)
  float         duty_max;  // 占空比上限 (物理约束, 如 0.95 = 最大 95%)
  bool          running;   // 运行状态标记
};

// ======== 基类构造 ========

// 初始化基类默认字段, ops 由子类 init 时绑定
void pwm_base_init(PwmBase *me);

// ======== 统一对外接口 (static inline, 零开销) ========

// 启动 PWM 输出
static inline void pwm_start(PwmBase *me) {
  if (me->ops->start) {
    me->ops->start(me);
    me->running = true;
  }
}

// 停止 PWM 输出
static inline void pwm_stop(PwmBase *me) {
  if (me->ops->stop) {
    me->ops->stop(me);
    me->running = false;
  }
}

// 设置占空比: 自动限幅到 [duty_min, duty_max], 委托子类写入硬件
static inline void pwm_set_duty(PwmBase *me, uint8_t ch, float duty) {
  // 物理限幅
  if (duty > me->duty_max) duty = me->duty_max;
  else if (duty < me->duty_min) duty = me->duty_min;

  if (me->ops->set_duty) {
    me->ops->set_duty(me, ch, duty);
  }
}

// 设置开关频率: 委托子类重配时基
static inline void pwm_set_freq(PwmBase *me, uint32_t freq_hz) {
  if (me->ops->set_freq && freq_hz > 0) {
    me->freq_hz = freq_hz;
    me->ops->set_freq(me, freq_hz);
  }
}

// 设置死区: 委托子类更新硬件死区寄存器
static inline void pwm_set_deadtime(PwmBase *me, uint32_t deadtime_ns) {
  if (me->ops->set_deadtime) {
    me->ops->set_deadtime(me, deadtime_ns);
  }
}

// 设置相位: 委托子类配置通道间移相
static inline void pwm_set_phase(PwmBase *me, uint8_t ch, float phase_deg) {
  if (me->ops->set_phase) {
    me->ops->set_phase(me, ch, phase_deg);
  }
}

// 紧急停机: 委托子类硬件急停
static inline void pwm_emergency_stop(PwmBase *me) {
  if (me->ops->emergency_stop) {
    me->ops->emergency_stop(me);
    me->running = false;
  }
}

#endif
