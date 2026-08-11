// 电机控制 — 速度估算 (从转角微分类)
//
// 来源: TI controlSUITE motor_control/math_blocks/v4.3 (speed_est.h, speed_pr.h)
// 翻译为 C-OOP 纯C float inline 版本
//
// 两种模式:
//   SpeedEstFreq — 频率法: 转角微分 → LPF → 速度 (适合中高速)
//   SpeedEstPeriod — 周期法: 1/周期 → 速度 (适合低速精测)
//   实际使用可混合: 高速用频率法, 低速用周期法

#ifndef COMP_SPEED_H
#define COMP_SPEED_H

#include <math.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265f
#endif

#ifndef M_2PI
#define M_2PI 6.283185f
#endif

// ======================= SpeedEstFreq (转角微分频率法) =======================

// 速度估算 — 通过连续两帧转角差计算速度, 再经一阶 LPF 平滑
typedef struct {
  float theta;            // 输入: 当前估计转角 (标幺, 0~1)
  float old_theta;        // 历史: 上一帧转角
  float speed_pu;         // 输出: 估计速度 (标幺)
  float speed_rpm;        // 输出: 估计速度 (rpm)
  float base_rpm;         // 参数: 基速 (rpm)
  float k1;               // 参数: 微分常数 = 1 / (Ts × f_base)
  float k2;               // 参数: LPF 反馈系数 = τ / (τ + Ts)
  float k3;               // 参数: LPF 输入系数 = Ts / (τ + Ts)
  float raw_speed;        // 内部: 原始微分速度
} SpeedEstFreq;

#define SPEED_EST_FREQ_DEFAULTS { 0, 0, 0, 0, 3600, 1, 0.9f, 0.1f, 0 }

// 初始化
//   ts:        采样周期 (s)
//   base_rpm:  基速 (rpm), 如 3600
//   fc_hz:     LPF 截止频率 (Hz), 如 10
static inline void speed_est_freq_init(SpeedEstFreq *me, float ts,
                                        float base_rpm, float fc_hz) {
  me->theta = 0.0f;
  me->old_theta = 0.0f;
  me->speed_pu = 0.0f;
  me->speed_rpm = 0;
  me->base_rpm = base_rpm;

  // 微分常数: k1 = f_base / f_samp = 1 / (Ts × fb)
  float fb = base_rpm / 60.0f;                    // 基频 (Hz)
  me->k1 = 1.0f / (ts * fb);                       // 微分缩放

  // LPF 系数
  float tc = 1.0f / (M_2PI * fc_hz);
  me->k2 = tc / (tc + ts);                         // 反馈
  me->k3 = ts / (tc + ts);                         // 输入
  me->raw_speed = 0.0f;
}

// 单步速度估算 (ISR 中每控制周期调用)
//   返回: 估计速度 (rpm)
static inline float speed_est_freq_tick(SpeedEstFreq *me, float theta) {
  me->theta = theta;

  // 阶段 1: 转角微分 + 跳变解包 (unwrap)
  float delta = me->theta - me->old_theta;
  if (delta < -0.5f) {
    delta += 1.0f;     // 下包: 0 → 1 跳变
  } else if (delta > 0.5f) {
    delta -= 1.0f;     // 上包: 1 → 0 跳变
  }

  // 微分: 原始速度 (标幺)
  me->raw_speed = me->k1 * delta;

  // 阶段 2: 一阶 LPF (IIR)
  // speed[k] = k2 × speed[k-1] + k3 × raw[k]
  me->speed_pu = me->k2 * me->speed_pu + me->k3 * me->raw_speed;

  // 饱和
  if (me->speed_pu > 1.0f)  me->speed_pu = 1.0f;
  if (me->speed_pu < -1.0f) me->speed_pu = -1.0f;

  // 历史更新
  me->old_theta = me->theta;

  // 阶段 3: 标幺 → rpm
  me->speed_rpm = me->base_rpm * me->speed_pu;

  return me->speed_rpm;
}

static inline void speed_est_freq_reset(SpeedEstFreq *me) {
  me->theta = 0.0f;
  me->old_theta = 0.0f;
  me->speed_pu = 0.0f;
  me->speed_rpm = 0;
  me->raw_speed = 0.0f;
}

// ======================= SpeedEstPeriod (周期法速度测量) =======================

// 速度测量 — 通过捕获事件周期计算速度
// Speed = SpeedScaler / EventPeriod, SpeedRpm = BaseRpm × Speed
typedef struct {
  float new_timestamp;    // 内部: 新时间戳
  float old_timestamp;    // 内部: 旧时间戳
  float timestamp;        // 输入: 当前捕获时间戳
  float speed_scaler;     // 参数: 速度标量 (用于定标转换)
  float event_period;     // 输入/内部: 事件周期 (计数值)
  float speed_pu;         // 输出: 速度 (标幺)
  float base_rpm;         // 参数: 基速 (rpm)
  float speed_rpm;        // 输出: 速度 (rpm)
  bool  use_event_period; // 模式: true=直接使用外部事件周期, false=从时间戳计算
} SpeedEstPeriod;

#define SPEED_EST_PERIOD_DEFAULTS { 0, 0, 0, 260, 0, 0, 1800, 0, false }

static inline void speed_est_period_init(SpeedEstPeriod *me, float speed_scaler,
                                          float base_rpm) {
  me->new_timestamp = 0;
  me->old_timestamp = 0;
  me->timestamp = 0;
  me->speed_scaler = speed_scaler;
  me->event_period = 0;
  me->speed_pu = 0;
  me->base_rpm = base_rpm;
  me->speed_rpm = 0;
  me->use_event_period = false;
}

// 周期法单步运行 — 从时间戳推算
//   timestamp: 当前计时器值
//   max_count: 计时器溢出值 (如 32767)
static inline float speed_est_period_tick_from_ts(SpeedEstPeriod *me,
                                                   float timestamp,
                                                   float max_count) {
  me->timestamp = timestamp;

  // 周期 = timestamp[k] - timestamp[k-1]
  me->old_timestamp = me->new_timestamp;
  me->new_timestamp = me->timestamp;
  me->event_period = me->new_timestamp - me->old_timestamp;

  // 处理计时器绕回
  if (me->event_period < 0) {
    me->event_period += max_count;
  }

  // 速度 = 标量 / 周期
  if (me->event_period > 0) {
    me->speed_pu = me->speed_scaler / me->event_period;
  } else {
    me->speed_pu = 0;
  }

  me->speed_rpm = me->base_rpm * me->speed_pu;
  return me->speed_rpm;
}

// 周期法单步运行 — 直接使用外部事件周期
static inline float speed_est_period_tick_direct(SpeedEstPeriod *me,
                                                  float event_period) {
  me->event_period = event_period;

  if (me->event_period > 0) {
    me->speed_pu = me->speed_scaler / me->event_period;
  } else {
    me->speed_pu = 0;
  }

  me->speed_rpm = me->base_rpm * me->speed_pu;
  return me->speed_rpm;
}

#endif  // COMP_SPEED_H
