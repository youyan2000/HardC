// 并行形式 PID 控制器 —— PidBase 的子类
//
// 并行结构: output = K * (Kp*err + Ki*∫err*dt + Kd*derr/dt)
// 各环节独立 — 改 Kp 不影响 I/D 贡献
// 与 PidStandard (串行形式) 的区别: 串行结构 output = Kp * (err + Ki*∫err*dt + Kd*derr/dt)
//
// 并行 PID 公式

#ifndef PID_PARALLEL_H
#define PID_PARALLEL_H

#include "comp_pid.h"
#include <stdbool.h>

// ======== 配置结构体 ========
typedef struct {
  float kp;        // 比例增益 (独立于 I/D)
  float ki;        // 积分增益 (独立于 P/D)
  float kd;        // 微分增益 (独立于 P/I)
  float k;         // 总输出增益 (可选, 默认 1.0)
  float i_limit;   // 积分项输出限幅 (0=不限)
  bool  d_on_measurement;  // true=微分先行(D只对反馈求导), false=标准D(对误差求导)
} PidParallelConfig;

// ======== 子类结构体 ========
typedef struct {
  PidBase           base;    // 基类 (必须第一个)
  PidParallelConfig cfg;     // 可热替换的配置

  // 运行时状态
  float integrator;          // 积分累加值 (已乘 DT)
  float prev_error;          // 上一拍误差 (标准 D 用)
  float prev_measure;        // 上一拍测量值 (微分先行用)
} PidParallel;

// ======== 构造 ========
void pid_parallel_init(PidParallel *me, float dt, float out_min, float out_max,
                       const PidParallelConfig *cfg);

// ======== 运行时调参 ========
void pid_parallel_update_config(PidParallel *me, const PidParallelConfig *cfg);
void pid_parallel_set_kp(PidParallel *me, float kp);
void pid_parallel_set_ki(PidParallel *me, float ki);
void pid_parallel_set_kd(PidParallel *me, float kd);
void pid_parallel_set_k(PidParallel *me, float k);

#endif
