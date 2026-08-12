#ifndef PID_STANDARD_H
#define PID_STANDARD_H

// 标准 PID 控制器 —— PidBase 的子类
//
// 内模原理: 内嵌积分器 (1/s) 跟踪阶跃/恒值信号, 微分器 (s) 提供阻尼
// 2-DOF 路径分离: D 项可选微分先行 (只对测量值求导) + Kf 前馈直接走目标通道
// 状态同步: on_saturation 回调实现反算抗积分饱和
//
// 特性: 死区 / 变速积分 / 积分分离 / 微分先行 / 前馈 / 积分输出限幅 / 抗积分饱和

#include "comp_pid.h"

// ======== 配置结构体 (与运行时状态分离, 可原子替换) ========
typedef struct {
  float kp, ki, kd, kf;           // PID + 前馈系数
  bool  d_on_measurement;         // true=微分先行(D只对反馈求导), false=标准D(对误差求导)
  float i_limit;                  // 积分项输出限幅 (0=不限), 如 100 → I贡献 ∈ [-100, 100]
  float i_var_a, i_var_b;         // 变速积分: |Err|≤A 全速, A<|Err|<A+B 线性递减, |Err|≥A+B 停止
  float i_sep_threshold;          // 积分分离: |Err|≥阈值时清零积分 (0=不分离)
  float deadzone;                 // 死区, |Err|<deadzone 强制 Err=0
} PidStdConfig;

// ======== 子类结构体 ========
typedef struct {
  PidBase      base;              // 基类 (必须第一个, container_of 依赖)
  PidStdConfig cfg;               // 可热替换的配置

  // 运行时状态 (仅标准 PID 需要, 不污染其他子类)
  float integrator;               // 积分累加值 (已乘 DT)
  float prev_measure;             // 上一拍测量值 (微分先行用)
  float prev_error;               // 上一拍误差 (标准 D 用)
  float prev_target;              // 上一拍目标值 (前馈 Kf 用)
} PidStandard;

// ======== 构造 ========

// 初始化标准 PID: 调基类构造 → 复制配置 → 绑定 ops
void pid_std_init(PidStandard *me, float dt, float out_min, float out_max,
                  const PidStdConfig *cfg);

// ======== 运行时调参 ========

// 运行时替换全部配置 (不影响积分器状态, 只改系数)
void pid_std_update_config(PidStandard *me, const PidStdConfig *cfg);

// 逐个参数热修改 (快捷方式)
void pid_std_set_kp(PidStandard *me, float kp);
void pid_std_set_ki(PidStandard *me, float ki);
void pid_std_set_kd(PidStandard *me, float kd);
void pid_std_set_kf(PidStandard *me, float kf);
void pid_std_set_i_limit(PidStandard *me, float v);
void pid_std_set_deadzone(PidStandard *me, float v);
void pid_std_set_d_on_measurement(PidStandard *me, bool on);

#endif
