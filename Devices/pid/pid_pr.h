// PR (比例谐振) 控制器 —— PidBase 的子类
//
// 内模原理: 内嵌谐振器 s/(s²+ω²), 在谐振频率 ω 处提供无穷大增益,
//   实现对该频率正弦信号的零静差跟踪。
//   对比: PI 的积分器 1/s 只能跟踪直流 (0Hz), PR 可以跟踪任意指定频率的交流信号。
//
// 典型应用: 并网逆变器电流控制 (50Hz 基波 + 3/5/7 次谐波补偿)、
//   有源电力滤波器 (APF)、UPS 逆变器
//
// 传递函数 (连续域):
//   C(s) = Kp + Σ Ki_n * s/(s² + ω_n²)
//   其中 ω_n = n * 2π * f₀  (n=1 为基波, n=3,5,7... 为谐波)
//
// 离散化: Tustin (双线性) 变换 s = (2/dt) * (z-1)/(z+1)
//   每个谐振器 → biquad Direct Form I: y = b0*x + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]

#ifndef PID_PR_H
#define PID_PR_H

#include "comp_pid.h"
#include <stdint.h>

// ======== 谐波配置 ========
typedef struct {
  uint8_t order;    // 谐波次数: 1=基波, 3=3次, 5=5次...
  float   ki;       // 该谐波的谐振增益 (越大响应越快, 但过大会振荡)
} PRHarmonic;

// ======== 配置结构体 ========
#define PR_MAX_HARMONICS 5    // 最多同时跟踪 5 个频率分量

typedef struct {
  float       kp;                    // 比例系数 (全频段)
  float       f0;                    // 基波频率 (Hz), 如 50.0
  uint8_t     num_harmonics;         // 实际谐波个数 (≤ PR_MAX_HARMONICS)
  PRHarmonic  harmonics[PR_MAX_HARMONICS]; // 各谐波配置
  float       deadzone;              // 死区 (0=不启用)
} PRConfig;

// ======== 单个谐振器状态 (biquad DF1) ========
typedef struct {
  float b0, b1, b2;   // 分子系数 (b1 恒为 0, 这里保留以便调试)
  float a1, a2;       // 分母系数 (a0 归一化为 1)
  float x1, x2;       // 输入历史: x[n-1], x[n-2]
  float y1, y2;       // 输出历史: y[n-1], y[n-2]
  float ki;           // 保存 ki 以便运行时调参重算系数
  float w;             // 保存角频率 ω (rad/s) 以便重算
} PRResonator;

// ======== 子类结构体 ========
typedef struct {
  PidBase   base;                     // 基类 (必须第一个)
  PRConfig  cfg;                      // 配置 (可热替换)
  PRResonator res[PR_MAX_HARMONICS];  // 各谐波谐振器状态
} PidPR;

// ======== 构造 ========

// 初始化 PR: 预计算所有 biquad 系数 + 绑定 ops
void pid_pr_init(PidPR *me, float dt, float out_min, float out_max,
                 const PRConfig *cfg);

// ======== 运行时调参 ========

// 整体替换配置 (重新计算所有谐振器系数)
void pid_pr_update_config(PidPR *me, const PRConfig *cfg);

// 快捷调参
void pid_pr_set_kp(PidPR *me, float kp);
void pid_pr_set_ki(PidPR *me, uint8_t harmonic_order, float ki);

#endif
