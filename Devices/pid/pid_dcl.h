// DCL 风格二自由度 PID —— PidBase 子类
//
// 来源: TI controlSUITE DCL (Digital Controller Library)
// 翻译为 C-OOP 纯C float 版本
//
// 与 PidStandard 的关键差异:
//   1. Kr 设定点权重 — Kr=1.0=标准PID, Kr=0.0=I-PD (P只对反馈作用), 默认0.5
//   2. D 项一阶低通滤波器 — c1/c2 系数, 抑制高频噪声放大
//   3. 支路独立运算 — P/I/D 三条支路并行计算后求和
//   4. 乘法型抗积分饱和 — 饱和时积分器冻结 (无回算, 速度更快)
//   5. 外部抗饱和输入 lk — 允许直流母线限制等外部条件冻结积分器
//
// 算法 (与 DCL_PID.asm 一致):
//   微分支路: v1 = Kd*c1*yk, v4 = v1-d2-d3, d2=v1, d3=c2*v4
//   比例支路: v5 = Kr*rk - yk - v4, v6 = Kp*v5
//   积分支路: v7 = Ki*Kp*(rk-yk)*i14, i10+=v7
//   输出:     v9 = v6+i10, uk=clamp(v9), i14=(饱和?0:1)*lk

#ifndef PID_DCL_H
#define PID_DCL_H

#include "comp_pid.h"

// ======== 配置结构体 ========
typedef struct {
  float kp;              // 比例增益
  float ki;              // 积分增益 (注: 有效积分增益 = Ki*Kp)
  float kd;              // 微分增益
  float kr;              // 设定点权重 (0=I-PD, 1=标准PID, 默认0.5)
  float c1;              // D 项滤波器输入系数 (通常 2*PI*fc*T)
  float c2;              // D 项滤波器反馈系数 (通常 1/(1+2*PI*fc*T))
} PidDclConfig;

// ======== 子类结构体 ========
typedef struct {
  PidBase base;                    // 基类 (必须为第一个成员)
  PidDclConfig cfg;                // 可热替换的配置

  // D 项滤波器状态 — 一阶低通 IIR
  float d2;                        // 滤波器输入延迟 d(k-1) = Kd*c1*yk(k-1)
  float d3;                        // 滤波器反馈延迟 d(k-1) = c2*v4(k-1)

  // I 项状态
  float i_storage;                 // 积分累加器 i10

  // 抗积分饱和
  float i14;                       // 饱和度标记 (1.0=未饱和正常积分, 0.0=饱和冻结)
  float prev_ref;                  // 上一拍目标值 (备用)
  float prev_fbk;                  // 上一拍反馈值 (备用)
} PidDcl;

// 构造
void pid_dcl_init(PidDcl *me, float dt, float out_min, float out_max,
                  const PidDclConfig *cfg);

// 运行时替换配置 (不影响积分器/D滤波器状态)
void pid_dcl_update_config(PidDcl *me, const PidDclConfig *cfg);

// 逐个参数热修改
void pid_dcl_set_kp(PidDcl *me, float kp);
void pid_dcl_set_ki(PidDcl *me, float ki);
void pid_dcl_set_kd(PidDcl *me, float kd);
void pid_dcl_set_kr(PidDcl *me, float kr);

// 设置 D 项滤波器截止频率
//   fc_hz: 滤波器截止频率 (Hz), 如 500
//   内部计算: c1 = 2*PI*fc*dt, c2 = 1/(1+2*PI*fc*dt)
void pid_dcl_set_dfilt_freq(PidDcl *me, float fc_hz);

#endif  // PID_DCL_H
