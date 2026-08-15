// 3 状态 PID 调节器 —— PidBase 子类 (反计算抗饱和 + 位置回绕变体)
//
// 来源: TI controlSUITE motor_control/math_blocks/v4.3/pid_reg3.h
//       翻译为 HardC PidBase 子类 (算法来自 comp_pid_reg3.h, 包装为子类形态)
//
// 标准变体 (compute):  反计算抗饱和 —— 输出被限幅时将饱和差反馈回积分器
//   Err   = Ref - Fdb
//   Up    = Kp×Err
//   Ui   += Ki×Up + Kc×SatErr          // 积分作用在比例输出 + 反计算校正
//   Out   = sat(Up + Ui, OutMax, OutMin)
//   SatErr = Out - (Up + Ui)
//
// 位置变体 (pid_reg3_run_pos): 增加微分项 Ud (作用在比例输出差分) + 误差 ±0.5 回绕,
//   适合位置/角度控制 (反馈归一化到 [0,1]).

#ifndef PID_REG3_H
#define PID_REG3_H

#include "comp_pid.h"

// ======== 配置 POD ========
typedef struct {
  float kp;      // 比例增益
  float ki;      // 积分增益 (作用于比例输出)
  float kc;      // 反计算抗饱和强度 (典型 0.5~1.0)
  float kd;      // 微分增益 (仅位置变体使用)
} PidReg3Cfg;

// ======== 子类结构体 —— 基类第一成员 ========
typedef struct {
  PidBase base;
  PidReg3Cfg cfg;
  // 运行时状态
  float ui;        // 积分输出
  float up;        // 比例输出 (上拍, 位置变体微分用)
  float sat_err;   // 饱和差 (反计算)
} PidReg3;

// 构造 (自动绑定 ops + 基类)
void pid_reg3_init(PidReg3 *me, float dt, float out_min, float out_max,
                   const PidReg3Cfg *cfg);
// 位置变体 (直呼, 不通过虚表; 误差 ±0.5 回绕 + D 作用比例差分)
float pid_reg3_run_pos(PidReg3 *me, float ref, float fdb);
// 运行时调参
void pid_reg3_update_config(PidReg3 *me, const PidReg3Cfg *cfg);
void pid_reg3_set_kp(PidReg3 *me, float v);
void pid_reg3_set_ki(PidReg3 *me, float v);
void pid_reg3_set_kc(PidReg3 *me, float v);
void pid_reg3_set_kd(PidReg3 *me, float v);

#endif  // PID_REG3_H
