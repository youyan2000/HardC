// 4 状态 PI 调节器 —— PidBase 子类 (设定值滤波 + 前馈 + clamping 抗饱和)
//
// 来源: TI controlSUITE motor_control/math_blocks/v4.3/pi_reg4.h
//       翻译为 HardC PidBase 子类 (算法来自 comp_pi_reg4.h, 包装为子类形态)
//
// 算法流程: sp_filter → P + I (含抗饱和) + FF → 输出限幅
//   状态 1 —— 设定值滤波: 一阶低通抑制阶跃微分冲击
//   状态 2 —— 比例路径 P:  Kp * (sp_filtered - fbk)
//   状态 3 —— 积分路径 I:  Ki * ∫(sp_filtered - fbk) dt, 抗积分饱和
//   状态 4 —— 前馈路径 FF: Kff * sp_raw, 加快跟踪

#ifndef PID_REG4_H
#define PID_REG4_H

#include "comp_pid.h"

// ======== 配置 POD ========
typedef struct {
  float kp;          // 比例增益
  float ki;          // 积分增益
  float kff;         // 前馈增益 (output/setpoint)
  float sp_fc;       // 设定值滤波器截止频率 (Hz, 0=直通)
} PidReg4Cfg;

// ======== 子类结构体 —— 基类第一成员 ========
typedef struct {
  PidBase base;
  PidReg4Cfg cfg;
  // 运行时状态
  float integral;
  float sp_filtered;
  bool  initialized;
} PidReg4;

// 构造 (自动绑定 ops + 基类)
void pid_reg4_init(PidReg4 *me, float dt, float out_min, float out_max,
                   const PidReg4Cfg *cfg);
// 运行时调参
void pid_reg4_update_config(PidReg4 *me, const PidReg4Cfg *cfg);
void pid_reg4_set_kp(PidReg4 *me, float v);
void pid_reg4_set_ki(PidReg4 *me, float v);
void pid_reg4_set_kff(PidReg4 *me, float v);
void pid_reg4_set_sp_fc(PidReg4 *me, float fc);

#endif  // PID_REG4_H
