// 比例积分 (PI) 控制器 —— PidBase 的子类
//
// 来源: TI controlSUITE solar/v1.2/float (CNTL_PI_F — PI with anti-windup)
// 翻译为 HardC PidBase 子类 (原名 pid_solar, 重命名为通用 pid_pi)
//
// 与 PidStandard 的关键差异:
//   1. 条件积分抗饱和: 当 Out != PreSat 时冻结积分 (简单高效)
//   2. 没有微分项 — 纯 PI 控制器
//   3. 没有设定点权重 — 直接用 (Ref - Fbk) 作为误差
//
// 应用场景: 太阳能 MPPT 电压环/电流环、PFC 电流环、直流母线电压环,
//           任何需要"纯 PI + 条件抗饱和"的功率环

#ifndef PID_PI_H
#define PID_PI_H

#include "comp_pid.h"

// ======== 配置结构体 ========
typedef struct {
  float kp;               // 比例增益
  float ki;               // 积分增益 (注: 有效积分增益 = Ki)
  float out_max;          // 输出上限
  float out_min;          // 输出下限
} PidPiConfig;

// ======== 子类结构体 ========
typedef struct {
  PidBase base;                    // 基类 (必须为第一个成员)
  PidPiConfig cfg;                 // 可热替换的配置

  // 状态
  float up;                        // 比例项 = Kp * error
  float ui;                        // 积分项 = ui_prev + Ki * up (非饱和时)
  float ui_prev;                   // 上拍积分项 (用于条件冻结)
  float v1;                        // 预饱和输出 = up + ui
  float prev_output;               // 上拍最终输出 (用于判断上次是否饱和)
  bool  prev_saturated;            // 上拍是否饱和 (代替 base->output 比较)
} PidPi;

// 构造
void pid_pi_init(PidPi *me, float dt, float out_min, float out_max,
                 const PidPiConfig *cfg);

// 运行时替换配置
void pid_pi_update_config(PidPi *me, const PidPiConfig *cfg);

// 逐个参数热修改
void pid_pi_set_kp(PidPi *me, float kp);
void pid_pi_set_ki(PidPi *me, float ki);

#endif  // PID_PI_H
