// 纯比例 (P) 控制器 —— PidBase 的子类
//
// 仅比例项: Out = Kp × (Ref − Fbk) ，无限幅、无积分、无历史
// 与 MATLAB PID 模块中"只用 P 项"等价
//
// 适用场景: 快速但不需消除稳态误差的环路 (如简单的占空比前馈/开环整形)、
//           与其它控制器串联前的临时整形、只需比例增益的功率环

#ifndef PID_P_H
#define PID_P_H

#include "comp_pid.h"

// ======== 配置结构体 (与运行时状态分离) ========
typedef struct {
  float kp;               // 比例增益
} PidPConfig;

// ======== 子类结构体 —— 基类必须为第一成员 ========
typedef struct {
  PidBase base;           // 基类 (必须为第一成员, container_of 依赖)
  PidPConfig cfg;         // 可热替换的配置
} PidP;

// 构造 (自动绑定 ops + 基类)
void pid_p_init(PidP *me, float dt, float out_min, float out_max,
                const PidPConfig *cfg);

// 运行时替换配置
void pid_p_update_config(PidP *me, const PidPConfig *cfg);

// 逐个参数热修改
void pid_p_set_kp(PidP *me, float kp);

#endif  // PID_P_H
