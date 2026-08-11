#ifndef PIDS_H
#define PIDS_H

// PID 全局句柄 —— 应用层通过此文件访问所有 PID 实例
// 遵循分层架构: Application → Module → Devices → Components → BSP
//
// 用法:
//   #include "pids.h"
//   float duty = pid_compute(g_pid_vel, target_speed, measured_speed);
//   pid_reset(g_pid_ang);

#include "comp_pid.h"

// 全局 PID 句柄 (由 board_init.c 绑定到具体子类实例)
extern PidBase *g_pid_vel;     // 速度环 PID
extern PidBase *g_pid_ang;     // 角度环 PID
extern PidBase *g_pid_follower; // 循迹 P2PD
extern PidBase *g_pid_current;  // 电流环 PID (PR/QPR)

// 级联 PID (组合模式, 不通过 PidBase*)
extern struct PidCascade *g_cascade;

// 用户扩展句柄
extern PidBase *g_pid_user0;
extern PidBase *g_pid_user1;

#endif
