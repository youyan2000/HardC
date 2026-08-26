// 球板平衡模块 — GenBalance (Module 层)
// gen_balance.c (3×PidStandard 球板平衡)
//
// 核心原理:
//   摄像头/触摸屏 → 球坐标 (x, y) → X/Y 轴 PID → 步进电机步数 → 丝杆倾斜
//   平板倾角 → 重力分量 → 球滚回目标位置
//
// 步进电机 X 轴: 控制平板左右倾斜
// 步进电机 Y 轴: 控制平板前后倾斜
//
// 用法:
//   1. bal_init(&me, motor_x, motor_y, pid_x, pid_y);
//   2. ISR 中: bal_tick(&me, ball_x, ball_y);

#ifndef GEN_BALANCE_H
#define GEN_BALANCE_H

#include <stdint.h>
#include <stdbool.h>

// 类型来源 (原 opaque 前向声明与 canonical 头重复 → typedef redefinition)
#include "comp_step_motor.h"  // StepMotorBase (权威定义头, 消除重复 typedef)
#include "comp_pid.h"         // PidBase       (权威定义头, 消除重复 typedef)

// 球板状态
typedef enum {
  BAL_IDLE,  // 空闲 — 平台水平
  BAL_HOLD,  // 保持 — PID 锁定球在目标位置
  BAL_MOVE,  // 移动 — 球跟踪动态目标
} BalState;

// 球板 PID 参数
typedef struct {
  float kp, ki, kd;
  float out_limit;  // 步进电机步数输出限幅
} BalAxisCfg;

// GenBalance 实例结构体
typedef struct {
  StepMotorBase *mot_x;  // [必须] X 轴步进电机
  StepMotorBase *mot_y;  // [必须] Y 轴步进电机
  PidBase *pid_x;        // [必须] X 轴 PID (位置环)
  PidBase *pid_y;        // [必须] Y 轴 PID (位置环)

  BalState state;  // 当前状态
  float target_x;  // 球目标 X 坐标 (归一化, 0.0=中心)
  float target_y;  // 球目标 Y 坐标
  float ball_x;    // 球当前 X 坐标
  float ball_y;    // 球当前 Y 坐标

  // 步进电机步数范围 (平板物理极限)
  int32_t steps_x_min, steps_x_max;
  int32_t steps_y_min, steps_y_max;

  // 死区 (球在中心附近时不调整, 防抖动)
  float deadzone;
} GenBalance;

// ======== API ========

// 初始化: 绑定步进电机 + PID + 步数范围
void bal_init(GenBalance *me, StepMotorBase *mot_x, StepMotorBase *mot_y, PidBase *pid_x, PidBase *pid_y,
              int32_t steps_x_min, int32_t steps_x_max, int32_t steps_y_min, int32_t steps_y_max);

// 设置目标球位置
void bal_set_target(GenBalance *me, float x, float y);

// 更新球位置反馈 + 执行控制 (在摄像头帧到达时调用, 不在 ISR)
// ball_x/y: 归一化坐标 (0.0=中心, -1.0~+1.0 范围)
void bal_update(GenBalance *me, float ball_x, float ball_y);

// 停止: 平台回水平
void bal_stop(GenBalance *me);

// 状态查询
bool bal_is_active(GenBalance *me);

#endif
