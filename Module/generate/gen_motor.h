// 电机应用模块 — MotApp (Module 层)
// 单电机状态机: IDLE / SPD(速度控制) / POS(位置控制) / SP(级联位置+速度)
// app_motor.h/c (MotApp 3-ops 虚表模式)
//       + 200 tick 超时 + CarConfig POD 注入
//
// 核心设计:
//   - 模块只通过 MotorBase* / PidBase* 操作下层, 不直接持有硬件指针 (LESSONS #40)
//   - 200 tick (2秒) 无新命令 → 自动 IDLE 防止失控
//   - 减速停 (M_IDLE=保持积分) 和急停 (清零积分) 分离 (LESSONS #4)
//
// 用法:
//   1. motapp_init(&me, motor, pid_vel, pid_pos, cascade);
//   2. ISR 每 tick: motapp_tick(&me);
//   3. 命令: motapp_set_speed(&me, 50) / motapp_set_position(&me, 1000)

#ifndef GEN_MOTOR_H
#define GEN_MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "comp_motor.h"
#include "comp_pid.h"
#include "pid_cascade.h"

// MotApp 状态枚举
typedef enum {
  MOT_IDLE,  // 空闲 — 电机输出 0, 积分保持不清零 (LESSONS #4)
  MOT_SPD,   // 速度模式 — 单速度环 PID: 目标→速度PID→PWM
  MOT_POS,   // 位置模式 — 单位置环 PID: 目标位置→位置PID→PWM, 到位自动→IDLE
  MOT_SP,    // 级联模式 — 外位置环→内速度环 (PidCascade)
} MotState;

// 超时 + 到达阈值
#define MOTAPP_TIMEOUT_TICKS 200  // 2 秒 @ 100Hz — 无命令后自动 IDLE
#define MOTAPP_POS_ARRIVE_TOL 5   // 位置到达容忍 (编码器 tick, 默认 ±5)

// MotApp 实例结构体
typedef struct MotApp {
  MotorBase *motor;         // [必须] 电机驱动句柄 (motor_write / motor_read)
  PidBase *pid_vel;         // [必须] 速度环 PID (SPD 模式, SP 模式内环)
  PidBase *pid_pos;         // [可选] 位置环 PID (POS 模式, NULL=不可用)
  PidCascade *pid_cascade;  // [可选] 级联 PID (SP 模式, NULL=不可用)

  MotState state;            // 当前状态
  int16_t target_speed;      // 目标速度 (编码器增量/周期, SPD 模式)
  int32_t target_position;   // 目标位置 (累计编码器脉冲, POS/SP 模式)
  int32_t current_position;  // 当前累计位置 (每 tick += read())
  int16_t current_speed;     // 当前速度 (上一 tick 的编码器增量)
  uint16_t timeout;          // 超时倒计时 — 0 时自动切 IDLE
  int16_t pwm_output;        // 最终 PWM 输出值 (调试/监控用)
  uint8_t pos_arrive_tol;    // 位置到达容忍 (可运行时调整, 默认 5)
} MotApp;

// ======== API ========

// 初始化: 绑定电机句柄 + PID 句柄 (pid_pos 和 pid_cascade 可选传 NULL)
void motapp_init(MotApp *me, MotorBase *motor, PidBase *pid_vel, PidBase *pid_pos, PidCascade *pid_cascade);

// 每控制周期调用一次 (ISR 中, 通常 100Hz / 10ms)
// 内部完成: 读编码器 → 状态分发 → PID 计算 → 写 PWM → 超时检测
void motapp_tick(MotApp *me);

// === 模式切换 ===

// 速度模式: 恒速运行 (target_speed 为正=前进, 负=后退)
void motapp_set_speed(MotApp *me, int16_t target_speed);

// 位置模式: 转到目标编码器脉冲数, 到达后自动切 IDLE
void motapp_set_position(MotApp *me, int32_t target_position);

// 级联模式: 外位置环 → 内速度环 (需要 pid_cascade 非 NULL)
void motapp_set_cascade(MotApp *me, int32_t target_position);

// 减速停: 目标速度=0, 状态→IDLE, 速度环积分保持不清零 (LESSONS #4)
void motapp_stop(MotApp *me);

// 急停: 输出=0, 状态→IDLE, 清零所有 PID 状态
void motapp_estop(MotApp *me);

#endif
