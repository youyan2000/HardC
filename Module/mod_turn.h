#ifndef MOD_TURN_H
#define MOD_TURN_H

// 转弯控制模块 — TurnCtrl (Module 层)
// 来源: LitteCar_STM32 TurnCtrl (编码器 tick 计数转弯)
//       + Car_Control_Study_Report §7.3 (3 种转弯: UTURN/LTURN/RTURN)
//       + LESSONS #3 (不用陀螺仪, 用 tick 计数)
//       + LESSONS #4 (制动不清积分)
//       + LESSONS #5 (切回循迹前清级联 PID)
//
// 核心设计:
//   - 差速转弯: 左轮+PWM, 右轮-PWM (或反过来), 编码器 tick 计数
//   - 转弯完成后自动进入制动阶段 → IDLE
//   - 制动阶段速度目标=0, 不清速度环积分 (制动自然均匀)
//
// 用法:
//   1. turnctrl_init(&me, mtr_a, mtr_b);
//   2. 触发: turnctrl_execute(&me, TURN_90_LEFT, pwm);
//   3. ISR 每 tick: turnctrl_tick(&me);

#include <stdint.h>
#include <stdbool.h>

// 前向声明
typedef struct MotApp MotApp;

// 转弯类型
typedef enum {
  TURN_90_LEFT,   // 左转 90° (原地左转)
  TURN_90_RIGHT,  // 右转 90° (原地右转)
  TURN_180,       // 180° 掉头
} TurnType;

// 转弯状态
typedef enum {
  TURN_IDLE,      // 空闲 — 无转弯
  TURN_RUNNING,   // 转弯中 — 差速 PWM + tick 计数
  TURN_BRAKING,   // 制动中 — 速度=0, 保持积分
} TurnSt;

// TurnCtrl 实例结构体
typedef struct {
  MotApp   *mtr_a;           // [必须] 左电机 MotApp 句柄
  MotApp   *mtr_b;           // [必须] 右电机 MotApp 句柄

  TurnSt   state;            // 当前状态
  TurnType type;             // 当前转弯类型
  int16_t  turn_pwm;         // 转弯时固定 PWM 值

  int32_t  target_ticks;     // 目标编码器 tick 数 (由类型查表得到)
  int32_t  accumulated;      // 已累计 tick 数 (取两侧电机 tick 较大者)
  int16_t  brake_ticks;      // 制动阶段剩余 tick 数
  bool     a_forward;        // 电机A 正向? (true=PWM正, false=PWM负)
  bool     b_forward;        // 电机B 正向?
} TurnCtrl;

// ======== 转弯 tick 经验值 (编码器脉冲/轮圈) ========
// 来源: LESSONS #3 — 固定 tick 计数, 不用陀螺仪
//        TURN_90_TICKS → 90° 直角弯 @ spd=90
//        TURN_180_TICKS → 180° 掉头 @ spd=90
// 实际值因车轮直径/编码器线数而异, 需实验标定
#define TURN_TICK_90   28    // 90° 转弯 tick 数 (参考值)
#define TURN_TICK_180  55    // 180° 掉头 tick 数 (参考值)
#define TURN_BRAKE_TICKS 10   // 制动阶段固定时长 (100ms @ 100Hz)

// ======== API ========

// 初始化: 绑定左右电机 MotApp
void turnctrl_init(TurnCtrl *me, MotApp *mtr_a, MotApp *mtr_b);

// 触发转弯: type = 转弯类型, pwm = 转弯时固定 PWM 值
// 调用前应确保 follower_stop / route_stop 已执行 (LESSONS #10)
void turnctrl_execute(TurnCtrl *me, TurnType type, int16_t pwm);

// 取消当前转弯: 急停 → IDLE
void turnctrl_cancel(TurnCtrl *me);

// 每控制周期调用一次 (ISR 中)
// 内部: 读编码器 → tick 累加 → 到达检测 → 状态转换
void turnctrl_tick(TurnCtrl *me);

// 查询转弯是否完成 (已完成 = IDLE 且制动结束)
bool turnctrl_is_done(TurnCtrl *me);

// 查询是否正在转弯 (RUNNING 或 BRAKING)
bool turnctrl_is_active(TurnCtrl *me);

#endif
