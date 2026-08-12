// 电机应用模块 — MotApp (Module 层实现)
// 来源: LitteCar_STM32 app_motor.c mot_tick() 状态机
//       + 3507_2026_eugene 200 tick 超时 + apply_car_cfg 同步
//       + Car_Control_Study_Report §7.1 (4 状态 + 级联 PID)
//
// 架构约束:
//   - Module 层不直接操作硬件寄存器, 全部通过 MotorBase*/PidBase* (LESSONS #40)
//   - 读编码器 (motor_read) 时自动清零 → 每 tick 天然微分
//   - 位置 = 累计增量 → 开环位置估计 (DC 电机无绝对编码器)
//   - LESSONS #4: 减速停不清积分, 只有急停才清零

#include "mod_motor.h"
#include "comp_math.h"   // math_clamp_f
#include <stddef.h>      // NULL

// ======== 初始化 ========

void motapp_init(MotApp *me, MotorBase *motor, PidBase *pid_vel,
                  PidBase *pid_pos, PidCascade *pid_cascade) {
  me->motor        = motor;
  me->pid_vel      = pid_vel;
  me->pid_pos      = pid_pos;
  me->pid_cascade  = pid_cascade;
  me->state        = MOT_IDLE;
  me->target_speed = 0;
  me->target_position = 0;
  me->current_position = 0;
  me->current_speed    = 0;
  me->timeout       = 0;
  me->pwm_output    = 0;
  me->pos_arrive_tol = MOTAPP_POS_ARRIVE_TOL;
}

// ======== 核心 tick ========

void motapp_tick(MotApp *me) {
  // 1. 读编码器增量 (读后自动清零, 天然微分)
  int16_t enc_delta = motor_read(me->motor);
  me->current_speed = enc_delta;
  me->current_position += enc_delta;

  // 2. 超时检测: >0 时每 tick 递减, 归零时自动切 IDLE
  if (me->timeout > 0) {
    me->timeout--;
    if (me->timeout == 0) {
      me->state = MOT_IDLE;
      me->target_speed = 0;
    }
  }

  // 3. 状态分发
  float pid_out = 0.0f;

  switch (me->state) {
  case MOT_IDLE:
    // 速度环 PID 目标=0, 保持积分不清零 (LESSONS #4)
    // 让电机自然减速, 积分项收敛到维持零速的值
    pid_out = pid_compute(me->pid_vel, 0.0f, (float)me->current_speed);
    break;

  case MOT_SPD:
    // 单速度环: 目标速度 → 速度 PID → PWM
    pid_out = pid_compute(me->pid_vel, (float)me->target_speed,
                          (float)me->current_speed);
    break;

  case MOT_POS:
    // 单位置环: 目标位置 → 位置 PID → PWM
    // 需要 pid_pos 非 NULL, 否则回退 IDLE
    if (me->pid_pos) {
      pid_out = pid_compute(me->pid_pos, (float)me->target_position,
                            (float)me->current_position);
      // 到位检测: 误差在容忍范围内 → 自动切 IDLE
      int32_t pos_err = me->target_position - me->current_position;
      if (pos_err < 0) pos_err = -pos_err;
      if (pos_err <= me->pos_arrive_tol) {
        me->state = MOT_IDLE;
        me->target_speed = 0;
      }
    } else {
      me->state = MOT_IDLE;
    }
    break;

  case MOT_SP:
    // 级联模式: 外位置环 → 内速度环 (PidCascade 编排)
    // 需要 pid_cascade 非 NULL
    if (me->pid_cascade) {
      pid_out = pid_cascade_compute(me->pid_cascade,
                                     (float)me->target_position,
                                     (float)me->current_position,
                                     (float)me->current_speed);
    } else {
      me->state = MOT_IDLE;
    }
    break;
  }

  // 4. 写 PWM (motor_write 内部处理 inv 翻转和 ±7200 限幅, LESSONS #7)
  int16_t pwm = (int16_t)pid_out;
  me->pwm_output = pwm;
  motor_write(me->motor, pwm);
}

// ======== 模式切换 ========

void motapp_set_speed(MotApp *me, int16_t target_speed) {
  me->target_speed = target_speed;
  me->state   = MOT_SPD;
  me->timeout = MOTAPP_TIMEOUT_TICKS;
}

void motapp_set_position(MotApp *me, int32_t target_position) {
  if (me->pid_pos == NULL) return;  // 未配置位置环, 忽略
  me->target_position = target_position;
  me->state   = MOT_POS;
  me->timeout = MOTAPP_TIMEOUT_TICKS;
}

void motapp_set_cascade(MotApp *me, int32_t target_position) {
  if (me->pid_cascade == NULL) return;  // 未配置级联, 忽略
  me->target_position = target_position;
  me->state   = MOT_SP;
  me->timeout = MOTAPP_TIMEOUT_TICKS;
}

// ======== 停车 ========

void motapp_stop(MotApp *me) {
  // 减速停: 目标速度=0, 不清积分 (LESSONS #4)
  // 速度环积分自然收敛到零速, 制动平滑
  me->target_speed = 0;
  me->state   = MOT_IDLE;
  me->timeout = 0;
}

void motapp_estop(MotApp *me) {
  // 急停: 输出=0, 清零所有 PID 状态
  me->target_speed  = 0;
  me->state    = MOT_IDLE;
  me->timeout  = 0;
  me->pwm_output = 0;
  motor_write(me->motor, 0);
  // 清零速度环积分, 防止下次启动跳变
  if (me->pid_vel) pid_reset(me->pid_vel);
  if (me->pid_pos) pid_reset(me->pid_pos);
  if (me->pid_cascade) pid_cascade_reset(me->pid_cascade);
}
