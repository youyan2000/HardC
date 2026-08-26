// 转弯控制模块 — TurnCtrl (Module 层实现)
// TurnCtrl (tick 计数转弯)
//       + LESSONS #3 (固定 tick, 不用陀螺仪)
//       + LESSONS #4 (rotate_stop 用 M_IDLE, 不清积分)
//       + LESSONS #5 (→LF_RUN 过渡必须清级联 PID, 由调用者负责)
//       + LESSONS #10 (启动转弯前必须 cancel 循迹/盲跑, 由调用者负责)

#include "gen_turn.h"
#include "gen_motor.h"

// ======== 查找表: 转弯类型 → tick 数 ========

static int32_t turn_ticks_for_type(TurnType type) {
  if (type == TURN_180)
    return TURN_TICK_180;
  return TURN_TICK_90;  // TURN_90_LEFT 和 TURN_90_RIGHT 都是 90°
}

// ======== 初始化 ========

void turnctrl_init(TurnCtrl *me, MotApp *mtr_a, MotApp *mtr_b) {
  me->mtr_a = mtr_a;
  me->mtr_b = mtr_b;
  me->state = TURN_IDLE;
  me->type = TURN_90_LEFT;
  me->turn_pwm = 0;
  me->target_ticks = 0;
  me->accumulated = 0;
  me->brake_ticks = 0;
  me->a_forward = true;
  me->b_forward = true;
}

// ======== 触发转弯 ========

void turnctrl_execute(TurnCtrl *me, TurnType type, int16_t pwm) {
  me->type = type;
  me->turn_pwm = pwm;
  me->target_ticks = turn_ticks_for_type(type);
  me->accumulated = 0;
  me->brake_ticks = 0;

  // 差速方向: 电机A 正向, 电机B 反向 → 原地左转/右转
  // TURN_90_LEFT:  A=反转(后退), B=正转(前进) → 向左自旋
  // TURN_90_RIGHT: A=正转(前进), B=反转(后退) → 向右自旋
  // TURN_180:      默认左转方向掉头
  switch (type) {
  case TURN_90_LEFT:
  case TURN_180:
    // 左转 和 180° 掉头: A 后退, B 前进 (向左自旋)
    // 区别仅在于 tick 数 (TURN_TICK_90 vs TURN_TICK_180)
    me->a_forward = false;
    me->b_forward = true;
    break;
  case TURN_90_RIGHT:
    // 右转: A 前进, B 后退 (向右自旋)
    me->a_forward = true;
    me->b_forward = false;
    break;
  }

  // 启动电机: 固定差速 PWM (经 MotApp 速度 PID 跟踪)
  int16_t spd_a = me->a_forward ? pwm : (int16_t) (-pwm);
  int16_t spd_b = me->b_forward ? pwm : (int16_t) (-pwm);
  motapp_set_speed(me->mtr_a, spd_a);
  motapp_set_speed(me->mtr_b, spd_b);

  me->state = TURN_RUNNING;
}

// ======== 取消转弯 ========

void turnctrl_cancel(TurnCtrl *me) {
  motapp_estop(me->mtr_a);
  motapp_estop(me->mtr_b);
  me->state = TURN_IDLE;
  me->accumulated = 0;
  me->brake_ticks = 0;
}

// ======== 每 tick 更新 ========

void turnctrl_tick(TurnCtrl *me) {
  switch (me->state) {
  case TURN_IDLE:
    return;

  case TURN_RUNNING: {
    // 读取两侧编码器增量 (motapp_tick 已经执行了 motor_read, 这里读 current_speed)
    // current_speed = 上一 tick 的编码器增量, 取绝对值累加
    int16_t da = me->mtr_a->current_speed;
    int16_t db = me->mtr_b->current_speed;
    if (da < 0)
      da = (int16_t) (-da);
    if (db < 0)
      db = (int16_t) (-db);
    // 取两侧较大者作为转弯进度 (防止一侧打滑导致提前结束)
    me->accumulated += (da > db) ? da : db;

    // 检查是否到达目标 tick 数
    if (me->accumulated >= me->target_ticks) {
      // 转入制动阶段: 速度目标=0, 不清积分 (LESSONS #4)
      motapp_stop(me->mtr_a);
      motapp_stop(me->mtr_b);
      me->state = TURN_BRAKING;
      me->brake_ticks = TURN_BRAKE_TICKS;
    }
    break;
  }

  case TURN_BRAKING:
    // 固定制动时长后 → IDLE
    if (me->brake_ticks > 0) {
      me->brake_ticks--;
    }
    if (me->brake_ticks == 0) {
      me->state = TURN_IDLE;
    }
    break;
  }
}

// ======== 状态查询 ========

bool turnctrl_is_done(TurnCtrl *me) {
  return (me->state == TURN_IDLE);
}

bool turnctrl_is_active(TurnCtrl *me) {
  return (me->state == TURN_RUNNING || me->state == TURN_BRAKING);
}
