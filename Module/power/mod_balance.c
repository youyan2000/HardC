// 球板平衡模块 — ModBalance (Module 层实现)
// mod_balance.c (2×PidLinear + StepMotor)
//
// 控制架构:
//   摄像头球坐标 (x,y) → X/Y 轴 PID → 步进电机步数 → 丝杆行程
//   X 轴 PID 计算 → stepmotor_set_steps(mot_x, delta_steps)
//   Y 轴 PID 计算 → stepmotor_set_steps(mot_y, delta_steps)
//
// 注意: 球板控制是相对调节 — 不设绝对位置, 每次只调整偏差量

#include "mod_balance.h"
#include "comp_pid.h"      // pid_compute — balance 调用 PidBase 计算
#include "comp_step_motor.h"
#include "comp_math.h"   // math_clamp_f, math_deadzone_f

// ======== 初始化 ========

void bal_init(ModBalance *me, StepMotorBase *mot_x, StepMotorBase *mot_y,
               PidBase *pid_x, PidBase *pid_y,
               int32_t steps_x_min, int32_t steps_x_max,
               int32_t steps_y_min, int32_t steps_y_max) {
  me->mot_x  = mot_x;
  me->mot_y  = mot_y;
  me->pid_x  = pid_x;
  me->pid_y  = pid_y;

  me->state    = BAL_IDLE;
  me->target_x = 0.0f;
  me->target_y = 0.0f;
  me->ball_x   = 0.0f;
  me->ball_y   = 0.0f;

  me->steps_x_min = steps_x_min;
  me->steps_x_max = steps_x_max;
  me->steps_y_min = steps_y_min;
  me->steps_y_max = steps_y_max;
  me->deadzone    = 0.02f;  // 默认 2% 死区
}

// ======== 设置目标 ========

void bal_set_target(ModBalance *me, float x, float y) {
  me->target_x = math_clamp_f(x, -1.0f, 1.0f);
  me->target_y = math_clamp_f(y, -1.0f, 1.0f);
  if (me->state == BAL_IDLE) {
    me->state = BAL_HOLD;
  }
}

// ======== 更新 (每帧调用, 不在 ISR) ========

void bal_update(ModBalance *me, float ball_x, float ball_y) {
  me->ball_x = ball_x;
  me->ball_y = ball_y;

  if (me->state == BAL_IDLE) return;

  // X 轴: 计算误差 → PID → 步进电机步数
  float err_x = me->target_x - ball_x;
  // 死区: 球在中心附近时不调整 (防微抖动)
  err_x = math_deadzone_f(err_x, me->deadzone);

  if (me->pid_x && me->mot_x) {
    float pid_out_x = pid_compute(me->pid_x, 0.0f, err_x);

    // PID 输出 = 相对步数调整, 限幅到步进范围
    int32_t delta_x = (int32_t)pid_out_x;
    int32_t cur_pos_x = stepmotor_get_steps(me->mot_x);
    int32_t new_pos_x = cur_pos_x + delta_x;

    // 软限位: 丝杆物理行程
    new_pos_x = math_clamp_i32(new_pos_x, me->steps_x_min, me->steps_x_max);
    stepmotor_set_steps(me->mot_x, new_pos_x - cur_pos_x);
  }

  // Y 轴: 同理
  float err_y = me->target_y - ball_y;
  err_y = math_deadzone_f(err_y, me->deadzone);

  if (me->pid_y && me->mot_y) {
    float pid_out_y = pid_compute(me->pid_y, 0.0f, err_y);

    int32_t delta_y = (int32_t)pid_out_y;
    int32_t cur_pos_y = stepmotor_get_steps(me->mot_y);
    int32_t new_pos_y = cur_pos_y + delta_y;

    new_pos_y = math_clamp_i32(new_pos_y, me->steps_y_min, me->steps_y_max);
    stepmotor_set_steps(me->mot_y, new_pos_y - cur_pos_y);
  }
}

// ======== 停止 ========

void bal_stop(ModBalance *me) {
  me->state = BAL_IDLE;
  // 平台不回水平 — 由应用层决定是否归零
}

bool bal_is_active(ModBalance *me) {
  return (me->state != BAL_IDLE);
}
