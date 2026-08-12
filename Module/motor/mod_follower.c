// 循迹控制模块 — Follower (Module 层实现)
// 来源: LitteCar_STM32 Follower (红外 → P2PD → 差速)
//       + 3507_2026_eugene Follower (task 独立参数 + ModProtocol 命令)
//       + Car_Control_Study_Report §7.2 (P2PD 算法 + 8路红外加权)
//       + LESSONS #1 (循迹不用陀螺仪)
//       + LESSONS #2 (用 P2PD 不用线性 PID)
//       + LESSONS #8 (|corr| ≤ base-1, 防止一侧反转)
//       + LESSONS #9 (转弯后切回: tp=cp 同步, prev_error=0)
//       + LESSONS #10 (启动前 turn_cancel + route_stop)

#include "mod_follower.h"
#include "mod_motor.h"
#include "mod_turn.h"
#include "comp_adc.h"
#include "comp_pid.h"
#include "comp_math.h"   // math_clamp_f

// ======== 初始化 ========

void follower_init(Follower *me, MotApp *mtr_a, MotApp *mtr_b,
                    AdcBase *adc, PidBase *pid_p2pd, TurnCtrl *turn) {
  me->mtr_a     = mtr_a;
  me->mtr_b     = mtr_b;
  me->adc       = adc;
  me->pid_p2pd  = pid_p2pd;
  me->turn      = turn;
  me->state     = FLW_IDLE;
  me->base_speed   = 0;
  me->corr_limit   = 0.0f;
  me->target_left  = 0;
  me->target_right = 0;
  me->current_pos  = 0;
  me->p2pd_output  = 0.0f;
  me->tp_synced    = false;
}

// ======== 启动/停止 ========

void follower_start(Follower *me, int16_t base_speed) {
  // LESSONS #10: 启动循迹前取消其他控制源
  if (me->turn) {
    turnctrl_cancel(me->turn);
  }
  // route_stop() — 如果有盲跑模块, 在此处调用 (暂无)

  // LESSONS #9: 转弯后切回循迹必须同步 tp=cp
  // tp(目标位置=0, 即线在正中间) vs cp(传感器当前值)
  // 通过重置 P2PD 的 prev_error=0 来消除 D 项突变
  if (me->pid_p2pd) {
    pid_reset(me->pid_p2pd);
  }

  me->base_speed = base_speed;
  // LESSONS #8: 修正量不能超过基础速度, 留 1 余量
  me->corr_limit = (float)(base_speed - 1);
  if (me->corr_limit < 1.0f) me->corr_limit = 1.0f;

  me->tp_synced = true;
  me->state = FLW_RUNNING;
}

void follower_stop(Follower *me) {
  motapp_stop(me->mtr_a);
  motapp_stop(me->mtr_b);
  me->state     = FLW_IDLE;
  me->tp_synced = false;
}

// ======== 核心 tick ========

void follower_tick(Follower *me) {
  if (me->state != FLW_RUNNING) return;

  // 1. 读传感器位置偏差 (AdcFollower.process 已更新 base.pos, 范围 -7~+7)
  me->current_pos = me->adc->pos;

  // 2. P2PD 计算修正量
  // target=0 (线在正中), measure=current_pos (传感器偏差)
  float corr = 0.0f;
  if (me->pid_p2pd) {
    corr = pid_compute(me->pid_p2pd, 0.0f, (float)me->current_pos);
    me->p2pd_output = corr;
  }

  // 3. LESSONS #8: |corr| ≤ base_speed - 1
  // 确保两侧目标速度始终 ≥ 1 (始终前进, 不会单侧反转)
  corr = math_clamp_f(corr, -me->corr_limit, me->corr_limit);

  // 4. 差速修正: 左轮 +corr, 右轮 -corr
  float target_l = (float)me->base_speed + corr;
  float target_r = (float)me->base_speed - corr;

  // 再次防护: 速度不能为负 (LESSONS #8 已保证, 这里再兜底)
  if (target_l < 0.0f) target_l = 0.0f;
  if (target_r < 0.0f) target_r = 0.0f;

  me->target_left  = (int16_t)target_l;
  me->target_right = (int16_t)target_r;

  // 5. 写入电机速度目标
  motapp_set_speed(me->mtr_a, me->target_left);
  motapp_set_speed(me->mtr_b, me->target_right);
}

// ======== 状态查询 ========

bool follower_is_running(Follower *me) {
  return (me->state == FLW_RUNNING);
}
