#ifndef MOD_FOLLOWER_H
#define MOD_FOLLOWER_H

// 循迹控制模块 — Follower (Module 层)
// Follower (红外 → P2PD → 差速)
//       + Follower (tp=cp 同步 + task 独立参数)
//       + LESSONS #1 (不用陀螺仪)
//       + LESSONS #2 (P2PD 非线性 PID, 不用于线性 PID)
//       + LESSONS #8 (corr ≤ base - 1 限幅)
//       + LESSONS #9 (转弯后切回循迹 tp=cp 同步)
//       + LESSONS #10 (启动前 turn_cancel + route_stop)
//
// 核心数据流:
//   8路红外 → AdcBase.pos (-7~+7) → P2PD PID → corr (修正量)
//   → target_L = base + corr, target_R = base - corr
//   → motapp_set_speed(L, target_L), motapp_set_speed(R, target_R)
//
// 用法:
//   1. follower_init(&me, mtr_a, mtr_b, adc, pid_p2pd, turn);
//   2. follower_start(&me);
//   3. ISR 每 tick: follower_tick(&me);

#include <stdint.h>
#include <stdbool.h>

// 前向声明
typedef struct MotApp   MotApp;
typedef struct TurnCtrl TurnCtrl;
typedef struct AdcBase  AdcBase;
typedef struct PidBase  PidBase;

// 循迹状态
typedef enum {
  FLW_IDLE,    // 空闲 — 不输出电机控制
  FLW_RUNNING, // 循迹中 — P2PD 差速控制
} FlwState;

// Follower 实例结构体
typedef struct {
  MotApp   *mtr_a;           // [必须] 左电机 MotApp
  MotApp   *mtr_b;           // [必须] 右电机 MotApp
  AdcBase  *adc;             // [必须] 红外循迹传感器 (AdcFollower)

  PidBase  *pid_p2pd;        // [必须] P2PD 非线性 PID (LESSONS #2)
  TurnCtrl *turn;            // [必须] 转弯控制器 (互斥用, LESSONS #10)

  FlwState state;            // 当前状态
  int16_t  base_speed;       // 基础速度 (直道车速, 编码器增量/周期)
  float    corr_limit;       // 修正量限幅 = base_speed - 1 (LESSONS #8)
  int16_t  target_left;      // 左轮目标速度 (调试用)
  int16_t  target_right;     // 右轮目标速度 (调试用)
  int16_t  current_pos;      // 当前传感器位置偏差 (-7~+7)
  float    p2pd_output;      // P2PD 原始输出 (调试用)
  bool     tp_synced;        // 切回循迹时是否已同步 tp=cp
} Follower;

// ======== API ========

// 初始化: 绑定电机 + 传感器 + PID + 转弯控制
void follower_init(Follower *me, MotApp *mtr_a, MotApp *mtr_b,
                    AdcBase *adc, PidBase *pid_p2pd, TurnCtrl *turn);

// 启动循迹 (自动 cancel 转弯 + 同步 tp=cp)
void follower_start(Follower *me, int16_t base_speed);

// 停止循迹
void follower_stop(Follower *me);

// 每控制周期调用一次 (ISR 中)
// 内部: 读传感器位置 → P2PD 计算 → 差速修正 → 写电机
void follower_tick(Follower *me);

// 查询是否正在循迹
bool follower_is_running(Follower *me);

#endif
