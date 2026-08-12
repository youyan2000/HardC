// PowerStage — 电源拓扑控制模块父类接口 (Components 层)
//
// "采样 + 环路控制 + PWM 输出" 集成模块的抽象:
//   设备绑定: PwmBase* (输出) + AdcBase* (采样) + PidBase* loop[2] (外/内环, 可空)
//   状态机:   INIT → IDLE → RUN ⇄ FAULT (FAULT_HOLD / RECOVER 供扩展)
//   虚表:     init / tick / start / stop / emergency / set_ref / apply_tune / state
//
// 具体拓扑 (Buck/Boost/Flyback/...) 在 Module 层以 mod_* 实现:
//   示例: Module/power/mod_buck.{h,c} — 级联电压环→电流环 + 前馈
//
// 与 mod_powerctrl.h (Module 层状态机模板) 的分工:
//   PowerStage   — "可驱动"的设备绑定接口 (持有 PwmBase*/AdcBase*), 子类实现 ops
//   mod_powerctrl — "可继承"的状态机骨架 (值包含 Device, 用户子类化), 与设备解耦

#ifndef COMP_POWER_STAGE_H
#define COMP_POWER_STAGE_H

#include <stdint.h>
#include <stdbool.h>
#include "comp_pwm.h"    // PwmBase + pwm_emergency_stop (默认辅助用)
#include "comp_adc.h"    // AdcBase — 采样设备绑定
#include "comp_pid.h"    // PidBase — 环路绑定 (外环/内环)

typedef struct PowerStage PowerStage;

// ======== 电源控制状态 ========
typedef enum {
  PST_INIT,        // 初始化: 参数加载 / 自检
  PST_IDLE,        // 空闲: 等待 start 命令
  PST_RUN,         // 运行: 采样 → 环路 → PWM
  PST_FAULT_HOLD,  // 故障预判: 去抖动确认
  PST_FAULT,       // 故障: 封波 + 诊断 + 等待恢复
  PST_RECOVER,     // 恢复: 软启动重新进入 RUN
} PstSt;

// ======== 虚函数签名 (子类在 Module 层实现, 统一 PowerStage* 为第一参数) ========
typedef void  (*pst_init_fn)      (PowerStage *me);
typedef void  (*pst_tick_fn)      (PowerStage *me);
typedef void  (*pst_start_fn)     (PowerStage *me);
typedef void  (*pst_stop_fn)      (PowerStage *me);
typedef void  (*pst_emergency_fn) (PowerStage *me);
typedef void  (*pst_set_ref_fn)   (PowerStage *me, float vref, float iref);
typedef void  (*pst_apply_tune_fn)(PowerStage *me, const float coef[10]);
typedef PstSt (*pst_state_fn)     (PowerStage *me);

// ======== 虚函数表 ========
typedef struct {
  pst_init_fn       init;         // 复位到 INIT 态 (重初始化)
  pst_tick_fn       tick;         // 控制周期: 状态机 + 采样/环路/PWM
  pst_start_fn      start;        // IDLE/FAULT/RECOVER → RUN
  pst_stop_fn       stop;         // RUN → IDLE (正常停机)
  pst_emergency_fn  emergency;    // 任意态 → FAULT (立即封波)
  pst_set_ref_fn    set_ref;      // 热切换参考 vref/iref
  pst_apply_tune_fn apply_tune;   // 0xFB 调参: coef[10] → 槽位
  pst_state_fn      state;        // 查询当前状态
} PowerStageOps;

// ======== 基类结构体 — 设备绑定 + 公共运行参数 ========
struct PowerStage {
  const PowerStageOps *ops;       // 虚表指针 (子类 init 时绑定)
  PwmBase  *pwm;                  // PWM 输出设备
  AdcBase  *adc;                  // ADC 采样设备
  PidBase  *loop[2];              // 环路控制器: [0]=外环, [1]=内环 (可空)
  float     vref;                 // 电压参考 (V)
  float     iref;                 // 电流参考 (A)
  float     ovp;                  // 过压保护阈值 (V)
  float     ocp;                  // 过流保护阈值 (A)
  uint32_t  debounce_cnt;         // 故障去抖计数 (子类在 tick 中维护)
  PstSt     st;                   // 当前状态
};

// ======== 默认辅助 (static inline, 极简) ========

// 紧急封波: 置 FAULT 态 + 调 pwm_emergency_stop
static inline void power_stage_emergency(PowerStage *me) {
  if (me->pwm) {
    pwm_emergency_stop(me->pwm);
  }
  me->st           = PST_FAULT;
  me->debounce_cnt = 0;
}

// 查询当前状态
static inline PstSt power_stage_get_state(PowerStage *me) {
  return me->st;
}

#endif  // COMP_POWER_STAGE_H
