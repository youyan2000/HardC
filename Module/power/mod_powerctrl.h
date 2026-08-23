// 功率控制状态机模板 — PWM-OOP Module 层
// Module 层状态机模式
//
// 状态: INIT → IDLE → RUN → FAULT_HOLD → FAULT → RECOVER
//
// 设计要点:
//   - 状态机是通用手段, 不限于 Module 层 — 但 Module 层最常用
//   - Device 值包含 (自持有) + Component 值包含 (PID等)
//   - Module_Status* 指针注入 (与其他模块共享状态)
//   - Module_Conn* 指针注入 (通信接口)
//   - 关键保护: 模式切换滞回 / 单周期同步 / 去抖动错误检测 / PWM 输出限幅
//
// 继承方式 (用户自建子类):
//   typedef struct {
//     ModPwr          base;        // ← 父类必须是第一个成员
//     Device_BuckBoost buckboost;  // 值包含具体 Device
//     Component_PID    pid_v;      // 值包含具体 PID
//     LowPassFilter    filt;       // 值包含滤波器
//   } MyPowerCtrl;

#ifndef MOD_POWERCTRL_H
#define MOD_POWERCTRL_H

#include <stdbool.h>
#include <stdint.h>

// 类型来源 (原 opaque 前向声明与 canonical 头重复 → typedef redefinition)
#include "comp_pwm.h"   // PwmBase  (权威定义头, 消除重复 typedef)
#include "comp_pid.h"   // PidBase  (权威定义头, 消除重复 typedef)
struct LowPassFilter;   // 非 typedef, 保留原样

// ======== 功率控制状态 ========
typedef enum {
  PCTRL_INIT,        // 初始化: 配置PWM/ADC/PID, 自检
  PCTRL_IDLE,        // 空闲: 等待启动命令, 心跳/状态上报
  PCTRL_RUN,         // 运行: 主控制循环 (采样→滤波→PID→PWM)
  PCTRL_FAULT_HOLD,  // 故障预判: 去抖动确认后才进入 FAULT
  PCTRL_FAULT,       // 故障: 封波 + 诊断记录 + 等待恢复
  PCTRL_RECOVER,     // 恢复: 故障清除后逐步重启 (软启动)
} PwrSt;

// ======== 功率控制模块参数 (YAML 可配置) ========
typedef struct {
  float vref;              // 目标电压/电流
  float fault_debounce_s;  // 故障去抖动时间 (秒)
  float recover_delay_s;   // 恢复延迟 (秒)
  float v_enter_bb;        // 进入 BuckBoost 窗口 (如 1.03)
  float v_exit_bb;         // 退出 BuckBoost 窗口 (如 1.10)
  float soft_start_step;   // 软启动步长 (占空比增量/tick)
} ModPwr_Param;

// ======== 功率控制模块 (父类模板) ========
typedef struct {
  // --- 状态机 ---
  PwrSt st;           // 当前状态
  PwrSt st_prev;      // 上一状态 (模式切换单周期同步用)

  // --- 值包含: 用户继承时嵌入具体实例 (注释仅示意) ---
  // Device_BuckBoost  buckboost_;   // 用户自嵌
  // Component_PID     pid_v_;       // 用户自嵌
  // LowPassFilter     filt_v_;      // 用户自嵌 (可选)

  // --- 指针注入: 与其他模块共享 (用户构造时绑定) ---
  // Module_Status    *status_;      // 共享状态 (错误码/心跳)
  // Module_Conn      *conn_;        // 通信接口 (CAN/串口)
  // Module_SampleMgr *sampler_;     // 采样管理

  // --- 运行时变量 ---
  ModPwr_Param param;              // 可热替换配置
  uint32_t     tick_cnt;           // tick 计数
  uint32_t     fault_debounce_cnt; // 故障去抖动计数
  float        last_duty;          // 上次占空比 (模式切换单周期同步用)
} ModPwr;

// ======== API ========

// 构造: 初始化状态机和配置
void  mod_pwr_init(ModPwr *me, const ModPwr_Param *param);

// 每 tick 驱动状态机 (由定时器 ISR 或 App 主循环调用)
void  mod_pwr_tick(ModPwr *me, float dt);

// 启动功率控制: IDLE / RECOVER → RUN
void  mod_pwr_start(ModPwr *me);

// 正常停机: 斜坡降输出 → 封波 → IDLE
void  mod_pwr_stop(ModPwr *me);

// 紧急封波: 立即清零 PWM, 不经过斜坡
void  mod_pwr_emergency_stop(ModPwr *me);

// 查询是否处于故障状态
bool  mod_pwr_is_fault(const ModPwr *me);

// 获取当前状态
PwrSt mod_pwr_get_state(const ModPwr *me);

#endif
