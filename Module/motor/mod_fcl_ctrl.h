// 快速电流环 (Fast Current Loop) — dq 旋转坐标系高带宽电流控制
//
// 来源: TI controlSUITE motor_control/libs/FCL
// 翻译为 HardC Module 层纯C float 版本
//
// 算法: dq 轴双环 PI 电流控制 + 交叉解耦 + 反电动势前馈 + 有源阻尼
//
// 典型 ISR 调用:
//   // Park 变换后得到 i_d, i_q
//   fcl_run(&fcl, i_d, i_q, i_d_ref, i_q_ref, omega_e, v_bus);
//   // 用 fcl.v_d_ref, fcl.v_q_ref 做反 Park → αβ → SVPWM
//
// 主循环:
//   定时检查 fcl_get_fault(&fcl), 必要时调用 fcl_clear_fault(&fcl)

#ifndef MOD_FCL_CTRL_H
#define MOD_FCL_CTRL_H

#include <stdbool.h>
#include <stdint.h>
#include "pid_parallel.h"  // dq 轴电流环 PI (PidParallel: 积分钳位走 i_limit, 输出不限幅)

// FCL 运行模式
typedef enum {
  FclMode_Idle,       // 空闲, PWM=0
  FclMode_Enabled,    // 正常电流控制
  FclMode_Fault,      // 故障锁定, 需手动清除
} FclMode;

// FCL 用户配置 POD (YAML 注入目标, 纯数据)
typedef struct {
  float dt;              // ISR 周期 (s)
  float ld;              // d 轴电感 (H)
  float lq;              // q 轴电感 (H)
  float rs;              // 定子电阻 (Ω)
  float flux_pm;         // 永磁磁链 (Wb)
  float v_dc_max;        // 直流母线电压 (V), 用于限幅
  float i_max;           // 最大电流 (A)
  float kp_d;            // d 轴电流环 Kp
  float ki_d;            // d 轴电流环 Ki
  float kp_q;            // q 轴电流环 Kp
  float ki_q;            // q 轴电流环 Ki
  float kp_damp;         // 有源阻尼增益 (可选, 0=禁用)
} FclCfg;

// FCL 运行时 Instance (有状态 — PID 积分器 + 滤波)
typedef struct {
  FclCfg  cfg;           // 配置副本 (apply_config 同步)
  FclMode mode;          // 当前模式

  // dq 轴电流环 PI (PidParallel) — 积分器钳位走 i_limit (= min(v_bus, v_dc_max), 每 tick 更新),
  // 输出不限幅: 电压圆限制在 fcl_run 自身 (不是 PI 抗饱和, 见 .c 注释)
  PidParallel pi_d;
  PidParallel pi_q;

  // 输出电压 (dq 轴, V) — 调试可见
  float v_d_ref;
  float v_q_ref;

  // 电流测量缓存 (调试用)
  float i_d_meas;
  float i_q_meas;
  float i_d_ref;
  float i_q_ref;

  // 故障检测
  uint16_t fault_code;         // 故障码 bitmask
  int      overcurrent_cnt;    // 过流去抖计数
} FclCtrl;

// ======== 初始化 & 控制 ========

// 初始化 — 绑定配置, 清零状态
void fcl_init(FclCtrl *me, const FclCfg *cfg);

// 重置 — 清积分器和故障, 回到 Idle
void fcl_reset(FclCtrl *me);

// 使能
void fcl_enable(FclCtrl *me);

// 禁用
void fcl_disable(FclCtrl *me);

// ======== ISR 调用 (热路径, 禁止 printf) ========

// 单步执行 — 每 PWM 周期在 ISR 中调用
//   i_d, i_q: 测量电流 (Park 变换后)
//   i_d_ref, i_q_ref: 目标电流
//   omega_e: 电角速度 (rad/s), 来自观测器/编码器
//   v_bus: 直流母线电压 (V)
// 输出: me->v_d_ref, me->v_q_ref (需做反 Park → αβ → SVPWM)
void fcl_run(FclCtrl *me, float i_d, float i_q,
             float i_d_ref, float i_q_ref,
             float omega_e, float v_bus);

// ======== 主循环调用 (慢速路径) ========

// 清除故障锁存
void fcl_clear_fault(FclCtrl *me);

// 获取故障码
uint16_t fcl_get_fault(const FclCtrl *me);

#endif  // MOD_FCL_CTRL_H
