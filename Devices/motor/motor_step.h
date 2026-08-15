// 步进电机驱动 — StepMotorBase 的子类 (Devices 层)
//       + motor_step.h/c (OOP 版, BSP 函数指针注入)
//       + user_step_motor.h/c (原始 4 相 A→C→B→D)
//
// 硬件: 4 相步进电机 (A/B/C/D) + DIR 方向引脚 + PUL 脉冲定时器
// 驱动模式: 全步进 4 拍 (A→C→B→D)
//           半步进 8 拍可选 (phase_tbl 注入)
//
// Bug 规避:
//   - ✅ P0#1: 所有状态在结构体成员, 无 static 局部变量
//   - ✅ P0#2: 速度控制 → bsp_step_pul_set_period → 定时器 LOAD 寄存器
//   - ✅ P1#4: ctx 参数被实际使用, 不丢弃
//   - ✅ P2#5: ramp_step 加减速占位
//   - ✅ P2#7: pos_limit_lo/hi 软限位

#ifndef MOTOR_STEP_H
#define MOTOR_STEP_H

#include "comp_step_motor.h"
#include "bsp_step_motor.h"

// 步进电机实例名
typedef enum {
  StepMotX,  // X 轴
  StepMotY,  // Y 轴
} StepMotName;

// BSP 操作函数指针 (平台注入)
typedef void (*step_dir_set_fn)(void *ctx, bool high);
typedef void (*step_pul_period_fn)(void *ctx, uint16_t period);
typedef void (*step_pul_enable_fn)(void *ctx, bool enable);

// 步进电机构造配置 POD
typedef struct {
  StepMotName        name;           // 实例名
  const BspStepDir  *phase_pins;     // 4 相引脚 {A, B, C, D}
  step_dir_set_fn    set_dir;        // DIR 引脚函数
  void              *dir_ctx;        // DIR ctx (BspStepDir*)
  step_pul_period_fn set_period;     // PUL 周期函数
  void              *pul_ctx;        // PUL ctx (BspStepPul*)
  step_pul_enable_fn set_enable;     // PUL 使能函数
  const StepPhaseTable *phase_tbl;   // 初始相序表
  int32_t            limit_lo;       // 软限位下限
  int32_t            limit_hi;       // 软限位上限
} StepMotorCfg;

// StepMotor 子类结构体
typedef struct {
  StepMotorBase      base;           // 基类 (必须第一个, container_of 依赖)

  // BSP 注入 (硬件操作)
  const BspStepDir  *phase_pins;     // 4 相 {A, B, C, D}
  step_dir_set_fn    set_dir;        // DIR 引脚操作
  void              *dir_ctx;        // DIR 引脚 ctx (BspStepDir*)
  step_pul_period_fn set_period;     // PUL 周期操作
  void              *pul_ctx;        // PUL 定时器 ctx (BspStepPul*)
  step_pul_enable_fn set_enable;     // PUL 使能操作

  // 去抖 / 锁止控制
  uint16_t  lock_cnt;                // 静止计时 (100 tick 后自动关相, 防发热)
  int8_t    last_dir;                // 上一拍方向 (方向变化检测)
  bool      locked;                  // 锁止状态: true=全相关断

  // 加减速状态
  bool      ramping;                 // 正在 ramp 中
} StepMotor;

// ======== 构造 ========

// 初始化步进电机: 绑定 BSP 函数指针 + ctx + 相位表
// cfg 中的所有函数指针和 ctx 必须有效 (Bug #4: ctx 必须被使用)
void stepmotor_init(StepMotor *me, const StepMotorCfg *cfg);

// ======== 每 tick 更新 (ISR 中调用, 由定时器中断驱动) ========

// 步进脉冲中断服务: 推进一相 → 递减 steps → ramp 速度 → 锁止检测
// 调用频率 = 脉冲频率 (由定时器周期决定)
// 注意: 此函数在定时器 ISR 中调用, 必须快速返回
void stepmotor_isr_tick(StepMotor *me);

#endif
