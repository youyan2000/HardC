// 三相均流模块 — 并联相电流均衡 + Buck/Boost/BuckBoost 模式迟滞 (Module 层, ctx fast)
//
// 职责 (每 FAST tick, 与 mod_supercap 同源频率):
//   1. 电压比 ratio = vb/va → 模式迟滞 (Hysteresis: 0.97/1.03 进, 0.90/1.10 出)
//   2. ibase = paside / num_phases — 每相基准电流 (supercap 注入 paside)
//   3. 每相 share_error = clamp(share_gain × (iavg − iphase), ±share_lim)
//      → 电流 PI (setpoint = ibase + share_error, feedback = iphase) → alpha = 1 + u
//   4. 模式切换单周期同步 (ModeSync): 变化周期三相统一用平均 alpha (防环流)
//   5. 写 PwmBuckBoost: set_mode / set_ratio / 逐相 set_alpha
//
// 相位参数化 (N=1..3):
//   N=1 退化: iavg=iphase → share_error=0, ibase=paside, ModeSync(1)=恒等; 迟滞仍选模式
//   三相 (N=3): 逐相独立 alpha 修正 + 切换同步
//
// 数据流 (FAST 单写者, supercap 注入, 本模块只读 + 写 PWM):
//   mod_share_set_voltages(va, vb) → set_paside(paside) → set_currents(i[3]) → tick()
//
// 与 mod_supercap 分工: supercap 做功率环 + 保护, 本模块做电流均衡 + 模式选择.
// supercap 每个 FAST tick 先注入电压/电流/相电流指令, 再调本模块 tick 写 PWM.
// 急停时 supercap 调 mod_share_emergency → 本模块置 fault, 后续 tick 不再写 PWM.

#ifndef MOD_CURRENT_SHARE_H
#define MOD_CURRENT_SHARE_H

#include <stdint.h>
#include <stdbool.h>
#include "pid_reg4.h"      // 相电流 PI (PidReg4 = TI pi_reg4)
#include "comp_protection.h"  // Hysteresis / ModeSync
#include "comp_math.h"        // math_clamp_f
#include "pwm_buckboost.h"

#define MOD_SHARE_MAX_PHASES 3  // 最大并联相数 (1..3)

// 电流 PI 输出限幅 (alpha 修正量 u ∈ ±5%, alpha ∈ [0.95, 1.05])
#define MOD_SHARE_ALPHA_DELTA_MAX 0.05f

// ======== 配置 POD — YAML 注入目标 (与 supercap_3ph.yaml share: 段一致) ========
typedef struct {
  uint8_t num_phases;  // 并联相数 (1..3)
  float share_gain;    // 均流比例增益 (修正: iavg−iphase → ±share_lim)
  float share_lim;     // 均流修正限幅 (默认 0.04 = ±4%)
  // 模式迟滞窗口 (占空比域, WEILAI 超电: 进 0.97/1.03, 出 0.90/1.10)
  float hyst_enter_lo, hyst_enter_hi;
  float hyst_exit_lo, hyst_exit_hi;
  // 每相电流 PI 增益 (各相共用; 均流增益 share_gain 由 supercap slot[9] 注入覆盖)
  float pid_kp;  // 电流 PI 比例增益
  float pid_ki;  // 电流 PI 积分增益
} ModShareCfg;

// ======== 运行时相位状态 (public, host 测试可读) ========
typedef struct {
  PidReg4 pi;     // 相电流 PI (输出 u, alpha = 1 + u)
  float share_error;  // 均流修正 (clamp 后)
  float alpha;        // 调制指数 (1 + PI 输出)
} ModSharePhase;

typedef struct {
  ModShareCfg cfg;

  PwmBuckBoost *pwm;  // 输出设备 (mod_share_bind 注入)

  Hysteresis mode_hyst;  // 模式迟滞 (ratio 域)
  ModeSync sync;         // 模式切换单周期同步
  uint8_t cur_mode;      // 当前模式 (PwmMode 值)
  float ratio;           // 电压比 vb/va (钳位后)

  float va, vb;                         // supercap 注入 (FAST 单写者)
  float iavg;                           // 相电流均值
  float paside;                         // 功率环输出的相电流指令 (supercap 注入)
  float ibase;                          // 每相基准 = paside / num_phases
  float i_phase[MOD_SHARE_MAX_PHASES];  // 各相电流 (supercap 注入)
  ModSharePhase ph[MOD_SHARE_MAX_PHASES];

  uint8_t last_written_mode;  // 上次写入 PWM 的模式 (仅变化时 set_mode)
  float last_written_ratio;   // 上次写入 PWM 的电压比
  bool fault;                 // 急停标志: 置位后 tick 不写 PWM (supercap 恢复时清)
} ModCurrentShare;

// 构造: 存 cfg + 初始化迟滞/同步/每相 PI (模式窗口默认 0.97/1.03/0.90/1.10)
void mod_share_init(ModCurrentShare *me, const ModShareCfg *cfg);

// 绑定输出设备 (board_init): 注入 PwmBuckBoost 实例
void mod_share_bind(ModCurrentShare *me, PwmBuckBoost *pwm);

// FAST 单写者注入 (supercap 每 tick 调): 电压 / 相电流指令 / 相电流
void mod_share_set_voltages(ModCurrentShare *me, float va, float vb);
void mod_share_set_paside(ModCurrentShare *me, float paside);
void mod_share_set_currents(ModCurrentShare *me, const float *i);

// FAST tick: 模式迟滞 + 均流 + 切换同步 → 写 PWM (fault 置位则 no-op)
void mod_share_tick(ModCurrentShare *me);

// 急停: 置 fault + 清每相 PI 积分 + alpha=0 (supercap 故障路径调, 之后 tick 停写 PWM)
void mod_share_emergency(ModCurrentShare *me);

// 恢复: 清 fault + 重置 PI + alpha=1 (supercap start/recover 调)
void mod_share_release(ModCurrentShare *me);

#endif  // MOD_CURRENT_SHARE_H
