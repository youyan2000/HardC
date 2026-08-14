#ifndef MOD_SUPERCAP_H
#define MOD_SUPERCAP_H

// 超级电容功率控制模块 — PowerStage 派生, 三相并联 (Module 层, ctx fast)
//
// 职责 (每 FAST tick, 28.3kHz):
//   功率环级联 (WEILAI 超电 PowerCtrl_Control):
//     p_referee = LPF(power_lpf, va × i_chassis)      # 实测母线功率 (正=负载消耗)
//     p_setpoint = PI(pid_p, sp=referee_power, fbk=p_referee)   # 期望功率
//     p_setpoint 限幅 [p_lim_lo, p_lim_hi] (PI 输出限幅实现 → 抗积分饱和)
//     taper 近满压 → i_side = p_setpoint / va
//   保护:
//     charge_ok Hysteresis (28.6/28.0): 满电停充 (p_lim_hi=0)
//     discharge_ok Hysteresis (18/19): 低压切除 (p_lim_lo=0)
//     short_deb / unbalance_deb Debounce → FAULT (0x08/0x40) → 急停
//   均流注入 (mod_current_share):
//     mod_share_set_voltages/paside/currents → mod_share_tick (写 PwmBuckBoost)
//
// 采样契约: 本模块每 tick 读 adc_dc_sampler_get_value(ch) — App FAST ISR 须在每
//   控制周期先调 adc_dc_sampler_fetch() 刷新 value[] 快照 (见 app_main.h.tmpl 采样路径).
// 绑定顺序: board_init 必须先 mod_share_bind(share, pwm) 再 mod_supercap_bind(...),
//   否则 base.pwm 快照为 NULL, 故障急停时 pwm_emergency_stop 不会封波 (致命).
//
// 功率符号约定 (本模块内部, 对照 WEILAI PowerCtrl):
//   referee_power > 0 = 期望充电 (吸收母线功率), < 0 = 期望放电 (向母线输出)
//   p_lim_hi > 0 = 充电功率上限, p_lim_lo < 0 = 放电功率下限
//   i_side > 0 = 充电流, < 0 = 放电流 (注入 share 后 per-phase 基准 ibase = i_side/N)
//   母线功率 p_referee = va × i_chassis, 正 = 负载从母线取电
//
// PowerStage 基类字段重解释 (超电不用 vref 级联, 头注释声明):
//   base.vref  = 充电目标电压 (charge_stop_v)
//   base.iref  = 裁判电流参考 = referee_power/va (派生值, 每 tick 更新, 非控制输入)
//   base.ovp/ocp = 未使用 (本模块故障为短路/失平衡去抖)
//   级联绕过 base.loop[2] (PowerStage 仅 2 槽): per-phase 电流 PI 归 mod_current_share
//
// 事件/命令 (五原语, 见 agent.md §1.2):
//   evt (SPSC 环, 单生产者 = FAST 故障路径): 短路 0x08 / 失平衡 0x40 (值 = comp_error.h 位掩码)
//   cmd (Command 邮箱, MAIN→FAST): referee_power 周期边界生效 (CAN 0x061 喂)

#include <stdint.h>
#include <stdbool.h>
#include "comp_power_stage.h"
#include "comp_pi_reg4.h"
#include "comp_filter.h"      // LowPassFilter
#include "comp_protection.h"  // Hysteresis / Debounce
#include "comp_math.h"        // math_clamp_f / math_abs_f
#include "comp_error.h"       // ERROR_SHORT_CIRCUIT (0x08) / ERROR_PHASE_UNBALANCE (0x40)
#include "comp_ring.h"        // SPSC 事件流 (FAST→MAIN)
#include "comp_latch.h"       // Latest 锁存 (FAST→SLOW)
#include "comp_mailbox.h"     // Command 邮箱 (MAIN→FAST)
#include "adc_dc_sampler.h"
#include "mod_current_share.h"

#define MOD_SC_MAX_PHASES MOD_SHARE_MAX_PHASES  // 复用均流模块相数上限 (3)

// ======== 配置 POD — YAML 注入目标 (与 supercap_3ph.yaml params 槽位一致) ========
typedef struct {
  // --- 0xFB 槽位 0-9 (supercap_3ph.yaml params.slot) ---
  float pid_p_kp;         // [0] 功率环比例增益
  float pid_p_ki;         // [1] 功率环积分增益 (Ki, output/(error·s))
  float charge_stop_v;    // [2] 满电停充电压 (V, 28.6)
  float charge_resume_v;  // [3] 恢复充电电压 (V, 28.0)
  float vcut_lo;          // [4] 低压切除电压 (V, 18)
  float vcut_hi;          // [5] 低压恢复电压 (V, 19)
  float i_lim_a;          // [6] 交流侧电流限幅 (A)
  float cap_in_ilimit;    // [7] 超级电容充电电流限幅 (A)
  float cap_out_ilimit;   // [8] 超级电容放电电流限幅 (A)
  float share_gain;       // [9] 均流增益 (注入 mod_current_share)

  // --- 非槽位 (supercap_3ph.yaml adc/保护段) ---
  float power_lpf_fc;                   // 裁判功率低通截止频率 (Hz, 120)
  float short_ilim;                     // 短路去抖阈值 (A, 默认 2×i_lim_a)
  float unbalance_thr;                  // 三相失平衡阈值 (A, 默认 0.5×i_lim_a)
  uint8_t num_phases;                   // 并联相数 (1..3)
  uint8_t adc_ch_va;                    // 母线电压 ADC 通道
  uint8_t adc_ch_vb;                    // 电容电压 ADC 通道
  uint8_t adc_ch_i[MOD_SC_MAX_PHASES];  // 相电流 ADC 通道
  uint8_t adc_ch_icap;                  // 电容电流 ADC 通道
  uint8_t adc_ch_ichassis;              // 底盘电流 ADC 通道
} ModSuperCapCfg;

// ======== 跨上下文事件/命令 (五原语) ========
// 事件字节直接采用 comp_error.h 位掩码 — MAIN 侧可 OR 进 ERROR_* 错误寄存器
typedef enum {
  MOD_SC_EVT_FAULT_SHORT = ERROR_SHORT_CIRCUIT,        // 0x08 短路
  MOD_SC_EVT_FAULT_UNBALANCE = ERROR_PHASE_UNBALANCE,  // 0x40 三相失平衡
} ModSuperCapEvt;

#define MOD_SC_CMD_SET_REFEREE_POWER 1u  // 裁判功率目标 (W, 正=充电/负=放电, CAN 0x061 喂)

// ======== 超级电容运行时实例 ========
typedef struct {
  PowerStage base;  // [首成员!] 父类 — container_of 下溯入口

  // --- 设备绑定 (mod_supercap_bind 注入) ---
  AdcDcSampler *adc;       // ADC 采样
  ModCurrentShare *share;  // 均流模块 (共享 PwmBuckBoost, supercap 驱动其 tick)

  // --- 运行配置 (可热替换, 0xFB 调参 / apply_config 目标) ---
  ModSuperCapCfg cfg;

  // --- 级联元件 ---
  LowPassFilter power_lpf;  // 裁判功率低通 (120Hz)
  PiReg4Cfg pid_p_cfg;      // 功率环 PI 配置 (sync_cfg 派生, 输出限幅 = 功率边界)
  PiReg4State pid_p;        // 功率环 PI 状态

  // --- 保护 ---
  Hysteresis charge_ok;     // 满电停充迟滞: state=true=允许充电 (进 28.0, 出 28.6)
  Hysteresis discharge_ok;  // 低压切除迟滞: state=true=允许放电 (进 19, 出 18)
  Debounce short_deb;       // 短路 (FAULT 0x08)
  Debounce unbalance_deb;   // 失平衡 (FAULT 0x40)

  // --- 运行态 (public, host 测试可读) ---
  float va, vb;                      // 母线/电容电压 (V)
  float i_phase[MOD_SC_MAX_PHASES];  // 相电流 (A)
  float icap, ichassis;              // 电容/底盘电流 (A)
  float p_referee;                   // LPF 后裁判功率 (W)
  float p_setpoint;                  // 功率 PI 输出 (W, 限幅后)
  float p_lim_lo, p_lim_hi;          // 功率限幅 (W)
  float i_side;                      // 交流侧电流指令 (A)
  float referee_power;               // 功率 PI 设定值 (W, 正=充电/负=放电)
  float cap_health;                  // 电容健康度 (vb/charge_stop_v, 遥测)

  // --- 跨上下文交接 (五原语) ---
  Latch telemetry;      // Latest 锁存 (FAST→SLOW): 每周期写 i_side
  Mailbox cmd;          // Command 邮箱 (MAIN→FAST): referee_power
  Ring evt;             // SPSC 环 (FAST→MAIN): 保护事件, 单生产者 = FAST 故障路径
  uint8_t evt_buf[16];  // evt 环缓冲 (容量 15 事件, 满则丢不阻塞 FAST)
} ModSuperCap;

// ======== API ========

// 构造: 绑定 ops + 存 cfg + 初始化级联/保护/五原语 (设备绑定由 mod_supercap_bind 完成)
void mod_supercap_init(ModSuperCap *me, const ModSuperCapCfg *cfg);

// 绑定设备: adc + share (board_init). base.pwm 从 share->pwm 快照 (功率环急停共用)
// 前提: 必须先 mod_share_bind(share, pwm) 再调本函数, 否则急停封波失效 (见头注释)
void mod_supercap_bind(ModSuperCap *me, AdcDcSampler *adc, ModCurrentShare *share);

// 配置同步: cfg → PI/保护迟滞/均流增益 派生 (apply_config / 0xFB 后调用)
void mod_supercap_sync_cfg(ModSuperCap *me);

// 控制周期: 状态机 + 级联功率环 + 保护 + 驱动均流/PWM
void mod_supercap_tick(ModSuperCap *me);

// 启动 / 正常停机 / 紧急封波
void mod_supercap_start(ModSuperCap *me);
void mod_supercap_stop(ModSuperCap *me);
void mod_supercap_emergency(ModSuperCap *me);

// MAIN 上下文: 热调裁判功率目标 (W, 正=充电/负=放电) — 命令邮箱周期边界生效
static inline void mod_supercap_set_referee_power(ModSuperCap *me, float w) {
  mailbox_post(&me->cmd, MOD_SC_CMD_SET_REFEREE_POWER, w);
}

// MAIN 上下文: 消费保护事件流 (SPSC 环排空); 返回 false = 无事件
static inline bool mod_supercap_evt_pop(ModSuperCap *me, uint8_t *ev) {
  return ring_pop(&me->evt, ev);
}

// 0xFB 调参: coef[10] 槽位 → cfg (与 supercap_3ph.yaml params.slot 一致)
void mod_supercap_apply_tune(ModSuperCap *me, const float coef[10]);

// 查询状态
PstSt mod_supercap_state(const ModSuperCap *me);

#endif  // MOD_SUPERCAP_H
