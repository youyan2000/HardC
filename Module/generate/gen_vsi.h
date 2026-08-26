// 三相电压源逆变器 (VSI) 控制模块 — PowerStage 派生, 六开关三相逆变 (Module 层, ctx fast)
//
// 职责 (每 FAST tick; 采样+保护所有状态每周期执行, 与 buck A6 对齐):
//   1. 采样: AdcAcSampler fast_fetch (Ia/Ic + Vab/Vbc + Vdc) → Clarke → Park(dq)
//   2. PLL:  SRF-PLL (PllSrf) 锁相 Vab (θ → Park/反Park 旋转角)
//   3. 电流环: d/q 轴双环 PI (PidReg4, 输出限幅 ±v_dq_max) → 反 Park → v_αβ
//   4. SVPWM: svpwm_set_vector(v_αβ, v_dc) → 三相占空比 → 6 开关 (写 PwmSvpwm)
//   5. 保护: 母线过压 (ovp) / 相电流过流 (ocp RMS) 去抖 → 急停 (SPSC 事件流)
// 可选电压环 (cfg.vdc_ctrl_enable): vdc_ref → PI → id_ref (外环), 电流环 d 轴作为内环
//
// 输入电压标幺约定 (对齐 pwm_svpwm.h): svpwm_set_vector 的 v_alpha/v_beta 为标幺值
//   (相对 0.5*Vdc, 线性调制区 |V| ≤ 0.577), 故反 Park 输出的 v_dq (V) 须除以 Vdc 归一:
//     v_alpha_pu = v_alpha / v_dc_bus (本模块内部做 V→pu 换算)
//
// 采样契约: 本模块每 tick 读 adc_ac_sampler 的 va/vb/vc + ia1/ib1/ic1 + vdc
//   — App FAST ISR 须在每控制周期先调 adc_ac_sampler_fast_fetch() (见 app_main.h.tmpl).
//
// PowerStage 基类字段重解释:
//   base.vref = 母线电压参考 vdc_ref (V, 电压环使能时)
//   base.iref = q 轴电流参考 iq_ref (A, 默认 0 = 纯无功能)
//   base.ovp/ocp = 母线过压/相电流过流阈值 (V/A)
//   base.loop[0] = 电压环 (可空), base.loop[1] = 电流环 (PidBase* 取 d 轴)

#ifndef GEN_VSI_H
#define GEN_VSI_H

#include <stdint.h>
#include <stdbool.h>
#include "comp_power_stage.h"  // PowerStage 基类
#include "pid_reg4.h"          // dq 电流环 / 母线电压环 PI
#include "pll_srf.h"           // SRF-PLL 锁相 (PllSrf, base 首成员)
#include "comp_transform.h"    // Clarke / Park / 反Park
#include "comp_protection.h"   // Hysteresis / Debounce
#include "comp_math.h"         // math_clamp_f / math_abs_f
#include "comp_error.h"        // ERROR_OVER_VOLTAGE (0x02) / ERROR_OVER_CURRENT (0x10)
#include "comp_ring.h"         // SPSC 事件流 (FAST→MAIN)
#include "comp_latch.h"        // Latest 锁存 (FAST→SLOW)
#include "comp_mailbox.h"      // Command 邮箱 (MAIN→FAST)
#include "pwm_svpwm.h"         // SVPWM 设备 (发波)
#include "adc_ac_sampler.h"    // 三相交流采样

// ======== 配置 POD — YAML 注入目标 (与 vsi_3ph.yaml params 槽位一致) ========
typedef struct {
  // --- 0xFB 槽位 0-9 (vsi_3ph.yaml params.slot) ---
  float vdc_ref;  // [0] 母线电压参考 (V, 电压环使能时)
  float id_ref;   // [1] d 轴电流参考 (A, 无电环等于 0)
  float iq_ref;   // [2] q 轴电流参考 (A)
  struct {
    float kp;  // [3] d 轴电流环比例增益
    float ki;  // [4] d 轴电流环积分增益
  } pid_id;
  struct {
    float kp;  // [5] q 轴电流环比例增益
    float ki;  // [6] q 轴电流环积分增益
  } pid_iq;
  float pll_bw;  // [7] PLL 带宽 (Hz, 派生 kp/ki)
  float ovp;     // [8] 母线过压阈值 (V)
  float ocp;     // [9] 相电流过流阈值 (A)

  // --- 非槽位 (vsi_3ph.yaml pwm/adc/保护段) ---
  float control_freq_hz;   // 控制环频率 (Hz, 缺省 20000; 决定 PID dt)
  float vdc_ctrl_enable;   // 1=母线电压环使能 (vdc_ref→id_ref), 0=直接 id_ref
  float v_dq_max;          // dq 电压参考限幅 (V, 缺省 0.577×vdc 线性区)
  float i_ocp_debounce_n;  // OCP 去抖确认次数 (缺省 8)
  float v_ovp_debounce_n;  // OVP 去抖确认次数 (缺省 8)
} GenVsiCfg;

// ======== 跨上下文事件/命令 (五原语) ========
typedef enum {
  GEN_VSI_EVT_FAULT_OVP = ERROR_OVER_VOLTAGE,  // 0x02 母线过压
  GEN_VSI_EVT_FAULT_OCP = ERROR_OVER_CURRENT,  // 0x10 相电流过流
} GenVsiEvt;

#define GEN_VSI_CMD_SET_REF 1u  // 热调参考: arg=id_ref (MAIN→FAST, 周期边界生效)

// ======== 三相 VSI 运行时实例 ========
typedef struct {
  PowerStage base;  // [首成员!] 父类 — container_of 下溯入口

  // --- 设备绑定 (gen_vsi_bind 注入) ---
  PwmSvpwm *pwm;      // SVPWM 设备 (六开关发波)
  AdcAcSampler *adc;  // 三相交流采样
  PllSrf pll;         // SRF-PLL (值包含, base 首成员 = PllBase* 可注入)

  // --- 运行配置 (可热替换, 0xFB 调参 / apply_config 目标) ---
  GenVsiCfg cfg;

  // --- 控制元件 ---
  PidReg4 pid_id;   // d 轴电流环 (内环之一)
  PidReg4 pid_iq;   // q 轴电流环
  PidReg4 pid_vdc;  // 母线电压环 (可选外环, vdc_ctrl_enable 时启用)

  // --- 保护 ---
  Debounce ovp_deb;  // 母线过压去抖 (FAULT 0x02)
  Debounce ocp_deb;  // 相电流过流去抖 (FAULT 0x10)

  // --- 运行态 (public, host 测试可读) ---
  float theta;                   // PLL 锁相角 (rad)
  float va, vb, vc;              // 逆变侧相电压 (V)
  float ia, ib, ic;              // 逆变侧相电流 (A)
  float vdc;                     // 母线电压 (V)
  float id, iq;                  // dq 轴电流反馈 (A)
  float vd_ref, vq_ref;          // dq 轴电压参考 (V, 电流环输出)
  float v_alpha, v_beta;         // 反 Park 输出 αβ 电压 (pu, 送 SVPWM)
  float duty_a, duty_b, duty_c;  // 三相上管占空比 (SVPWM 输出镜像)
  float cur_mod_index;           // 当前调制比 (svpwm_get_modulation_index)
  bool fault_active;             // 曾发生故障锁存 (start/emergency 复位, 诊断用)

  // --- 跨上下文交接 (五原语) ---
  Latch telemetry;            // Latest 锁存 (FAST→SLOW): 每周期写 vdc
  Mailbox cmd;                // Command 邮箱 (MAIN→FAST): SET_REF
  Ring evt;                   // SPSC 环 (FAST→MAIN): 保护事件, 单生产者 = FAST 故障路径
  uint8_t evt_buf[16];        // evt 环缓冲 (容量 15 事件)
  uint32_t evt_overflow_cnt;  // 环满丢弃计数 (A5)
} GenVsi;

// ======== API ========

// 构造: 绑定 ops + 存 cfg + 初始化双环/PLL/保护/五原语
//   grid_freq_hz: 电网标称频率 (Hz, 如 50/60) — PLL 基准
//   (设备绑定由 gen_vsi_bind 完成)
void gen_vsi_init(GenVsi *me, const GenVsiCfg *cfg, float grid_freq_hz);

// 绑定设备: svpwm + adc (board_init). base.pwm/acd 同步 (PowerStage 基类)
void gen_vsi_bind(GenVsi *me, PwmSvpwm *pwm, AdcAcSampler *adc);

// 配置同步: cfg → 双环 PI / PLL 带宽 / 保护去抖次数 派生 (apply_config / 0xFB 后)
void gen_vsi_sync_cfg(GenVsi *me);

// 控制周期 (FAST): 采样读 → PLL → Park → 双环 → 反 Park → SVPWM → 保护
void gen_vsi_tick(GenVsi *me);

// 启动 / 正常停机 / 紧急封波
void gen_vsi_start(GenVsi *me);
void gen_vsi_stop(GenVsi *me);
void gen_vsi_emergency(GenVsi *me);

// MAIN 上下文: 热调 d 轴电流参考 (命令邮箱, 周期边界生效)
static inline void gen_vsi_set_id_ref(GenVsi *me, float id_ref) {
  mailbox_post(&me->cmd, GEN_VSI_CMD_SET_REF, id_ref);
}

// MAIN 上下文: 消费保护事件流 (SPSC 环排空); 返回 false = 无事件
static inline bool gen_vsi_evt_pop(GenVsi *me, uint8_t *ev) {
  return ring_pop(&me->evt, ev);
}

// 诊断: evt 环满丢弃计数
static inline uint32_t gen_vsi_evt_overflow(const GenVsi *me) {
  return me->evt_overflow_cnt;
}

// 0xFB 调参: coef[10] 槽位 → cfg (与 vsi_3ph.yaml params.slot 一致)
void gen_vsi_apply_tune(GenVsi *me, const float coef[10]);

// 查询状态
PstSt gen_vsi_state(const GenVsi *me);

#endif  // GEN_VSI_H
