// Buck 降压控制模块 — PowerStage 具体实现 (Module 层)
//
// 集成模式示例: "采样 + 环路控制 + PWM 输出"
//   采样: AdcDcSampler 读 vout/iout/vin (ADC 通道工程量, 校准 k/b 在采样器内)
//   环路: 级联 电压环(PidReg4) → 电流环(PidReg4) + 前馈 D = ff_weight·Vref/Vin
//   输出: PwmBuckBoost 写占空比 (ch_drive, [duty_min, duty_max])
//   保护: vout>ovp 或 iout>ocp 去抖确认 → power_stage_emergency
//
// 配置来源: Config/topologies/buck.yaml
//   params 槽位 0-9 经 0xFB 帧 apply_tune 热调参 (槽位映射与 buck.yaml 完全一致)
//   pwm/adc 段 (ch_drive/duty 限幅/ADC 通道) 注入 ModBuckCfg 非槽位字段;
//   ADC 增益不是模块职责 — 在 board_init 校准 AdcDcSampler 的 k[] 数组

#ifndef MOD_BUCK_H
#define MOD_BUCK_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>  // NULL (mod_buck_bind)
#include "comp_power_stage.h"
#include "pid_reg4.h"        // buck 功率环 = TI pi_reg4 语义 (aw=CLAMP 位级)
#include "comp_ring.h"     // SPSC 事件流 (FAST→MAIN)
#include "comp_latch.h"    // Latest 锁存 (FAST→SLOW)
#include "comp_mailbox.h"  // Command 邮箱 (MAIN→FAST)
#include "pwm_buckboost.h"
#include "adc_dc_sampler.h"

// ======== 配置 POD — YAML 注入目标 (与 buck.yaml params/pwm/adc 字段一一对应) ========
typedef struct {
  // --- 0xFB 槽位 0-9 (Config/topologies/buck.yaml params.slot) ---
  float vref;  // [0] 输出电压参考 (V)
  float iref;  // [1] 输出电流参考 (A) — 兼作电压环输出限幅 (限流)
  struct {
    float kp;  // [2] 电压环比例增益
    float ki;  // [3] 电压环积分增益 (Ki, output/(error·s))
  } pid_v;
  struct {
    float kp;  // [4] 电流环比例增益
    float ki;  // [5] 电流环积分增益
  } pid_i;
  float ff_weight;        // [6] 前馈混合系数 (0~1)
  float ovp;              // [7] 过压保护阈值 (V)
  float ocp;              // [8] 过流保护阈值 (A)
  float soft_start_step;  // [9] 软启动步长 (V/tick)

  // --- 非槽位: 通道与限幅 (buck.yaml pwm/adc 段) ---
  uint8_t ch_drive;     // PWM 驱动通道号
  float duty_min;       // 占空比下限
  float duty_max;       // 占空比上限
  uint8_t adc_ch_vout;  // 输出电压 ADC 通道
  uint8_t adc_ch_iout;  // 输出电流 ADC 通道
  uint8_t adc_ch_vin;   // 输入电压 ADC 通道
} ModBuckCfg;

// ======== 跨上下文事件/命令 (五原语, 见 agent.md §1.2) ========
// 事件流单生产者 = FAST 故障路径 (SPSC 契约); START/STOP 状态变化由调用方直接读
// mod_buck_state 观察, 不进事件环 (避免 MAIN 调用 start/stop 引入第二生产者竞态)
typedef enum {
  MOD_BUCK_EVT_FAULT_OVP = 1,  // 过压保护触发 (去抖确认, FAST 推)
  MOD_BUCK_EVT_FAULT_OCP = 2,  // 过流保护触发 (去抖确认, FAST 推)
} ModBuckEvt;

#define MOD_BUCK_CMD_SET_VREF 1u  // 命令邮箱: 热调输出电压参考 (周期边界生效)

// ======== Buck 运行时实例 ========
typedef struct {
  PowerStage base;  // [首成员!] 父类 — container_of 下溯入口

  // --- 设备绑定 (mod_buck_bind 注入) ---
  PwmBuckBoost *pwm_buck;  // PWM 设备具体实例
  AdcDcSampler *adc;       // ADC 设备具体实例

  // --- 环路 (值包含, PidReg4 内嵌配置; 经 .base 挂到 PowerStage.loop[]) ---
  PidReg4 pid_v;  // 电压环 (外环): 输出 = 电流指令 — TI pi_reg4 语义
  PidReg4 pid_i;  // 电流环 (内环): 输出 = 占空比 — TI pi_reg4 语义

  // --- 运行配置 (可热替换, 0xFB 调参目标) ---
  ModBuckCfg cfg;

  // --- 运行时状态 ---
  float vout;            // 输出电压采样 (V)
  float iout;            // 输出电流采样 (A)
  float vin;             // 输入电压采样 (V)
  float i_ref;           // 电流环指令 (电压环输出)
  float duty_pi;         // 电流环 PI 输出 (占空比分量)
  float duty;            // 最终占空比 (PI + 前馈, 已限幅)
  float soft_start_ref;  // 软启动当前参考电压 (V)

  // --- 跨上下文交接 (五原语, 见 agent.md §1.2) ---
  Latch telemetry;      // Latest 锁存 (FAST→SLOW): 每周期写 vout, SLOW 读最新值
  Mailbox cmd;          // Command 邮箱 (MAIN→FAST): set_vref 周期边界生效
  Ring evt;             // SPSC 环 (FAST→MAIN): 保护事件流, 单生产者 = FAST 故障路径, MAIN 排空
  uint8_t evt_buf[16];  // evt 环缓冲 (容量 15 事件, 满则丢不阻塞 FAST)
} ModBuck;

// ======== API ========

// 构造: 绑定 ops + 存 cfg + 初始化双环 PidReg4 (设备绑定由 mod_buck_bind 完成)
void mod_buck_init(ModBuck *me, const ModBuckCfg *cfg);

// 绑定设备: 同时填充 PowerStage 基类 pwm/adc 指针 (board_init 中调用)
static inline void mod_buck_bind(ModBuck *me, PwmBuckBoost *pwm_buck, AdcDcSampler *adc) {
  me->pwm_buck = pwm_buck;
  me->adc = adc;
  me->base.pwm = pwm_buck ? &pwm_buck->base : NULL;
  me->base.adc = adc ? &adc->base : NULL;
}

// 配置同步: 将 me->cfg 的 PI 参数/限幅派生到运行时 (apply_config / 0xFB 后调用)
void mod_buck_sync_cfg(ModBuck *me);

// 控制周期: 状态机 + 采样/保护/软启/双环/前馈/PWM
void mod_buck_tick(ModBuck *me);

// 启动 / 正常停机 / 紧急封波
void mod_buck_start(ModBuck *me);
void mod_buck_stop(ModBuck *me);
void mod_buck_emergency(ModBuck *me);

// 热切换参考: vref / iref (同步批量改, MAIN/启动期用; 运行期热调单值用 mod_buck_set_vref)
void mod_buck_set_ref(ModBuck *me, float vref, float iref);

// MAIN 上下文: 热调输出电压参考 — 经命令邮箱周期边界生效 (无锁安全, 最新命令覆盖)
static inline void mod_buck_set_vref(ModBuck *me, float vref) {
  mailbox_post(&me->cmd, MOD_BUCK_CMD_SET_VREF, vref);
}

// MAIN 上下文: 消费保护事件流 (SPSC 环排空); 返回 false = 无事件
static inline bool mod_buck_evt_pop(ModBuck *me, uint8_t *ev) {
  return ring_pop(&me->evt, ev);
}

// 0xFB 调参: coef[10] 槽位 → cfg (与 buck.yaml params.slot 一致)
void mod_buck_apply_tune(ModBuck *me, const float coef[10]);

// 查询状态
PstSt mod_buck_state(const ModBuck *me);

#endif  // MOD_BUCK_H
