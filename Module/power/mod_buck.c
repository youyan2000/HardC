// Buck 降压控制模块 — PowerStage 具体实现
//
// 每 tick 流程 (PST_RUN):
//   采样(vout/iout/vin = AdcDcSampler 工程量, 校准在采样器内) → 保护(vout>ovp / iout>ocp 去抖 → 急停)
//   → 软启动(vref 从 0 每 tick +soft_start_step) → 电压环(err=vref-vout → 电流指令)
//   → 电流环(err=电流指令-iout → duty_pi) → 前馈(duty = clamp(duty_pi + ff·vref/vin))
//   → pwm_set_duty(ch_drive, duty)
//
// 状态机: INIT → IDLE → RUN ⇄ FAULT (start 恢复重新软启)
//   FAULT_HOLD / RECOVER 在枚举中保留, 供更复杂拓扑 (如多级保护确认) 扩展

#include "mod_buck.h"
#include "container_of.h"
#include "comp_math.h"   // math_clamp_f
#include <string.h>

// 控制周期 (秒) — 100us = 10kHz, 须与 App_OnControlTick 调用频率一致
#define MOD_BUCK_DT  0.0001f

// 故障去抖确认次数 (OVP/OCP 连续 N tick 触发才急停, 防噪声误报)
#define MOD_BUCK_FAULT_DEBOUNCE_N  8u

// ======== 内部辅助 ========

// 采样: AdcDcSampler 工程量 (k/b 校准已在采样器内部完成)
static inline void buck_sample(ModBuck *me) {
  me->vout = adc_dc_sampler_get_value(me->adc, me->cfg.adc_ch_vout);
  me->iout = adc_dc_sampler_get_value(me->adc, me->cfg.adc_ch_iout);
  me->vin  = adc_dc_sampler_get_value(me->adc, me->cfg.adc_ch_vin);
  if (me->vin <= 0.0f) {
    me->vin = 1.0f;   // 防前馈除零
  }
}

// 软启动: 当前参考每 tick 上升 soft_start_step, 限到目标; 参考下调时立即跟随
static inline float buck_soft_start(ModBuck *me, float target) {
  if (me->soft_start_ref < target) {
    me->soft_start_ref += me->cfg.soft_start_step;
    if (me->soft_start_ref > target) {
      me->soft_start_ref = target;
    }
  } else if (me->soft_start_ref > target) {
    me->soft_start_ref = target;   // set_ref 下调 vref 时防卡在高值
  }
  return me->soft_start_ref;
}

// 运行态主控制循环: 采样 → 保护 → 软启 → 双环 → 前馈 → PWM
static void buck_run(ModBuck *me) {
  // 设备未绑定则拒绝运行
  if (!me->pwm_buck || !me->adc) {
    me->base.st = PST_FAULT;
    return;
  }

  // 1. 采样
  buck_sample(me);

  // 2. 保护: OVP/OCP 去抖确认 → 急停 (置 FAULT + pwm_emergency_stop)
  //    阈值读 PowerStage 基类 (base.ovp/ocp, 由 sync_cfg 从 cfg 同步)
  bool fault = (me->vout > me->base.ovp) || (me->iout > me->base.ocp);
  if (fault) {
    if (me->base.debounce_cnt < MOD_BUCK_FAULT_DEBOUNCE_N) {
      me->base.debounce_cnt++;
    }
    if (me->base.debounce_cnt >= MOD_BUCK_FAULT_DEBOUNCE_N) {
      power_stage_emergency(&me->base);
    }
    return;   // 故障期间不刷新 PWM
  }
  me->base.debounce_cnt = 0;

  // 3. 软启动参考 (从 0 斜坡到基类 vref, 由 sync_cfg 从 cfg.vref 同步)
  float vref = buck_soft_start(me, me->base.vref);

  // 4. 电压环: err = vref - vout → 电流指令 (输出限幅 [0, iref])
  me->i_ref = pi_reg4_run(&me->pid_v.st, &me->pid_v.cfg, vref, me->vout);

  // 5. 电流环: err = 电流指令 - iout → 占空比分量 (输出限幅 [0, 1])
  me->duty_pi = pi_reg4_run(&me->pid_i.st, &me->pid_i.cfg, me->i_ref, me->iout);

  // 6. 前馈混合: D = duty_pi + ff_weight·(vref/vin), 钳到 [duty_min, duty_max]
  float ff = me->cfg.ff_weight * (vref / me->vin);
  me->duty = math_clamp_f(me->duty_pi + ff, me->cfg.duty_min, me->cfg.duty_max);

  // 7. PWM 输出 (pwm_set_duty 内部二次限幅到 base.duty_min/max)
  pwm_set_duty(&me->pwm_buck->base, me->cfg.ch_drive, me->duty);
}

// ======== ops 实现 (PowerStage* 第一参数, container_of 下溯) ========

static void buck_init(PowerStage *base) {
  ModBuck *me = container_of(base, ModBuck, base);
  pi_reg4_init(&me->pid_v.st);
  pi_reg4_init(&me->pid_i.st);
  me->soft_start_ref   = 0.0f;
  me->vout = me->iout = me->vin = 0.0f;
  me->i_ref = me->duty_pi = me->duty = 0.0f;
  me->base.debounce_cnt = 0;
  me->base.st = PST_INIT;
}

static void buck_tick(PowerStage *base) {
  ModBuck *me = container_of(base, ModBuck, base);

  switch (me->base.st) {
  case PST_INIT:
    // 自检通过 → 空闲 (真实工程在此检查 PWM/ADC 就绪)
    me->base.st = PST_IDLE;
    break;

  case PST_IDLE:
    break;   // 等待 start

  case PST_RUN:
    buck_run(me);
    break;

  case PST_FAULT_HOLD:
  case PST_FAULT:
    break;   // 保持封波, 等待 start 恢复

  case PST_RECOVER:
    // 恢复: 软启动从 0 重新开始, 进入 RUN (软启在 RUN 内完成)
    me->soft_start_ref = 0.0f;
    me->base.st = PST_RUN;
    break;

  default:
    break;
  }
}

static void buck_start(PowerStage *base) {
  ModBuck *me = container_of(base, ModBuck, base);
  if (me->base.st == PST_IDLE || me->base.st == PST_FAULT ||
      me->base.st == PST_RECOVER) {
    me->soft_start_ref    = 0.0f;   // 重新软启动
    me->base.debounce_cnt = 0;
    // 清双环积分器 — FAULT 急停后残留的饱和积分会顶满电流指令, 破坏软启动
    pi_reg4_reset(&me->pid_v.st);
    pi_reg4_reset(&me->pid_i.st);
    me->base.st = PST_RUN;
    if (me->base.pwm) {
      pwm_start(me->base.pwm);
    }
  }
}

static void buck_stop(PowerStage *base) {
  ModBuck *me = container_of(base, ModBuck, base);
  if (me->base.pwm) {
    pwm_stop(me->base.pwm);
  }
  me->base.st = PST_IDLE;
}

static void buck_emergency(PowerStage *base) {
  power_stage_emergency(base);   // 默认辅助: FAULT + pwm_emergency_stop
}

static void buck_set_ref(PowerStage *base, float vref, float iref) {
  ModBuck *me = container_of(base, ModBuck, base);
  me->cfg.vref = vref;
  me->cfg.iref = iref;
  mod_buck_sync_cfg(me);
}

static void buck_apply_tune(PowerStage *base, const float coef[10]) {
  ModBuck *me = container_of(base, ModBuck, base);
  me->cfg.vref             = coef[0];
  me->cfg.iref             = coef[1];
  me->cfg.pid_v.kp         = coef[2];
  me->cfg.pid_v.ki         = coef[3];
  me->cfg.pid_i.kp         = coef[4];
  me->cfg.pid_i.ki         = coef[5];
  me->cfg.ff_weight        = coef[6];
  me->cfg.ovp              = coef[7];
  me->cfg.ocp              = coef[8];
  me->cfg.soft_start_step  = coef[9];
  mod_buck_sync_cfg(me);
}

static PstSt buck_state(PowerStage *base) {
  return base->st;
}

// ======== 虚表 ========
static const PowerStageOps buck_ops = {
  .init       = buck_init,
  .tick       = buck_tick,
  .start      = buck_start,
  .stop       = buck_stop,
  .emergency  = buck_emergency,
  .set_ref    = buck_set_ref,
  .apply_tune = buck_apply_tune,
  .state      = buck_state,
};

// ======== 构造 ========

void mod_buck_init(ModBuck *me, const ModBuckCfg *cfg) {
  memset(me, 0, sizeof(*me));

  me->base.ops = &buck_ops;   // 绑定虚表

  if (cfg) {
    me->cfg = *cfg;
  }

  me->base.vref = me->cfg.vref;
  me->base.iref = me->cfg.iref;
  me->base.ovp  = me->cfg.ovp;
  me->base.ocp  = me->cfg.ocp;
  me->base.st   = PST_INIT;

  mod_buck_sync_cfg(me);   // 由 cfg 派生双环 PI 运行时参数
  pi_reg4_init(&me->pid_v.st);
  pi_reg4_init(&me->pid_i.st);
}

// ======== 配置同步 ========

void mod_buck_sync_cfg(ModBuck *me) {
  // 同步 PowerStage 基类公共参数
  me->base.vref = me->cfg.vref;
  me->base.iref = me->cfg.iref;
  me->base.ovp  = me->cfg.ovp;
  me->base.ocp  = me->cfg.ocp;

  // 电压环 PI: 输出 = 电流指令, 限幅 [0, base.iref] (限流)
  me->pid_v.cfg.kp      = me->cfg.pid_v.kp;
  me->pid_v.cfg.ki      = me->cfg.pid_v.ki;
  me->pid_v.cfg.kff     = 0.0f;
  me->pid_v.cfg.dt      = MOD_BUCK_DT;
  me->pid_v.cfg.out_max = me->base.iref;
  me->pid_v.cfg.out_min = 0.0f;
  me->pid_v.cfg.sp_fc   = 0.0f;

  // 电流环 PI: 输出 = 占空比分量, 限幅 [0, 1] (最终由 duty_min/max 钳)
  me->pid_i.cfg.kp      = me->cfg.pid_i.kp;
  me->pid_i.cfg.ki      = me->cfg.pid_i.ki;
  me->pid_i.cfg.kff     = 0.0f;
  me->pid_i.cfg.dt      = MOD_BUCK_DT;
  me->pid_i.cfg.out_max = 1.0f;
  me->pid_i.cfg.out_min = 0.0f;
  me->pid_i.cfg.sp_fc   = 0.0f;
}

// ======== 公开 API (包装 ops) ========

void mod_buck_tick(ModBuck *me) {
  if (me->base.ops && me->base.ops->tick) {
    me->base.ops->tick(&me->base);
  }
}

void mod_buck_start(ModBuck *me) {
  if (me->base.ops && me->base.ops->start) {
    me->base.ops->start(&me->base);
  }
}

void mod_buck_stop(ModBuck *me) {
  if (me->base.ops && me->base.ops->stop) {
    me->base.ops->stop(&me->base);
  }
}

void mod_buck_emergency(ModBuck *me) {
  if (me->base.ops && me->base.ops->emergency) {
    me->base.ops->emergency(&me->base);
  }
}

void mod_buck_set_ref(ModBuck *me, float vref, float iref) {
  if (me->base.ops && me->base.ops->set_ref) {
    me->base.ops->set_ref(&me->base, vref, iref);
  }
}

void mod_buck_apply_tune(ModBuck *me, const float coef[10]) {
  if (me->base.ops && me->base.ops->apply_tune) {
    me->base.ops->apply_tune(&me->base, coef);
  }
}

PstSt mod_buck_state(const ModBuck *me) {
  return me->base.st;
}
