// Buck 降压控制模块 — PowerStage 具体实现
//
// 每 tick 流程 (所有状态):
//   采样(vout/iout/vin = AdcDcSampler 工程量, 校准在采样器内) → 保护(vout>ovp / iout>ocp
//   去抖 → 急停; A6: 所有状态每周期检查, 防 IDLE/FAULT 下 PWM 意外输出无保护)
//   PST_RUN 内: 软启动(vref 从 0 每 tick +soft_start_step) → 电压环(err=vref-vout → 电流指令)
//   → 电流环(err=电流指令-iout → duty_pi) → 前馈(duty = clamp(duty_pi + ff·vref/vin))
//   → pwm_set_duty(ch_drive, duty)
//
// 状态机: INIT → IDLE → RUN ⇄ FAULT (start 恢复重新软启)
//   FAULT_HOLD / RECOVER 在枚举中保留, 供更复杂拓扑 (如多级保护确认) 扩展

#include "mod_buck.h"
#include "container_of.h"
#include "comp_math.h"  // math_clamp_f
#include <string.h>

// 控制周期 (秒) — 100us = 10kHz, 须与 App_OnControlTick 调用频率一致
#define MOD_BUCK_DT 0.0001f

// 故障去抖确认次数 (OVP/OCP 连续 N tick 触发才急停, 防噪声误报)
#define MOD_BUCK_FAULT_DEBOUNCE_N 8u

// ======== 内部辅助 ========

// 采样: AdcDcSampler 工程量 (k/b 校准已在采样器内部完成)
static inline void buck_sample(ModBuck *me) {
  me->vout = adc_dc_sampler_get_value(me->adc, me->cfg.adc_ch_vout);
  me->iout = adc_dc_sampler_get_value(me->adc, me->cfg.adc_ch_iout);
  me->vin = adc_dc_sampler_get_value(me->adc, me->cfg.adc_ch_vin);
  if (me->vin <= 0.0f) {
    me->vin = 1.0f;  // 防前馈除零
  }
  latch_write(&me->telemetry, me->vout);  // FAST→SLOW 遥测 (Latest 锁存, 故障周期也更新)
}

// 软启动: 当前参考每 tick 上升 soft_start_step, 限到目标; 参考下调时立即跟随
static inline float buck_soft_start(ModBuck *me, float target) {
  if (me->soft_start_ref < target) {
    me->soft_start_ref += me->cfg.soft_start_step;
    if (me->soft_start_ref > target) {
      me->soft_start_ref = target;
    }
  } else if (me->soft_start_ref > target) {
    me->soft_start_ref = target;  // set_ref 下调 vref 时防卡在高值
  }
  return me->soft_start_ref;
}

// 保护: OVP/OCP 去抖确认 → 急停 (A6: 所有状态每周期检查 — 防 IDLE/FAULT 下 PWM 意外输出无保护)
// 返回 true = 故障 (含去抖期): 调用方本周期不推进状态机输出 (不刷新 PWM)
// 事件只在新确认上升沿推一次 (fault_latched 防重复): power_stage_emergency 会清 debounce_cnt,
//   持续故障若不加锁会每 N tick 重复急停/重复推事件; 故障消失后锁复位, 允许下次重新上报
static bool buck_protect(ModBuck *me) {
  // 阈值读 PowerStage 基类 (base.ovp/ocp, 由 sync_cfg 从 cfg 同步)
  bool fault = (me->vout > me->base.ovp) || (me->iout > me->base.ocp);
  if (!fault) {
    me->base.debounce_cnt = 0;
    me->fault_latched = false;
    return false;
  }
  if (me->base.debounce_cnt < MOD_BUCK_FAULT_DEBOUNCE_N) {
    me->base.debounce_cnt++;
  }
  if (me->base.debounce_cnt >= MOD_BUCK_FAULT_DEBOUNCE_N && !me->fault_latched) {
    // 首次确认 (上升沿): 急停 + 事件流 (SPSC 环 fire-and-forget = IO_NONE; 满则计数丢弃, 不阻塞 FAST)
    power_stage_emergency(&me->base);  // 置 PST_FAULT + pwm_emergency_stop + 清 debounce_cnt
    me->fault_latched = true;
    if (me->vout > me->base.ovp) {
      if (!ring_push(&me->evt, MOD_BUCK_EVT_FAULT_OVP)) {
        me->evt_overflow_cnt++;
      }
    }
    if (me->iout > me->base.ocp) {
      if (!ring_push(&me->evt, MOD_BUCK_EVT_FAULT_OCP)) {
        me->evt_overflow_cnt++;
      }
    }
  }
  return true;  // 故障期间不刷新 PWM
}

// 运行态主控制循环: 软启 → 双环 → 前馈 → PWM (采样/保护已由 buck_tick 全状态执行)
static void buck_run(ModBuck *me) {
  // 设备未绑定则拒绝运行
  if (!me->pwm_buck || !me->adc) {
    me->base.st = PST_FAULT;
    return;
  }

  // 1. 软启动参考 (从 0 斜坡到基类 vref, 由 sync_cfg 从 cfg.vref 同步)
  float vref = buck_soft_start(me, me->base.vref);

  // 2. 电压环: err = vref - vout → 电流指令 (输出限幅 [0, iref])
  me->i_ref = pid_compute(&me->pid_v.base, vref, me->vout);

  // 3. 电流环: err = 电流指令 - iout → 占空比分量 (输出限幅 [0, 1])
  me->duty_pi = pid_compute(&me->pid_i.base, me->i_ref, me->iout);

  // 4. 前馈混合: D = duty_pi + ff_weight·(vref/vin), 钳到 [duty_min, duty_max]
  float ff = me->cfg.ff_weight * (vref / me->vin);
  me->duty = math_clamp_f(me->duty_pi + ff, me->cfg.duty_min, me->cfg.duty_max);

  // 5. PWM 输出 (pwm_set_duty 内部二次限幅到 base.duty_min/max)
  pwm_set_duty(&me->pwm_buck->base, me->cfg.ch_drive, me->duty);
}

// ======== ops 实现 (PowerStage* 第一参数, container_of 下溯) ========

static void buck_init(PowerStage *base) {
  ModBuck *me = container_of(base, ModBuck, base);
  pid_reset(&me->pid_v.base);
  pid_reset(&me->pid_i.base);
  me->soft_start_ref = 0.0f;
  me->vout = me->iout = me->vin = 0.0f;
  me->i_ref = me->duty_pi = me->duty = 0.0f;
  me->base.debounce_cnt = 0;
  me->fault_latched = false;
  me->base.st = PST_INIT;
}

static void buck_tick(PowerStage *base) {
  ModBuck *me = container_of(base, ModBuck, base);

  // 周期边界收取 MAIN 命令 (Command 邮箱): 最新命令生效, 跨上下文无锁安全
  uint32_t c;
  float a;
  if (mailbox_poll(&me->cmd, &c, &a)) {
    if (c == MOD_BUCK_CMD_SET_VREF) {
      me->cfg.vref = a;
      mod_buck_sync_cfg(me);
    }
  }

  // 采样 + 保护: 所有状态每周期执行 (A6) — IDLE/FAULT 下若 PWM 意外输出, OVP/OCP 仍生效
  // 设备未绑定则无从采样 (init 后 board_init 已绑定; 未绑定保持 0 值不误报)
  if (me->adc != NULL) {
    buck_sample(me);
  }
  if (buck_protect(me)) {
    return;  // 故障 (含去抖期): 不推进状态机输出
  }

  switch (me->base.st) {
  case PST_INIT:
    // 自检通过 → 空闲 (真实工程在此检查 PWM/ADC 就绪)
    me->base.st = PST_IDLE;
    break;

  case PST_IDLE:
    break;  // 等待 start

  case PST_RUN:
    buck_run(me);
    break;

  case PST_FAULT_HOLD:
  case PST_FAULT:
    break;  // 保持封波, 等待 start 恢复

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
  if (me->base.st == PST_IDLE || me->base.st == PST_FAULT || me->base.st == PST_RECOVER) {
    me->soft_start_ref = 0.0f;  // 重新软启动
    me->base.debounce_cnt = 0;
    me->fault_latched = false;  // 允许新故障再次上报 (恢复后故障仍在 → 去抖重新确认即再跳闸)
    // 清双环积分器 — FAULT 急停后残留的饱和积分会顶满电流指令, 破坏软启动
    pid_reset(&me->pid_v.base);
    pid_reset(&me->pid_i.base);
    me->base.st = PST_RUN;
    // 不进事件环: start/stop 常由 MAIN (HMI/命令) 调用, 事件环单生产者必须为 FAST
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
  // 同 buck_start: START/STOP 状态变化由调用方读 mod_buck_state 观察, 不推事件环
}

static void buck_emergency(PowerStage *base) {
  power_stage_emergency(base);  // 默认辅助: FAULT + pwm_emergency_stop
}

static void buck_set_ref(PowerStage *base, float vref, float iref) {
  // 同步批量改参考 (MAIN, 启动/配置期); 运行期热调单值走 mod_buck_set_vref (命令邮箱周期边界生效)
  ModBuck *me = container_of(base, ModBuck, base);
  me->cfg.vref = vref;
  me->cfg.iref = iref;
  mod_buck_sync_cfg(me);
}

static void buck_apply_tune(PowerStage *base, const float coef[10]) {
  ModBuck *me = container_of(base, ModBuck, base);
  me->cfg.vref = coef[0];
  me->cfg.iref = coef[1];
  me->cfg.pid_v.kp = coef[2];
  me->cfg.pid_v.ki = coef[3];
  me->cfg.pid_i.kp = coef[4];
  me->cfg.pid_i.ki = coef[5];
  me->cfg.ff_weight = coef[6];
  me->cfg.ovp = coef[7];
  me->cfg.ocp = coef[8];
  me->cfg.soft_start_step = coef[9];
  mod_buck_sync_cfg(me);
}

static PstSt buck_state(PowerStage *base) {
  return base->st;
}

// ======== 虚表 ========
static const PowerStageOps buck_ops = {
    .init = buck_init,
    .tick = buck_tick,
    .start = buck_start,
    .stop = buck_stop,
    .emergency = buck_emergency,
    .set_ref = buck_set_ref,
    .apply_tune = buck_apply_tune,
    .state = buck_state,
};

// ======== 构造 ========

void mod_buck_init(ModBuck *me, const ModBuckCfg *cfg) {
  memset(me, 0, sizeof(*me));

  // 五原语交接点初始化 (Latest 锁存 / Command 邮箱 / SPSC 事件环)
  ring_init(&me->evt, me->evt_buf, sizeof(me->evt_buf));
  latch_init(&me->telemetry);
  mailbox_init(&me->cmd);

  me->base.ops = &buck_ops;  // 绑定虚表

  if (cfg) {
    me->cfg = *cfg;
  }

  me->base.vref = me->cfg.vref;
  me->base.iref = me->cfg.iref;
  me->base.ovp = me->cfg.ovp;
  me->base.ocp = me->cfg.ocp;
  me->base.st = PST_INIT;

  // 双环 PI (每个都是 PidReg4 子类, TI pi_reg4 语义 = buck 原 PidLinear aw=CLAMP 位级):
  //   初值由 pid_reg4_init 种默认, 增益/限幅由下方 sync_cfg 派生
  PidReg4Cfg r4 = { 0.0f, 0.0f, 0.0f, 0.0f };   // kp, ki, kff, sp_fc (sync_cfg 填)
  pid_reg4_init(&me->pid_v, MOD_BUCK_DT, 0.0f, me->base.iref, &r4);
  pid_reg4_init(&me->pid_i, MOD_BUCK_DT, 0.0f, 1.0f, &r4);
  // 挂到 PowerStage.loop 槽位 (PidBase*, 供框架访问)
  me->base.loop[0] = &me->pid_v.base; // 外环: 电压 → 电流指令
  me->base.loop[1] = &me->pid_i.base; // 内环: 电流 → 占空比
  mod_buck_sync_cfg(me);  // 由 cfg 派生双环 PI 运行时参数
}

// ======== 配置同步 ========

void mod_buck_sync_cfg(ModBuck *me) {
  // 双上下文可写: FAST (mailbox SET_VREF 分支) 与 MAIN (apply_tune/set_ref) 都会调本函数
  // float 写原子 + 单一抢占源 → 瞬态混用最多一周期, 属调参容忍范围; 不做跨上下文并发同步
  // 同步 PowerStage 基类公共参数
  me->base.vref = me->cfg.vref;
  me->base.iref = me->cfg.iref;
  me->base.ovp = me->cfg.ovp;
  me->base.ocp = me->cfg.ocp;

  // 电压环 PI (PidReg4): 输出 = 电流指令, 限幅 [0, base.iref] (限流)
  me->pid_v.cfg.kp  = me->cfg.pid_v.kp;
  me->pid_v.cfg.ki  = me->cfg.pid_v.ki;
  me->pid_v.cfg.kff = 0.0f;
  me->pid_v.cfg.sp_fc = 0.0f;
  me->pid_v.base.dt = MOD_BUCK_DT;
  me->pid_v.base.out_max = me->base.iref;
  me->pid_v.base.out_min = 0.0f;

  // 电流环 PI (PidReg4): 输出 = 占空比分量, 限幅 [0, 1] (最终由 duty_min/max 钳)
  me->pid_i.cfg.kp  = me->cfg.pid_i.kp;
  me->pid_i.cfg.ki  = me->cfg.pid_i.ki;
  me->pid_i.cfg.kff = 0.0f;
  me->pid_i.cfg.sp_fc = 0.0f;
  me->pid_i.base.dt = MOD_BUCK_DT;
  me->pid_i.base.out_max = 1.0f;
  me->pid_i.base.out_min = 0.0f;
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
