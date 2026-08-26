// 三相电压源逆变器 (VSI) 控制模块 — PowerStage 具体实现 (六开关三相逆变)
//
// 每 tick 流程 (所有状态):
//   采样(adc_ac_sampler fast_fetch → ia/ib/ic + vdc) → 保护(vdc>ovp / irms>ocp 去抖 → 急停)
//   → PLL(SRF 锁相 Vab) → Clarke/Park(电流) → dq 双环(电流) → 反Park → SVPWM 发波
//
// 状态机: INIT → IDLE → RUN ⇄ FAULT (start 恢复重新软启)
//   FAULT_HOLD / RECOVER 在枚举中保留, 供更复杂拓扑扩展
//
// 采样契约: App FAST ISR 须在每控制周期先调 adc_ac_sampler_fast_fetch() (见 app_main.h.tmpl).
//   本模块每 tick 直接读采样器的 va/vb/vc + ia1/ib1/ic1 + vdc 工程量字段.

#include "gen_vsi.h"
#include "container_of.h"
#include "comp_math.h"  // math_clamp_f / math_sqrt_f
#include <string.h>

// 控制周期 (秒) — 50us = 20kHz, 须与 App_OnControlTick 调用频率一致
#define GEN_VSI_DT 0.00005f

// 故障去抖确认次数 (OVP/OCP 连续 N tick 触发才急停, 防噪声误报)
#define GEN_VSI_OVP_DEBOUNCE_N 8u
#define GEN_VSI_OCP_DEBOUNCE_N 8u

// ======== 内部辅助 ========

// 采样: 直接读 AdcAcSampler 的工程量字段 (fast_fetch 已完成三相重构/RMS)
static inline void vsi_sample(GenVsi *me) {
  const AdcAcSampler *adc = me->adc;
  me->va = adc->va;
  me->vb = adc->vb;
  me->vc = adc->vc;
  me->ia = adc->ia1;
  me->ib = adc->ib1;
  me->ic = adc->ic1;
  me->vdc = adc->vdc;
  if (me->vdc <= 0.0f) {
    me->vdc = 1.0f;  // 防 SVPWM 除零
  }
  latch_write(&me->telemetry, me->vdc);  // FAST→SLOW 遥测 (Latest 锁存, 故障周期也更新)
}

// 保护: OVP(母线)/OCP(相电流 RMS) 去抖确认 → 急停 (A6: 所有状态每周期检查)
static bool vsi_protect(GenVsi *me) {
  // 阈值读 PowerStage 基类 (base.ovp/ocp, 由 sync_cfg 从 cfg 同步)
  bool ovp = (me->vdc > me->base.ovp);
  bool ocp = (me->adc != NULL) ? (me->adc->irms1 > me->base.ocp) : false;
  bool fault = ovp || ocp;
  if (!fault) {
    me->base.debounce_cnt = 0;
    me->fault_active = false;
    return false;
  }
  if (me->base.debounce_cnt < GEN_VSI_OVP_DEBOUNCE_N) {
    me->base.debounce_cnt++;
  }
  if (me->base.debounce_cnt >= GEN_VSI_OVP_DEBOUNCE_N && !me->fault_active) {
    // 首次确认 (上升沿): 急停 + 事件流 (SPSC 环 fire-and-forget = IO_NONE; 满则计数丢弃, 不阻塞 FAST)
    power_stage_emergency(&me->base);  // 置 PST_FAULT + pwm_emergency_stop + 清 debounce_cnt
    me->fault_active = true;
    if (ovp) {
      if (!ring_push(&me->evt, GEN_VSI_EVT_FAULT_OVP)) {
        me->evt_overflow_cnt++;
      }
    }
    if (ocp) {
      if (!ring_push(&me->evt, GEN_VSI_EVT_FAULT_OCP)) {
        me->evt_overflow_cnt++;
      }
    }
  }
  return true;  // 故障期间不刷新 PWM
}

// 运行态主控制循环: PLL → Park → 双环 → 反Park → SVPWM (采样/保护已由 vsi_tick 全状态执行)
static void vsi_run(GenVsi *me) {
  // 设备未绑定则拒绝运行
  if (!me->pwm || !me->adc) {
    me->base.st = PST_FAULT;
    return;
  }

  // 1. PLL: SRF-LOCK on Vab 线电压 → theta (PllSrf, 输入 αβ)
  PllInput pin = {0};
  // 用线电压 Vab/Vbc 合成 αβ (重构相电压 va/vb/vc 在采样器内已就绪)
  //   α = Va, β = (Vb - Vc)/√3 (三相形式)
  pin.v_alpha = me->va;
  pin.v_beta = (me->vb - me->vc) * ONE_OVER_SQRT3;
  pll_run(&me->pll.base, &pin);
  me->theta = me->pll.base.theta;

  // 2. 电流 Clarke + Park (dq 定向)
  float sa = (float) sinf(me->theta);
  float ca = (float) cosf(me->theta);
  Park cur = {.alpha = me->ia, .beta = (me->ia + 2.0f * me->ib) * ONE_OVER_SQRT3, .sine = sa, .cosine = ca};
  park_run(&cur);
  me->id = cur.ds;
  me->iq = cur.qs;

  // 3. 电流环 (可选母线电压外环 → d 轴 id_ref)
  float id_ref = me->cfg.id_ref;
  if (me->cfg.vdc_ctrl_enable > 0.5f) {
    id_ref = pid_compute(&me->pid_vdc.base, me->cfg.vdc_ref, me->vdc);
  }
  me->vd_ref = pid_compute(&me->pid_id.base, id_ref, me->id);
  me->vq_ref = pid_compute(&me->pid_iq.base, me->cfg.iq_ref, me->iq);

  // 4. 反 Park → αβ (V), 归一化为标幺 (除 vdc) → SVPWM
  ParkInv ip = {.ds = me->vd_ref, .qs = me->vq_ref, .sine = sa, .cosine = ca};
  park_inv_run(&ip);
  me->v_alpha = ip.alpha / me->vdc;
  me->v_beta = ip.beta / me->vdc;

  // 5. SVPWM 发波 (内部算扇区 + 6 路占空比)
  svpwm_set_vector(me->pwm, me->v_alpha, me->v_beta, me->vdc);
  me->duty_a = me->pwm->duty_a;
  me->duty_b = me->pwm->duty_b;
  me->duty_c = me->pwm->duty_c;
  me->cur_mod_index = svpwm_get_modulation_index(me->pwm);
}

// ======== ops 实现 (PowerStage* 第一参数, container_of 下溯) ========

static void vsi_init(PowerStage *base) {
  GenVsi *me = container_of(base, GenVsi, base);
  pid_reset(&me->pid_id.base);
  pid_reset(&me->pid_iq.base);
  pid_reset(&me->pid_vdc.base);
  pll_reset(&me->pll.base);
  me->va = me->vb = me->vc = 0.0f;
  me->ia = me->ib = me->ic = 0.0f;
  me->vdc = 0.0f;
  me->id = me->iq = 0.0f;
  me->vd_ref = me->vq_ref = 0.0f;
  me->v_alpha = me->v_beta = 0.0f;
  me->duty_a = me->duty_b = me->duty_c = 0.0f;
  me->base.debounce_cnt = 0;
  me->fault_active = false;
  me->base.st = PST_INIT;
}

static void vsi_tick(PowerStage *base) {
  GenVsi *me = container_of(base, GenVsi, base);

  // 周期边界收取 MAIN 命令 (Command 邮箱): 最新命令生效, 跨上下文无锁安全
  uint32_t c;
  float a;
  if (mailbox_poll(&me->cmd, &c, &a)) {
    if (c == GEN_VSI_CMD_SET_REF) {
      me->cfg.id_ref = a;
      gen_vsi_sync_cfg(me);
    }
  }

  // 采样 + 保护: 所有状态每周期执行 (A6)
  if (me->adc != NULL) {
    vsi_sample(me);
  }
  if (vsi_protect(me)) {
    return;  // 故障 (含去抖期): 不推进状态机输出
  }

  switch (me->base.st) {
  case PST_INIT:
    me->base.st = PST_IDLE;
    break;

  case PST_IDLE:
    break;  // 等待 start

  case PST_RUN:
    vsi_run(me);
    break;

  case PST_FAULT_HOLD:
  case PST_FAULT:
    break;  // 保持封波, 等待 start 恢复

  case PST_RECOVER:
    me->base.st = PST_RUN;
    break;

  default:
    break;
  }
}

static void vsi_start(PowerStage *base) {
  GenVsi *me = container_of(base, GenVsi, base);
  if (me->base.st == PST_IDLE || me->base.st == PST_FAULT || me->base.st == PST_RECOVER) {
    me->base.debounce_cnt = 0;
    me->fault_active = false;
    pid_reset(&me->pid_id.base);
    pid_reset(&me->pid_iq.base);
    pid_reset(&me->pid_vdc.base);
    pll_reset(&me->pll.base);
    me->base.st = PST_RUN;
    if (me->base.pwm) {
      pwm_start(me->base.pwm);
    }
  }
}

static void vsi_stop(PowerStage *base) {
  GenVsi *me = container_of(base, GenVsi, base);
  if (me->base.pwm) {
    pwm_stop(me->base.pwm);
  }
  me->base.st = PST_IDLE;
}

static void vsi_emergency(PowerStage *base) {
  power_stage_emergency(base);  // 默认辅助: FAULT + pwm_emergency_stop
}

static void vsi_set_ref(PowerStage *base, float vref, float iref) {
  GenVsi *me = container_of(base, GenVsi, base);
  me->cfg.vdc_ref = vref;
  me->cfg.id_ref = iref;
  gen_vsi_sync_cfg(me);
}

static void vsi_apply_tune(PowerStage *base, const float coef[10]) {
  GenVsi *me = container_of(base, GenVsi, base);
  me->cfg.vdc_ref = coef[0];
  me->cfg.id_ref = coef[1];
  me->cfg.iq_ref = coef[2];
  me->cfg.pid_id.kp = coef[3];
  me->cfg.pid_id.ki = coef[4];
  me->cfg.pid_iq.kp = coef[5];
  me->cfg.pid_iq.ki = coef[6];
  me->cfg.pll_bw = coef[7];
  me->cfg.ovp = coef[8];
  me->cfg.ocp = coef[9];
  gen_vsi_sync_cfg(me);
}

static PstSt vsi_state(PowerStage *base) {
  return base->st;
}

// ======== 虚表 ========
static const PowerStageOps vsi_ops = {
    .init = vsi_init,
    .tick = vsi_tick,
    .start = vsi_start,
    .stop = vsi_stop,
    .emergency = vsi_emergency,
    .set_ref = vsi_set_ref,
    .apply_tune = vsi_apply_tune,
    .state = vsi_state,
};

// ======== 构造 ========

void gen_vsi_init(GenVsi *me, const GenVsiCfg *cfg, float grid_freq_hz) {
  memset(me, 0, sizeof(*me));

  // 五原语交接点初始化
  ring_init(&me->evt, me->evt_buf, sizeof(me->evt_buf));
  latch_init(&me->telemetry);
  mailbox_init(&me->cmd);

  me->base.ops = &vsi_ops;  // 绑定虚表

  if (cfg) {
    me->cfg = *cfg;
  }

  me->base.vref = me->cfg.vdc_ref;
  me->base.iref = me->cfg.iq_ref;
  me->base.ovp = me->cfg.ovp;
  me->base.ocp = me->cfg.ocp;
  me->base.st = PST_INIT;

  // PLL (SRF-PLL): 频带宽由 sync_cfg 由 cfg.pll_bw 派生 kp/ki
  pll_srf_init(&me->pll, grid_freq_hz, GEN_VSI_DT, 0.0f, 0.0f);

  // dq 双环 + 可选母线电压环 (PidReg4)
  PidReg4Cfg r4 = {0.0f, 0.0f, 0.0f, 0.0f};
  pid_reg4_init(&me->pid_id, GEN_VSI_DT, 0.0f, me->cfg.v_dq_max > 0.0f ? me->cfg.v_dq_max : 0.577f, &r4);
  pid_reg4_init(&me->pid_iq, GEN_VSI_DT, 0.0f, me->cfg.v_dq_max > 0.0f ? me->cfg.v_dq_max : 0.577f, &r4);
  pid_reg4_init(&me->pid_vdc, GEN_VSI_DT, 0.0f, 0.0f, &r4);
  me->base.loop[0] = &me->pid_id.base;
  me->base.loop[1] = &me->pid_iq.base;
  gen_vsi_sync_cfg(me);  // 由 cfg 派生 PI/PLL/保护
}

// ======== 配置同步 ========

void gen_vsi_sync_cfg(GenVsi *me) {
  // 同步 PowerStage 基类公共参数
  me->base.vref = me->cfg.vdc_ref;
  me->base.iref = me->cfg.iq_ref;
  me->base.ovp = me->cfg.ovp;
  me->base.ocp = me->cfg.ocp;

  // PLL PI: 由带宽派生 kp/ki (标准 SRF-PLL 整定: kp = bw·√2, ki ≈ bw²)
  float bw = me->cfg.pll_bw > 0.0f ? me->cfg.pll_bw : 10.0f;
  float wz = bw * 1.41421356f;  // ω_z ≈ √2·bw
  pll_base_set_pi(&me->pll.base, wz, bw * bw);

  // d 轴电流环
  me->pid_id.cfg.kp = me->cfg.pid_id.kp;
  me->pid_id.cfg.ki = me->cfg.pid_id.ki;
  me->pid_id.cfg.kff = 0.0f;
  me->pid_id.cfg.sp_fc = 0.0f;
  me->pid_id.base.dt = GEN_VSI_DT;
  me->pid_id.base.out_max = me->vdc * 0.577f;
  me->pid_id.base.out_min = -me->vdc * 0.577f;

  // q 轴电流环
  me->pid_iq.cfg.kp = me->cfg.pid_iq.kp;
  me->pid_iq.cfg.ki = me->cfg.pid_iq.ki;
  me->pid_iq.cfg.kff = 0.0f;
  me->pid_iq.cfg.sp_fc = 0.0f;
  me->pid_iq.base.dt = GEN_VSI_DT;
  me->pid_iq.base.out_max = me->vdc * 0.577f;
  me->pid_iq.base.out_min = -me->vdc * 0.577f;

  // 母线电压环 (外环 → id_ref)
  me->pid_vdc.cfg.kp = 0.0f;
  me->pid_vdc.cfg.ki = 0.0f;
  me->pid_vdc.cfg.kff = 0.0f;
  me->pid_vdc.cfg.sp_fc = 0.0f;
  me->pid_vdc.base.dt = GEN_VSI_DT;
  me->pid_vdc.base.out_max = me->cfg.ocp * 0.5f;
  me->pid_vdc.base.out_min = 0.0f;
}

// ======== 公开 API (包装 ops) ========

void gen_vsi_tick(GenVsi *me) {
  if (me->base.ops && me->base.ops->tick) {
    me->base.ops->tick(&me->base);
  }
}

void gen_vsi_start(GenVsi *me) {
  if (me->base.ops && me->base.ops->start) {
    me->base.ops->start(&me->base);
  }
}

void gen_vsi_stop(GenVsi *me) {
  if (me->base.ops && me->base.ops->stop) {
    me->base.ops->stop(&me->base);
  }
}

void gen_vsi_emergency(GenVsi *me) {
  if (me->base.ops && me->base.ops->emergency) {
    me->base.ops->emergency(&me->base);
  }
}

void gen_vsi_apply_tune(GenVsi *me, const float coef[10]) {
  if (me->base.ops && me->base.ops->apply_tune) {
    me->base.ops->apply_tune(&me->base, coef);
  }
}

PstSt gen_vsi_state(const GenVsi *me) {
  return me->base.st;
}
