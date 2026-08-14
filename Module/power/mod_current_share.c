// 三相均流模块 — 并联相电流均衡 + 模式迟滞 (实现)
//
// 每 FAST tick (mod_share_tick):
//   ratio = clamp(vb/va, 0.73, 1.37) → 模式迟滞 (进 0.97/1.03, 出 0.90/1.10)
//   iavg = mean(i_phase), ibase = paside / N
//   每相: share_error = clamp(share_gain×(iavg−iphase), ±share_lim)
//         u = PI(setpoint=ibase+share_error, feedback=iphase) → alpha = 1+u
//   ModeSync: 模式变化周期三相统一用平均 alpha
//   写 PWM: set_mode(变化时) / set_ratio(变化时) / 逐相 set_alpha(每周期)

#include "mod_current_share.h"
#include <string.h>

// 控制周期 (秒) — 28.3kHz, 须与 App_OnControlTick 调用频率一致 (与 mod_supercap 同源)
#define MOD_SHARE_DT (1.0f / 28333.0f)

// 占空比律输入钳位 — 与 pwm_buckboost 律常量一致 (ratio_lo/hi, 律在 Device 内部再次钳位)
#define MOD_SHARE_RATIO_LO 0.73f
#define MOD_SHARE_RATIO_HI 1.37f

// ======== 构造 ========

void mod_share_init(ModCurrentShare *me, const ModShareCfg *cfg) {
  memset(me, 0, sizeof(*me));

  ModShareCfg d;
  if (cfg) {
    d = *cfg;
  } else {
    d.num_phases = 1;
    d.share_gain = 2.0f;
    d.share_lim = 0.04f;
    d.hyst_enter_lo = 0.97f;
    d.hyst_enter_hi = 1.03f;
    d.hyst_exit_lo = 0.90f;
    d.hyst_exit_hi = 1.10f;
    d.pid_kp = 0.5f;
    d.pid_ki = 0.0f;
  }
  if (d.num_phases == 0u) {
    d.num_phases = 1u;
  }
  if (d.num_phases > MOD_SHARE_MAX_PHASES) {
    d.num_phases = MOD_SHARE_MAX_PHASES;
  }
  if (d.share_lim <= 0.0f) {
    d.share_lim = 0.04f;
  }
  me->cfg = d;

  Hysteresis_Init(&me->mode_hyst, d.hyst_enter_lo, d.hyst_enter_hi, d.hyst_exit_lo, d.hyst_exit_hi);
  ModeSync_Init(&me->sync, d.num_phases);

  me->va = 1.0f;
  me->vb = 1.0f;
  me->ratio = 1.0f;
  me->last_written_mode = 0xFFu;  // 无效值, 首 tick 强制写模式
  me->fault = false;

  for (uint8_t i = 0; i < MOD_SHARE_MAX_PHASES; i++) {
    pi_reg4_init(&me->ph[i].pi);
    me->ph[i].alpha = 1.0f;
  }
}

void mod_share_bind(ModCurrentShare *me, PwmBuckBoost *pwm) {
  me->pwm = pwm;
}

// ======== FAST 单写者注入 ========

void mod_share_set_voltages(ModCurrentShare *me, float va, float vb) {
  me->va = va;
  me->vb = vb;
}

void mod_share_set_paside(ModCurrentShare *me, float paside) {
  me->paside = paside;
}

void mod_share_set_currents(ModCurrentShare *me, const float *i) {
  uint8_t n = me->cfg.num_phases;
  if (n < 1u) {
    n = 1u;
  }
  for (uint8_t k = 0; k < n; k++) {
    me->i_phase[k] = i[k];
  }
}

// ======== FAST tick ========

void mod_share_tick(ModCurrentShare *me) {
  if (!me->pwm || me->fault) {
    return;
  }
  uint8_t n = me->cfg.num_phases;
  if (n < 1u) {
    n = 1u;
  }

  // 1. 电压比 → 模式迟滞 (钳位到律窗口)
  float va = me->va > 1.0f ? me->va : 1.0f;
  me->ratio = me->vb / va;
  me->ratio = math_clamp_f(me->ratio, MOD_SHARE_RATIO_LO, MOD_SHARE_RATIO_HI);
  bool bb = Hysteresis_Update(&me->mode_hyst, me->ratio);
  uint8_t mode =
      bb ? (uint8_t) PwmMode_BuckBoost : (me->ratio < 1.0f ? (uint8_t) PwmMode_Buck : (uint8_t) PwmMode_Boost);
  me->cur_mode = mode;

  // 2. 相电流均值 + 每相基准
  float sum = 0.0f;
  for (uint8_t i = 0; i < n; i++) {
    sum += me->i_phase[i];
  }
  me->iavg = sum / (float) n;
  me->ibase = me->paside / (float) n;

  // 3. 每相均流修正 + 电流 PI → alpha (各相共用 PI 配置)
  PiReg4Cfg pc;
  pc.kp = me->cfg.pid_kp;
  pc.ki = me->cfg.pid_ki;
  pc.kff = 0.0f;
  pc.dt = MOD_SHARE_DT;
  pc.out_max = MOD_SHARE_ALPHA_DELTA_MAX;
  pc.out_min = -MOD_SHARE_ALPHA_DELTA_MAX;
  pc.sp_fc = 0.0f;

  float alpha[MOD_SHARE_MAX_PHASES];
  for (uint8_t i = 0; i < n; i++) {
    float err = me->cfg.share_gain * (me->iavg - me->i_phase[i]);
    me->ph[i].share_error = math_clamp_f(err, -me->cfg.share_lim, me->cfg.share_lim);
    float sp = me->ibase + me->ph[i].share_error;
    float u = pi_reg4_run(&me->ph[i].pi, &pc, sp, me->i_phase[i]);
    me->ph[i].alpha = 1.0f + u;
    alpha[i] = me->ph[i].alpha;
  }

  // 4. 模式切换单周期同步: 变化周期三相统一用平均 alpha (防环流)
  if (ModeSync_Update(&me->sync, mode, alpha)) {
    for (uint8_t i = 0; i < n; i++) {
      me->ph[i].alpha = me->sync.avg_cmd;
      alpha[i] = me->sync.avg_cmd;
    }
  }

  // 5. 写 PWM (模式/电压比仅变化时写, alpha 逐相每周期写 — 28kHz 热路径)
  if (mode != me->last_written_mode) {
    pwm_bb_set_mode(me->pwm, (PwmMode) mode);
    me->last_written_mode = mode;
  }
  if (me->ratio != me->last_written_ratio) {
    pwm_bb_set_ratio(me->pwm, me->ratio);
    me->last_written_ratio = me->ratio;
  }
  for (uint8_t i = 0; i < n; i++) {
    pwm_bb_set_alpha(me->pwm, i, me->ph[i].alpha);
  }
}

// ======== 急停 / 恢复 ========

void mod_share_emergency(ModCurrentShare *me) {
  me->fault = true;
  me->paside = 0.0f;
  uint8_t n = me->cfg.num_phases;
  if (n < 1u) {
    n = 1u;
  }
  for (uint8_t i = 0; i < n; i++) {
    pi_reg4_reset(&me->ph[i].pi);
    me->ph[i].alpha = 0.0f;
  }
}

void mod_share_release(ModCurrentShare *me) {
  me->fault = false;
  uint8_t n = me->cfg.num_phases;
  if (n < 1u) {
    n = 1u;
  }
  for (uint8_t i = 0; i < n; i++) {
    pi_reg4_reset(&me->ph[i].pi);
    me->ph[i].alpha = 1.0f;
  }
  me->last_written_mode = 0xFFu;  // 强制下次 tick 重写模式
}
