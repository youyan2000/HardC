// 三相均流模块 — 并联相电流均衡 + 模式迟滞 (实现)
//
// 每 FAST tick (gen_share_tick):
//   ratio = clamp(vb/va, 0.73, 1.37) → 模式迟滞 (进 0.97/1.03, 出 0.90/1.10)
//   iavg = mean(i_phase), ibase = paside / N
//   每相: share_error = clamp(share_gain×(iavg−iphase), ±share_lim)
//         u = PI(setpoint=ibase+share_error, feedback=iphase) → alpha = 1+u
//   ModeSync: 模式变化周期三相统一用平均 alpha
//   写 PWM: set_mode(变化时) / set_ratio(变化时) / 逐相 set_alpha(每周期)

#include "gen_current_share.h"
#include <string.h>

// 控制周期 (秒) — 固定 28.3kHz 契约: 须与 App_OnControlTick 调用频率一致 (与 gen_supercap 同源)
//   (与 gen_supercap 的 cfg.control_freq_hz 参数化不同, 本模块保持固定 28333 — 若需参数化
//    由工程在 gen_current_share.h 增加 cfg 字段后派生, 当前保持最小改动)
#define GEN_SHARE_DT (1.0f / 28333.0f)

// 占空比律输入钳位 — 与 pwm_buckboost 律常量一致 (ratio_lo/hi, 律在 Device 内部再次钳位)
#define GEN_SHARE_RATIO_LO 0.73f
#define GEN_SHARE_RATIO_HI 1.37f

// ======== 构造 ========

void gen_share_init(GenCurrentShare *me, const GenShareCfg *cfg) {
  memset(me, 0, sizeof(*me));

  GenShareCfg d;
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
  if (d.num_phases > GEN_SHARE_MAX_PHASES) {
    d.num_phases = GEN_SHARE_MAX_PHASES;
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

  // 每相电流 PI (PidReg4 = TI pi_reg4, 复刻原 PidLinear aw=CLAMP):
  //   配置构造期一次性派生 (pid_kp/pid_ki 无运行时调参路径)
  PidReg4Cfg pc = {me->cfg.pid_kp, me->cfg.pid_ki, 0.0f, 0.0f};  // kp, ki, kff, sp_fc

  for (uint8_t i = 0; i < GEN_SHARE_MAX_PHASES; i++) {
    pid_reg4_init(&me->ph[i].pi, GEN_SHARE_DT, -GEN_SHARE_ALPHA_DELTA_MAX, GEN_SHARE_ALPHA_DELTA_MAX, &pc);
    me->ph[i].alpha = 1.0f;
  }
}

void gen_share_bind(GenCurrentShare *me, PwmBuckBoost *pwm) {
  me->pwm = pwm;
}

// ======== FAST 单写者注入 ========

void gen_share_set_voltages(GenCurrentShare *me, float va, float vb) {
  me->va = va;
  me->vb = vb;
}

void gen_share_set_paside(GenCurrentShare *me, float paside) {
  me->paside = paside;
}

void gen_share_set_currents(GenCurrentShare *me, const float *i) {
  uint8_t n = me->cfg.num_phases;
  if (n < 1u) {
    n = 1u;
  }
  for (uint8_t k = 0; k < n; k++) {
    me->i_phase[k] = i[k];
  }
}

// ======== FAST tick ========

void gen_share_tick(GenCurrentShare *me) {
  if (!me->pwm || me->fault) {
    return;
  }
  uint8_t n = me->cfg.num_phases;
  if (n < 1u) {
    n = 1u;
  }

  // 1. 电压比 → 模式迟滞 (钳位到律窗口)
  // va 下限防除零 (母线电压 <1V 视为无效采样, 用 1V 兜底保 ratio 有界)
  float va = me->va > 1.0f ? me->va : 1.0f;
  me->ratio = me->vb / va;
  me->ratio = math_clamp_f(me->ratio, GEN_SHARE_RATIO_LO, GEN_SHARE_RATIO_HI);
  bool bb = Hysteresis_Update(&me->mode_hyst, me->ratio);
  // 模式判定与迟滞退出边界一致 (0.90/1.10): bb=false 时在窗口内保持上次子模式 (迟滞记忆),
  // 超出退出边界才切 Buck/Boost — 避免"迟滞窗口内按 1.0 误判"导致模式抖动
  uint8_t mode = PwmMode_BuckBoost;
  if (!bb) {
    if (me->ratio <= me->cfg.hyst_exit_lo) {
      mode = (uint8_t) PwmMode_Buck;
    } else if (me->ratio >= me->cfg.hyst_exit_hi) {
      mode = (uint8_t) PwmMode_Boost;
    } else {
      // 窗口内: 保持上次子模式 (初次默认 Buck)
      mode = (me->last_written_mode == (uint8_t) PwmMode_Boost) ? (uint8_t) PwmMode_Boost : (uint8_t) PwmMode_Buck;
    }
  }
  me->cur_mode = mode;

  // 2. 相电流均值 + 每相基准
  float sum = 0.0f;
  for (uint8_t i = 0; i < n; i++) {
    sum += me->i_phase[i];
  }
  me->iavg = sum / (float) n;
  me->ibase = me->paside / (float) n;

  // 3. 每相均流修正 + 电流 PI → alpha (各相共用 PI 配置, 构造期已派生进实例)
  float alpha[GEN_SHARE_MAX_PHASES];
  for (uint8_t i = 0; i < n; i++) {
    float err = me->cfg.share_gain * (me->iavg - me->i_phase[i]);
    me->ph[i].share_error = math_clamp_f(err, -me->cfg.share_lim, me->cfg.share_lim);
    float sp = me->ibase + me->ph[i].share_error;
    float u = pid_compute(&me->ph[i].pi.base, sp, me->i_phase[i]);
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

void gen_share_emergency(GenCurrentShare *me) {
  me->fault = true;
  me->paside = 0.0f;
  uint8_t n = me->cfg.num_phases;
  if (n < 1u) {
    n = 1u;
  }
  for (uint8_t i = 0; i < n; i++) {
    pid_reset(&me->ph[i].pi.base);
    me->ph[i].alpha = 0.0f;
  }
}

void gen_share_release(GenCurrentShare *me) {
  me->fault = false;
  uint8_t n = me->cfg.num_phases;
  if (n < 1u) {
    n = 1u;
  }
  for (uint8_t i = 0; i < n; i++) {
    pid_reset(&me->ph[i].pi.base);
    me->ph[i].alpha = 1.0f;
  }
  me->last_written_mode = 0xFFu;  // 强制下次 tick 重写模式
}
