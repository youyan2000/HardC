// 超级电容功率控制模块 — PowerStage 派生 (实现)
//
// 每 FAST tick (PST_RUN):
//   采样(va/vb/i_phase/icap/ichassis) → 保护(短路/失平衡去抖→急停; 满电/低压迟滞削边界)
//   → 级联功率环(LPF→PI→限幅→taper→i_side) → 注入均流 + 驱动 share tick → PWM
//
// 状态机: INIT → IDLE → RUN ⇄ FAULT (start 恢复重新软启)
//   级联绕过 base.loop[2] (per-phase PI 归 mod_current_share), vref/iref 重解释见 header

#include "mod_supercap.h"
#include "container_of.h"
#include <string.h>

// 控制周期 (秒) — 28.3kHz, 须与 App_OnControlTick 调用频率一致 (与 mod_current_share 同源)
#define MOD_SC_DT (1.0f / 28333.0f)

// 故障去抖确认次数 (短路/失平衡连续 N tick 触发才急停, 防噪声误报)
#define MOD_SC_FAULT_DEBOUNCE_N 8u

// 母线电压下限 — 低于此值视作无功率输出 (i_side 除零保护)
#define MOD_SC_VA_MIN 1.0f

// ======== 内部辅助 ========

static inline float sc_min(float a, float b) {
  return a < b ? a : b;
}

// 采样: AdcDcSampler 工程量 (k/b 校准已在采样器内部完成)
static void sc_sample(ModSuperCap *me) {
  me->va = adc_dc_sampler_get_value(me->adc, me->cfg.adc_ch_va);
  me->vb = adc_dc_sampler_get_value(me->adc, me->cfg.adc_ch_vb);
  uint8_t n = me->cfg.num_phases;
  if (n < 1u) {
    n = 1u;
  }
  if (n > MOD_SC_MAX_PHASES) {
    n = MOD_SC_MAX_PHASES;
  }
  for (uint8_t i = 0; i < n; i++) {
    me->i_phase[i] = adc_dc_sampler_get_value(me->adc, me->cfg.adc_ch_i[i]);
  }
  me->icap = adc_dc_sampler_get_value(me->adc, me->cfg.adc_ch_icap);
  me->ichassis = adc_dc_sampler_get_value(me->adc, me->cfg.adc_ch_ichassis);
}

// 满电渐入: vb 从 charge_resume_v → charge_stop_v 线性衰减充电功率 (28.0→28.6)
static float sc_taper_scale(const ModSuperCap *me) {
  float span = me->cfg.charge_stop_v - me->cfg.charge_resume_v;
  float scale = (span > 0.0f) ? (me->cfg.charge_stop_v - me->vb) / span : 0.0f;
  return math_clamp_f(scale, 0.0f, 1.0f);
}

// 保护: 短路/失平衡去抖确认 → 急停 (返回 true = 已进入 FAULT, 调用方停止级联)
// 健康路径计算功率边界 p_lim_hi/p_lim_lo (满电停充/低压切除/taper 均在此削减)
static bool sc_protect(ModSuperCap *me) {
  // 满电停充 / 低压切除 (迟滞, 非跳闸 — 只削功率边界)
  //   charge_ok state=true = 允许充电 (进 ≤charge_resume_v, 出 ≥charge_stop_v)
  //   discharge_ok state=true = 允许放电 (进 ≥vcut_hi, 出 ≤vcut_lo)
  bool charge_ok = Hysteresis_Update(&me->charge_ok, me->vb);
  bool discharge_ok = Hysteresis_Update(&me->discharge_ok, me->vb);

  // 短路: |底盘电流| > short_ilim 连续 N tick → FAULT 0x08
  bool short_trig = math_abs_f(me->ichassis) > me->cfg.short_ilim;
  if (Debounce_Update(&me->short_deb, short_trig)) {
    power_stage_emergency(&me->base);
    mod_share_emergency(me->share);
    ring_push(&me->evt, MOD_SC_EVT_FAULT_SHORT);
    return true;
  }

  // 失平衡 (N>1): max|i_phase − iavg| > unbalance_thr 连续 N tick → FAULT 0x80
  bool unbal_trig = false;
  uint8_t n = me->cfg.num_phases;
  if (n > 1u) {
    float sum = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
      sum += me->i_phase[i];
    }
    float iavg = sum / (float) n;
    for (uint8_t i = 0; i < n; i++) {
      if (math_abs_f(me->i_phase[i] - iavg) > me->cfg.unbalance_thr) {
        unbal_trig = true;
        break;
      }
    }
  }
  if (Debounce_Update(&me->unbalance_deb, unbal_trig)) {
    power_stage_emergency(&me->base);
    mod_share_emergency(me->share);
    ring_push(&me->evt, MOD_SC_EVT_FAULT_UNBALANCE);
    return true;
  }

  // 健康: 按迟滞状态 + taper 削功率边界
  //   p_lim_hi = min(i_lim_a×va, cap_in_ilimit×vb) × taper  (充电侧, 正)
  //   p_lim_lo = max(-i_lim_a×va, -cap_out_ilimit×vb)        (放电侧, 负)
  float va = me->va > MOD_SC_VA_MIN ? me->va : MOD_SC_VA_MIN;
  float p_in = sc_min(me->cfg.i_lim_a * va, me->cfg.cap_in_ilimit * me->vb) * sc_taper_scale(me);
  float p_out = -sc_min(me->cfg.i_lim_a * va, me->cfg.cap_out_ilimit * me->vb);
  if (!charge_ok) {
    p_in = 0.0f;  // 满电: 停充
  }
  if (!discharge_ok) {
    p_out = 0.0f;  // 低压: 切除放电
  }
  me->p_lim_hi = p_in;
  me->p_lim_lo = p_out;
  return false;
}

// 级联功率环 + 均流驱动 (PST_RUN 内)
static void sc_run(ModSuperCap *me) {
  if (!me->adc || !me->share) {
    me->base.st = PST_FAULT;
    return;
  }

  // 周期边界收取 MAIN 命令 (Command 邮箱): 最新命令生效, 跨上下文无锁安全
  uint32_t c;
  float a;
  if (mailbox_poll(&me->cmd, &c, &a)) {
    if (c == MOD_SC_CMD_SET_REFEREE_POWER) {
      me->referee_power = a;
    }
  }

  // 1. 采样 + 健康度遥测
  sc_sample(me);
  me->cap_health = (me->cfg.charge_stop_v > 0.0f) ? (me->vb / me->cfg.charge_stop_v) : 0.0f;

  // 2. 保护: 故障确认 → 急停并返回 (本周期不再驱动均流/PWM); 健康 → 计算功率边界
  if (sc_protect(me)) {
    return;
  }

  // 3. 功率环级联
  //    p_referee = LPF(va × i_chassis) — 实测母线功率 (正=负载消耗)
  me->p_referee = LowPassFilter_Update(&me->power_lpf, me->va * me->ichassis, MOD_SC_DT);

  //    PI 输出限幅 = 功率边界 (抗积分饱和, 等价于外部 clamp — 不二次限幅)
  me->pid_p_cfg.out_max = me->p_lim_hi;
  me->pid_p_cfg.out_min = me->p_lim_lo;
  me->p_setpoint = pi_reg4_run(&me->pid_p, &me->pid_p_cfg, me->referee_power, me->p_referee);

  //    i_side = p_setpoint / va (va 下限保护除零)
  float va = me->va > MOD_SC_VA_MIN ? me->va : MOD_SC_VA_MIN;
  me->i_side = me->p_setpoint / va;
  me->base.iref = me->referee_power / va;   // 裁判电流参考 (派生, 重解释见 header)
  latch_write(&me->telemetry, me->i_side);  // FAST→SLOW 遥测

  // 4. 注入均流 + 驱动 share tick (写 PWM)
  mod_share_set_voltages(me->share, me->va, me->vb);
  mod_share_set_paside(me->share, me->i_side);
  mod_share_set_currents(me->share, me->i_phase);
  mod_share_tick(me->share);
}

// ======== ops 实现 (PowerStage* 第一参数, container_of 下溯) ========

static void sc_init(PowerStage *base) {
  ModSuperCap *me = container_of(base, ModSuperCap, base);
  LowPassFilter_Init(&me->power_lpf, me->cfg.power_lpf_fc);
  pi_reg4_init(&me->pid_p);
  Debounce_Init(&me->short_deb, MOD_SC_FAULT_DEBOUNCE_N);
  Debounce_Init(&me->unbalance_deb, MOD_SC_FAULT_DEBOUNCE_N);
  // 迟滞派生 (与 cfg 槽位一致): charge_ok 进 28.0 / 出 28.6; discharge_ok 进 19 / 出 18
  Hysteresis_Init(&me->charge_ok, -1e6f, me->cfg.charge_resume_v, -1e6f, me->cfg.charge_stop_v);
  Hysteresis_Init(&me->discharge_ok, me->cfg.vcut_hi, 1e6f, me->cfg.vcut_lo, 1e6f);
  me->va = me->vb = 0.0f;
  me->p_referee = me->p_setpoint = 0.0f;
  me->i_side = 0.0f;
  me->referee_power = 0.0f;
  if (me->share) {
    mod_share_release(me->share);
  }
  me->base.st = PST_INIT;
}

static void sc_tick(PowerStage *base) {
  ModSuperCap *me = container_of(base, ModSuperCap, base);

  switch (me->base.st) {
  case PST_INIT:
    // 自检通过 → 空闲 (真实工程在此检查 PWM/ADC 就绪)
    me->base.st = PST_IDLE;
    break;

  case PST_IDLE:
    break;  // 等待 start

  case PST_RUN:
    sc_run(me);  // 内部可置 PST_FAULT (短路/失平衡确认)
    break;

  case PST_FAULT_HOLD:
  case PST_FAULT:
    break;  // 保持封波, 等待 start 恢复

  case PST_RECOVER:
    // 恢复: 重新进入 RUN (软启由功率环 LPF/PI 自然收敛)
    me->base.st = PST_RUN;
    break;

  default:
    break;
  }
}

static void sc_start(PowerStage *base) {
  ModSuperCap *me = container_of(base, ModSuperCap, base);
  // FAULT → RUN 手动再布防: 不检查故障是否仍存在 — 持续性故障由去抖 (8 tick) 挡住,
  // 恢复后若故障仍在, 重新确认即再次跳闸 (0x08/0x40), 不会在故障下维持运行
  if (me->base.st == PST_IDLE || me->base.st == PST_FAULT || me->base.st == PST_RECOVER) {
    me->base.debounce_cnt = 0;
    Debounce_Reset(&me->short_deb);
    Debounce_Reset(&me->unbalance_deb);
    LowPassFilter_Init(&me->power_lpf, me->cfg.power_lpf_fc);
    pi_reg4_reset(&me->pid_p);
    me->referee_power = 0.0f;
    if (me->share) {
      mod_share_release(me->share);
    }
    me->base.st = PST_RUN;
    if (me->base.pwm) {
      pwm_start(me->base.pwm);
    }
  }
}

static void sc_stop(PowerStage *base) {
  ModSuperCap *me = container_of(base, ModSuperCap, base);
  if (me->base.pwm) {
    pwm_stop(me->base.pwm);
  }
  me->base.st = PST_IDLE;
  // START/STOP 状态变化由调用方读 mod_supercap_state 观察, 不推事件环 (环单生产者=FAST)
}

static void sc_emergency(PowerStage *base) {
  power_stage_emergency(base);  // 默认辅助: FAULT + pwm_emergency_stop
  ModSuperCap *me = container_of(base, ModSuperCap, base);
  if (me->share) {
    mod_share_emergency(me->share);  // 同步清均流积分 + 停写
  }
}

static void sc_set_ref(PowerStage *base, float vref, float iref) {
  // vref = 充电目标电压, iref = 裁判功率目标 (W) — 重解释, 见 header
  ModSuperCap *me = container_of(base, ModSuperCap, base);
  me->cfg.charge_stop_v = vref;
  me->referee_power = iref;
  mod_supercap_sync_cfg(me);
}

static void sc_apply_tune(PowerStage *base, const float coef[10]) {
  ModSuperCap *me = container_of(base, ModSuperCap, base);
  me->cfg.pid_p_kp = coef[0];
  me->cfg.pid_p_ki = coef[1];
  me->cfg.charge_stop_v = coef[2];
  me->cfg.charge_resume_v = coef[3];
  me->cfg.vcut_lo = coef[4];
  me->cfg.vcut_hi = coef[5];
  me->cfg.i_lim_a = coef[6];
  me->cfg.cap_in_ilimit = coef[7];
  me->cfg.cap_out_ilimit = coef[8];
  me->cfg.share_gain = coef[9];
  mod_supercap_sync_cfg(me);
}

static PstSt sc_state(PowerStage *base) {
  return base->st;
}

// ======== 虚表 ========
static const PowerStageOps sc_ops = {
    .init = sc_init,
    .tick = sc_tick,
    .start = sc_start,
    .stop = sc_stop,
    .emergency = sc_emergency,
    .set_ref = sc_set_ref,
    .apply_tune = sc_apply_tune,
    .state = sc_state,
};

// ======== 构造 ========

void mod_supercap_init(ModSuperCap *me, const ModSuperCapCfg *cfg) {
  memset(me, 0, sizeof(*me));

  // 五原语交接点初始化
  ring_init(&me->evt, me->evt_buf, sizeof(me->evt_buf));
  latch_init(&me->telemetry);
  mailbox_init(&me->cmd);

  me->base.ops = &sc_ops;

  if (cfg) {
    me->cfg = *cfg;
  }

  // 非槽位默认
  if (me->cfg.power_lpf_fc <= 0.0f) {
    me->cfg.power_lpf_fc = 120.0f;
  }
  if (me->cfg.num_phases == 0u) {
    me->cfg.num_phases = 1u;
  }
  if (me->cfg.num_phases > MOD_SC_MAX_PHASES) {
    me->cfg.num_phases = MOD_SC_MAX_PHASES;
  }
  if (me->cfg.short_ilim <= 0.0f) {
    me->cfg.short_ilim = 2.0f * me->cfg.i_lim_a;
  }
  if (me->cfg.short_ilim <= 0.0f) {
    me->cfg.short_ilim = 50.0f;  // i_lim_a 未配置时的兜底
  }
  if (me->cfg.unbalance_thr <= 0.0f) {
    me->cfg.unbalance_thr = 0.5f * me->cfg.i_lim_a;
  }

  // 控制元件初始化 (与 sc_init 同款 — 状态机不调用 ops->init, 构造时即完成)
  LowPassFilter_Init(&me->power_lpf, me->cfg.power_lpf_fc);
  pi_reg4_init(&me->pid_p);
  Debounce_Init(&me->short_deb, MOD_SC_FAULT_DEBOUNCE_N);
  Debounce_Init(&me->unbalance_deb, MOD_SC_FAULT_DEBOUNCE_N);

  me->base.st = PST_INIT;
  mod_supercap_sync_cfg(me);
}

// ======== 配置同步 ========

void mod_supercap_sync_cfg(ModSuperCap *me) {
  me->base.vref = me->cfg.charge_stop_v;  // 重解释: 充电目标电压

  // 功率环 PI (输出限幅在 sc_run 内逐周期设为功率边界, 初始给宽限)
  me->pid_p_cfg.kp = me->cfg.pid_p_kp;
  me->pid_p_cfg.ki = me->cfg.pid_p_ki;
  me->pid_p_cfg.kff = 0.0f;
  me->pid_p_cfg.dt = MOD_SC_DT;
  me->pid_p_cfg.out_max = 1e6f;
  me->pid_p_cfg.out_min = -1e6f;
  me->pid_p_cfg.sp_fc = 0.0f;

  // 保护迟滞派生
  Hysteresis_Init(&me->charge_ok, -1e6f, me->cfg.charge_resume_v, -1e6f, me->cfg.charge_stop_v);
  Hysteresis_Init(&me->discharge_ok, me->cfg.vcut_hi, 1e6f, me->cfg.vcut_lo, 1e6f);

  // 均流增益 + 相数注入 (share 可能未绑定 — 绑定后由 mod_supercap_bind 再同步)
  if (me->share) {
    me->share->cfg.share_gain = me->cfg.share_gain;
    me->share->cfg.num_phases = me->cfg.num_phases;
  }
}

void mod_supercap_bind(ModSuperCap *me, AdcDcSampler *adc, ModCurrentShare *share) {
  // 前提 (头注释声明): 必须先 mod_share_bind(share, pwm) 再调本函数 —
  //   base.pwm 从这里快照, 错序则故障急停时 pwm_emergency_stop 失效
  me->adc = adc;
  me->share = share;
  me->base.adc = adc ? &adc->base : NULL;
  me->base.pwm = (share && share->pwm) ? &share->pwm->base : NULL;  // 功率环急停共用 PWM
  mod_supercap_sync_cfg(me);                                        // 把 num_phases/share_gain 推入 share
}

// ======== 公开 API (包装 ops) ========

void mod_supercap_tick(ModSuperCap *me) {
  if (me->base.ops && me->base.ops->tick) {
    me->base.ops->tick(&me->base);
  }
}

void mod_supercap_start(ModSuperCap *me) {
  if (me->base.ops && me->base.ops->start) {
    me->base.ops->start(&me->base);
  }
}

void mod_supercap_stop(ModSuperCap *me) {
  if (me->base.ops && me->base.ops->stop) {
    me->base.ops->stop(&me->base);
  }
}

void mod_supercap_emergency(ModSuperCap *me) {
  if (me->base.ops && me->base.ops->emergency) {
    me->base.ops->emergency(&me->base);
  }
}

void mod_supercap_apply_tune(ModSuperCap *me, const float coef[10]) {
  if (me->base.ops && me->base.ops->apply_tune) {
    me->base.ops->apply_tune(&me->base, coef);
  }
}

PstSt mod_supercap_state(const ModSuperCap *me) {
  return me->base.st;
}
