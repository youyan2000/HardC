// 超级电容功率控制模块 — PowerStage 派生 (实现)
//
// 每 FAST tick (所有状态):
//   采样(va/vb/i_phase/icap/ichassis) → 保护(短路/失平衡去抖→急停; 满电/低压迟滞削边界;
//   与 buck A6 对齐: 所有状态每周期检查 — IDLE/FAULT 下外部母线短路/意外输出仍被检出)
//   PST_RUN 内: 级联功率环(LPF→PI→限幅→taper→i_side) → 注入均流 + 驱动 share tick → PWM
//
// 状态机: INIT → IDLE → RUN ⇄ FAULT (start 恢复重新软启)
//   级联绕过 base.loop[2] (per-phase PI 归 gen_current_share), vref/iref 重解释见 header

#include "gen_supercap.h"
#include "container_of.h"
#include <string.h>

// 控制周期由 cfg.control_freq_hz 派生 (见 gen_sc_dt) — 不再硬编码 28333

// 故障去抖确认次数 (短路/失平衡连续 N tick 触发才急停, 防噪声误报)
#define GEN_SC_FAULT_DEBOUNCE_N 8u

// 母线电压下限 — 低于此值视作无功率输出 (i_side 除零保护)
#define GEN_SC_VA_MIN 1.0f

// ======== 内部辅助 ========

// 控制周期 (秒) — 由 cfg.control_freq_hz 派生; 缺省 28333Hz (须与 App_OnControlTick 实际频率一致)
static float gen_sc_dt(const GenSuperCap *me) {
  float f = me->cfg.control_freq_hz;
  if (f <= 0.0f) {
    f = 28333.0f;
  }
  return 1.0f / f;
}

static inline float sc_min(float a, float b) {
  return a < b ? a : b;
}

// 采样: AdcDcSampler 工程量 (k/b 校准已在采样器内部完成)
static void sc_sample(GenSuperCap *me) {
  me->va = adc_dc_sampler_get_value(me->adc, me->cfg.adc_ch_va);
  me->vb = adc_dc_sampler_get_value(me->adc, me->cfg.adc_ch_vb);
  uint8_t n = me->cfg.num_phases;
  if (n < 1u) {
    n = 1u;
  }
  if (n > GEN_SC_MAX_PHASES) {
    n = GEN_SC_MAX_PHASES;
  }
  for (uint8_t i = 0; i < n; i++) {
    me->i_phase[i] = adc_dc_sampler_get_value(me->adc, me->cfg.adc_ch_i[i]);
  }
  me->icap = adc_dc_sampler_get_value(me->adc, me->cfg.adc_ch_icap);
  me->ichassis = adc_dc_sampler_get_value(me->adc, me->cfg.adc_ch_ichassis);
}

// 满电渐入: vb 从 charge_resume_v → charge_stop_v 线性衰减充电功率 (28.0→28.6)
static float sc_taper_scale(const GenSuperCap *me) {
  float span = me->cfg.charge_stop_v - me->cfg.charge_resume_v;
  float scale = (span > 0.0f) ? (me->cfg.charge_stop_v - me->vb) / span : 0.0f;
  return math_clamp_f(scale, 0.0f, 1.0f);
}

// 保护: 短路/失平衡去抖确认 → 急停 (返回 true = 已进入 FAULT, 调用方停止级联)
// 健康路径计算功率边界 p_lim_hi/p_lim_lo (满电停充/低压切除/taper 均在此削减)
static bool sc_protect(GenSuperCap *me) {
  // 满电停充 / 低压切除 (迟滞, 非跳闸 — 只削功率边界)
  //   charge_ok state=true = 允许充电 (进 ≤charge_resume_v, 出 ≥charge_stop_v)
  //   discharge_ok state=true = 允许放电 (进 ≥vcut_hi, 出 ≤vcut_lo)
  bool charge_ok = Hysteresis_Update(&me->charge_ok, me->vb);
  bool discharge_ok = Hysteresis_Update(&me->discharge_ok, me->vb);

  // 短路: |底盘电流| > short_ilim 连续 N tick → FAULT 0x08
  bool short_trig = math_abs_f(me->ichassis) > me->cfg.short_ilim;
  if (Debounce_Update(&me->short_deb, short_trig)) {
    power_stage_emergency(&me->base);
    gen_share_emergency(me->share);
    // 事件流 (SPSC 环 fire-and-forget = IO_NONE): 满则计数丢弃, 不阻塞 FAST (A5)
    if (!ring_push(&me->evt, GEN_SC_EVT_FAULT_SHORT)) {
      me->evt_overflow_cnt++;
    }
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
    gen_share_emergency(me->share);
    // 事件流 (SPSC 环 fire-and-forget = IO_NONE): 满则计数丢弃, 不阻塞 FAST (A5)
    if (!ring_push(&me->evt, GEN_SC_EVT_FAULT_UNBALANCE)) {
      me->evt_overflow_cnt++;
    }
    return true;
  }

  // 健康: 按迟滞状态 + taper 削功率边界
  //   p_lim_hi = min(i_lim_a×va, cap_in_ilimit×vb) × taper  (充电侧, 正)
  //   p_lim_lo = max(-i_lim_a×va, -cap_out_ilimit×vb)        (放电侧, 负)
  float va = me->va > GEN_SC_VA_MIN ? me->va : GEN_SC_VA_MIN;
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

// 级联功率环 + 均流驱动 (PST_RUN 内; 采样/保护已由 sc_tick 全状态执行, A6 对齐)
static void sc_run(GenSuperCap *me) {
  if (!me->adc || !me->share) {
    me->base.st = PST_FAULT;
    return;
  }

  // 周期边界收取 MAIN 命令 (Command 邮箱): 最新命令生效, 跨上下文无锁安全
  uint32_t c;
  float a;
  if (mailbox_poll(&me->cmd, &c, &a)) {
    if (c == GEN_SC_CMD_SET_REFEREE_POWER) {
      me->referee_power = a;
    }
  }

  // 1. 功率环级联
  //    p_referee = LPF(va × i_chassis) — 实测母线功率 (正=负载消耗)
  me->p_referee = LowPassFilter_Update(&me->power_lpf, me->va * me->ichassis, gen_sc_dt(me));

  //    PI 输出限幅 = 功率边界 (PidReg4 base 限幅, 抗积分饱和, 等价于外部 clamp — 不二次限幅)
  me->pid_p.base.out_max = me->p_lim_hi;
  me->pid_p.base.out_min = me->p_lim_lo;
  me->p_setpoint = pid_compute(&me->pid_p.base, me->referee_power, me->p_referee);

  //    i_side = p_setpoint / va (va 下限保护除零)
  float va = me->va > GEN_SC_VA_MIN ? me->va : GEN_SC_VA_MIN;
  me->i_side = me->p_setpoint / va;
  me->base.iref = me->referee_power / va;   // 裁判电流参考 (派生, 重解释见 header)
  latch_write(&me->telemetry, me->i_side);  // FAST→SLOW 遥测

  // 2. 注入均流 + 驱动 share tick (写 PWM)
  gen_share_set_voltages(me->share, me->va, me->vb);
  gen_share_set_paside(me->share, me->i_side);
  gen_share_set_currents(me->share, me->i_phase);
  gen_share_tick(me->share);
}

// ======== ops 实现 (PowerStage* 第一参数, container_of 下溯) ========

static void sc_init(PowerStage *base) {
  GenSuperCap *me = container_of(base, GenSuperCap, base);
  LowPassFilter_Init(&me->power_lpf, me->cfg.power_lpf_fc);
  pid_reset(&me->pid_p.base);  // 清积分器, 保留已同步配置 (sync_cfg 派生)
  Debounce_Init(&me->short_deb, GEN_SC_FAULT_DEBOUNCE_N);
  Debounce_Init(&me->unbalance_deb, GEN_SC_FAULT_DEBOUNCE_N);
  // 迟滞派生 (与 cfg 槽位一致): charge_ok 进 28.0 / 出 28.6; discharge_ok 进 19 / 出 18
  Hysteresis_Init(&me->charge_ok, -1e6f, me->cfg.charge_resume_v, -1e6f, me->cfg.charge_stop_v);
  Hysteresis_Init(&me->discharge_ok, me->cfg.vcut_hi, 1e6f, me->cfg.vcut_lo, 1e6f);
  me->va = me->vb = 0.0f;
  me->p_referee = me->p_setpoint = 0.0f;
  me->i_side = 0.0f;
  me->referee_power = 0.0f;
  if (me->share) {
    gen_share_release(me->share);
  }
  me->base.st = PST_INIT;
}

static void sc_tick(PowerStage *base) {
  GenSuperCap *me = container_of(base, GenSuperCap, base);

  // 采样 + 保护: 所有状态每周期执行 (A6 对齐, 同 buck) — IDLE/FAULT 下外部母线短路/意外输出仍被检出
  // 设备未绑定 (adc/share) 则无从采样 (board_init 绑定; 未绑定保持 0 值不误报)
  // 注: 短路/失平衡 Debounce 内置上升沿确认 (confirmed 标志), 持续故障不会重复急停/推事件 — 无需额外锁
  if (me->adc != NULL && me->share != NULL) {
    sc_sample(me);
    me->cap_health = (me->cfg.charge_stop_v > 0.0f) ? (me->vb / me->cfg.charge_stop_v) : 0.0f;
    if (sc_protect(me)) {
      return;  // 故障已确认 (FAULT): 本周期不推进状态机输出
    }
  }

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
  GenSuperCap *me = container_of(base, GenSuperCap, base);
  // FAULT → RUN 手动再布防: 不检查故障是否仍存在 — 持续性故障由去抖 (8 tick) 挡住,
  // 恢复后若故障仍在, 重新确认即再次跳闸 (0x08/0x40), 不会在故障下维持运行
  if (me->base.st == PST_IDLE || me->base.st == PST_FAULT || me->base.st == PST_RECOVER) {
    me->base.debounce_cnt = 0;
    Debounce_Reset(&me->short_deb);
    Debounce_Reset(&me->unbalance_deb);
    LowPassFilter_Init(&me->power_lpf, me->cfg.power_lpf_fc);
    pid_reset(&me->pid_p.base);
    me->referee_power = 0.0f;
    if (me->share) {
      gen_share_release(me->share);
    }
    me->base.st = PST_RUN;
    if (me->base.pwm) {
      pwm_start(me->base.pwm);
    }
  }
}

static void sc_stop(PowerStage *base) {
  GenSuperCap *me = container_of(base, GenSuperCap, base);
  if (me->base.pwm) {
    pwm_stop(me->base.pwm);
  }
  me->base.st = PST_IDLE;
  // START/STOP 状态变化由调用方读 gen_supercap_state 观察, 不推事件环 (环单生产者=FAST)
}

static void sc_emergency(PowerStage *base) {
  power_stage_emergency(base);  // 默认辅助: FAULT + pwm_emergency_stop
  GenSuperCap *me = container_of(base, GenSuperCap, base);
  if (me->share) {
    gen_share_emergency(me->share);  // 同步清均流积分 + 停写
  }
}

static void sc_set_ref(PowerStage *base, float vref, float iref) {
  // vref = 充电目标电压, iref = 裁判功率目标 (W) — 重解释, 见 header
  GenSuperCap *me = container_of(base, GenSuperCap, base);
  me->cfg.charge_stop_v = vref;
  me->referee_power = iref;
  gen_supercap_sync_cfg(me);
}

static void sc_apply_tune(PowerStage *base, const float coef[10]) {
  GenSuperCap *me = container_of(base, GenSuperCap, base);
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
  gen_supercap_sync_cfg(me);
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

void gen_supercap_init(GenSuperCap *me, const GenSuperCapCfg *cfg) {
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
  if (me->cfg.control_freq_hz <= 0.0f) {
    me->cfg.control_freq_hz = 28333.0f;  // 缺省 28.3kHz
  }
  if (me->cfg.num_phases == 0u) {
    me->cfg.num_phases = 1u;
  }
  if (me->cfg.num_phases > GEN_SC_MAX_PHASES) {
    me->cfg.num_phases = GEN_SC_MAX_PHASES;
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
  // 功率环 PI (PidReg4 = TI pi_reg4, 复刻原 PidLinear aw=CLAMP); 增益/限幅由下方 sync_cfg 派生
  PidReg4Cfg r4 = {0.0f, 0.0f, 0.0f, 0.0f};                    // kp, ki, kff, sp_fc (sync_cfg 填)
  pid_reg4_init(&me->pid_p, gen_sc_dt(me), -1e6f, 1e6f, &r4);  // 初值宽限, sync_cfg 收窄
  Debounce_Init(&me->short_deb, GEN_SC_FAULT_DEBOUNCE_N);
  Debounce_Init(&me->unbalance_deb, GEN_SC_FAULT_DEBOUNCE_N);

  me->base.st = PST_INIT;
  gen_supercap_sync_cfg(me);
}

// ======== 配置同步 ========

void gen_supercap_sync_cfg(GenSuperCap *me) {
  me->base.vref = me->cfg.charge_stop_v;  // 重解释: 充电目标电压

  // 功率环 PI (PidReg4, TI pi_reg4 语义, 复刻原 PidLinear aw=CLAMP):
  //   输出限幅在 sc_run 内逐周期设为功率边界, 初始给宽限
  me->pid_p.cfg.kp = me->cfg.pid_p_kp;
  me->pid_p.cfg.ki = me->cfg.pid_p_ki;
  me->pid_p.cfg.kff = 0.0f;
  me->pid_p.cfg.sp_fc = 0.0f;
  me->pid_p.base.dt = gen_sc_dt(me);
  me->pid_p.base.out_max = 1e6f;
  me->pid_p.base.out_min = -1e6f;

  // 保护迟滞派生
  Hysteresis_Init(&me->charge_ok, -1e6f, me->cfg.charge_resume_v, -1e6f, me->cfg.charge_stop_v);
  Hysteresis_Init(&me->discharge_ok, me->cfg.vcut_hi, 1e6f, me->cfg.vcut_lo, 1e6f);

  // 均流增益 + 相数注入 (share 可能未绑定 — 绑定后由 gen_supercap_bind 再同步)
  if (me->share) {
    me->share->cfg.share_gain = me->cfg.share_gain;
    me->share->cfg.num_phases = me->cfg.num_phases;
  }
}

void gen_supercap_bind(GenSuperCap *me, AdcDcSampler *adc, GenCurrentShare *share) {
  // 前提 (头注释声明): 必须先 gen_share_bind(share, pwm) 再调本函数 —
  //   base.pwm 从这里快照, 错序则故障急停时 pwm_emergency_stop 失效
  me->adc = adc;
  me->share = share;
  me->base.adc = adc ? &adc->base : NULL;
  me->base.pwm = (share && share->pwm) ? &share->pwm->base : NULL;  // 功率环急停共用 PWM
  gen_supercap_sync_cfg(me);                                        // 把 num_phases/share_gain 推入 share
}

// ======== 公开 API (包装 ops) ========

void gen_supercap_tick(GenSuperCap *me) {
  if (me->base.ops && me->base.ops->tick) {
    me->base.ops->tick(&me->base);
  }
}

void gen_supercap_start(GenSuperCap *me) {
  if (me->base.ops && me->base.ops->start) {
    me->base.ops->start(&me->base);
  }
}

void gen_supercap_stop(GenSuperCap *me) {
  if (me->base.ops && me->base.ops->stop) {
    me->base.ops->stop(&me->base);
  }
}

void gen_supercap_emergency(GenSuperCap *me) {
  if (me->base.ops && me->base.ops->emergency) {
    me->base.ops->emergency(&me->base);
  }
}

void gen_supercap_apply_tune(GenSuperCap *me, const float coef[10]) {
  if (me->base.ops && me->base.ops->apply_tune) {
    me->base.ops->apply_tune(&me->base, coef);
  }
}

PstSt gen_supercap_state(const GenSuperCap *me) {
  return me->base.st;
}
