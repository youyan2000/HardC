// 四开关 Buck/Boost PWM —— PwmBase 子类 (相位参数化实现)
//
// 占空比律在 Device 内部实现 (host 可测), 模块只写电压比/模式/相位调制指数:
//   ratio = vb/va → 律给出两腿占空比 (duty_a/duty_b), 钳位 [ratio_lo, ratio_hi]
//   alpha (每相)  → 实际占空比 = 律值 × alpha, 上限 duty_max
// BSP 层只接收物理量 (CMP 计数值), 占空比 → CMP 换算在本层.
//
// 相位分布: 各相相位偏移 = 360°/N × i, init 自动计算, start 时经 bsp_set_phase_shift
//   写入两腿定时器. 真交错 (HRTIM CMP2 主复位连线) 是 BSP/工具链 AI 缺口
//   (见 AGENT-SYNC); 本层存相位并调 API, 对齐即正确.
//
// 接线契约: init 阶段不写硬件 (bsp_cfg.handle 为 NULL). App 注入 handle + clk_hz
//   后调 pwm_bb_set_freq → set_mode/ratio/alpha → start. 四开关走双 CMP 独立占空比,
//   绕过互补+死区路径 — duty_max 0.97 已留死区裕量 (头注释声明).

#include "pwm_buckboost.h"
#include "container_of.h"
#include <stddef.h>

// ======== 内部辅助 ========

static inline float clamp_f(float d, float lo, float hi) {
  if (d < lo) return lo;
  if (d > hi) return hi;
  return d;
}

// 单腿占空比 → CMP1/CMP3 (中心对齐: 对称双沿; 边沿对齐: 单沿)
static void bb_leg_cmp(PwmBuckBoost *me, float duty, uint32_t *cmp1, uint32_t *cmp3) {
  if (me->center_aligned) {
    uint32_t half = me->period / 2;
    *cmp1 = (uint32_t)(half * (1.0f - duty));
    *cmp3 = (uint32_t)(half * (1.0f + duty));
  } else {
    *cmp1 = (uint32_t)(me->period * duty);
    *cmp3 = 0u;
  }
}

// 按当前 mode/ratio 重算某相两腿占空比 (律值 × alpha, 限幅 duty_max)
static void bb_recompute_phase(PwmBuckBoost *me, PwmBbPhase *ph) {
  float ratio = me->ratio;
  float da, db;
  switch (me->mode) {
    case PwmMode_Buck:
      da = me->duty_base * ratio;
      db = me->duty_base;
      break;
    case PwmMode_Boost:
      da = me->duty_base;
      db = me->duty_base / ratio;   // ratio ≥ ratio_lo > 0, 无除零
      break;
    case PwmMode_BuckBoost:
    default:
      da = me->buckboost_gain * (ratio + 1.0f);
      db = me->buckboost_gain * (1.0f / ratio + 1.0f);
      break;
  }
  ph->duty_a = clamp_f(da * ph->alpha, 0.0f, me->duty_max);
  ph->duty_b = clamp_f(db * ph->alpha, 0.0f, me->duty_max);
}

// 写某相两腿 CMP (热路径)
static void bb_write_phase(PwmBuckBoost *me, PwmBbPhase *ph) {
  uint32_t c1, c3;
  bb_leg_cmp(me, ph->duty_a, &c1, &c3);
  bsp_update_duty(me->bsp_cfg.handle, ph->timer_a, c1, c3);
  if (ph->leg_b_used) {
    bb_leg_cmp(me, ph->duty_b, &c1, &c3);
    bsp_update_duty(me->bsp_cfg.handle, ph->timer_b, c1, c3);
  }
}

// 全部相重算 + 写 (模式/电压比/频率变化后)
static void bb_recompute_all(PwmBuckBoost *me) {
  for (uint8_t i = 0; i < me->num_phases; i++) {
    bb_recompute_phase(me, &me->phases[i]);
    bb_write_phase(me, &me->phases[i]);
  }
}

// 配置单腿定时器 (start 时)
static void bb_config_leg(PwmBuckBoost *me, BspPwmTimer timer, uint32_t mask,
                          float duty, BspPwmTimerConfig *tcfg) {
  uint32_t c1, c3;
  bb_leg_cmp(me, duty, &c1, &c3);
  tcfg->timer         = timer;
  tcfg->period        = me->period;
  tcfg->cmp1          = c1;
  tcfg->cmp2          = me->center_aligned ? me->period / 2 : 0u;
  tcfg->cmp3          = c3;
  tcfg->output_mask   = mask;
  tcfg->complementary = me->sync_rect;
  tcfg->deadtime_rising  = me->sync_rect ? me->deadtime_ns : 0u;
  tcfg->deadtime_falling = me->sync_rect ? me->deadtime_ns : 0u;
}

// ======== ops 实现 ========

static void bb_start(PwmBase *base) {
  PwmBuckBoost *me = container_of(base, PwmBuckBoost, base);
  uint32_t timer_mask = 0u, output_mask = 0u;

  for (uint8_t i = 0; i < me->num_phases; i++) {
    PwmBbPhase *ph = &me->phases[i];
    BspPwmTimerConfig tcfg;

    bb_config_leg(me, ph->timer_a, ph->mask_a, ph->duty_a, &tcfg);
    bsp_config_timer(me->bsp_cfg.handle, &tcfg);
    timer_mask  |= (1u << ph->timer_a);
    output_mask |= ph->mask_a;

    if (ph->leg_b_used) {
      bb_config_leg(me, ph->timer_b, ph->mask_b, ph->duty_b, &tcfg);
      bsp_config_timer(me->bsp_cfg.handle, &tcfg);
      timer_mask  |= (1u << ph->timer_b);
      output_mask |= ph->mask_b;
    }

    // 交错并联 (N>1): 各相 360°/N 均匀错相; 单相无偏移
    if (ph->phase_deg > 0.0f) {
      bsp_set_phase_shift(me->bsp_cfg.handle, ph->timer_a, ph->phase_deg);
      if (ph->leg_b_used) {
        bsp_set_phase_shift(me->bsp_cfg.handle, ph->timer_b, ph->phase_deg);
      }
    }
  }

  bsp_start(me->bsp_cfg.handle, timer_mask, output_mask);
}

static void bb_stop(PwmBase *base) {
  PwmBuckBoost *me = container_of(base, PwmBuckBoost, base);
  uint32_t timer_mask = 0u, output_mask = 0u;
  for (uint8_t i = 0; i < me->num_phases; i++) {
    PwmBbPhase *ph = &me->phases[i];
    timer_mask  |= (1u << ph->timer_a);
    output_mask |= ph->mask_a;
    if (ph->leg_b_used) {
      timer_mask  |= (1u << ph->timer_b);
      output_mask |= ph->mask_b;
    }
  }
  bsp_stop(me->bsp_cfg.handle, timer_mask, output_mask);
}

// 设置某相 A 腿绝对占空比 (兼容单相 PwmBase 路径): ch = 相位索引
static void bb_set_duty(PwmBase *base, uint8_t ch, float duty) {
  PwmBuckBoost *me = container_of(base, PwmBuckBoost, base);
  if (ch >= me->num_phases) return;
  PwmBbPhase *ph = &me->phases[ch];
  ph->duty_a = clamp_f(duty, 0.0f, me->duty_max);
  uint32_t c1, c3;
  bb_leg_cmp(me, ph->duty_a, &c1, &c3);
  bsp_update_duty(me->bsp_cfg.handle, ph->timer_a, c1, c3);
}

static void bb_set_freq(PwmBase *base, uint32_t freq_hz) {
  PwmBuckBoost *me = container_of(base, PwmBuckBoost, base);
  if (freq_hz == 0u) return;
  me->period = me->bsp_cfg.clk_hz / freq_hz;
  for (uint8_t i = 0; i < me->num_phases; i++) {
    bsp_update_period(me->bsp_cfg.handle, me->phases[i].timer_a, me->period);
    if (me->phases[i].leg_b_used) {
      bsp_update_period(me->bsp_cfg.handle, me->phases[i].timer_b, me->period);
    }
  }
  bb_recompute_all(me);   // 保持占空比重映射
}

static void bb_set_deadtime(PwmBase *base, uint32_t deadtime_ns) {
  PwmBuckBoost *me = container_of(base, PwmBuckBoost, base);
  me->deadtime_ns = deadtime_ns;
  if (!me->sync_rect) return;
  for (uint8_t i = 0; i < me->num_phases; i++) {
    bsp_update_deadtime(me->bsp_cfg.handle, me->phases[i].timer_a,
                            deadtime_ns, deadtime_ns);
    if (me->phases[i].leg_b_used) {
      bsp_update_deadtime(me->bsp_cfg.handle, me->phases[i].timer_b,
                              deadtime_ns, deadtime_ns);
    }
  }
}

// 设置某相相位偏移: ch = 相位索引, 两腿同步移相
static void bb_set_phase(PwmBase *base, uint8_t ch, float phase_deg) {
  PwmBuckBoost *me = container_of(base, PwmBuckBoost, base);
  if (ch >= me->num_phases) return;
  PwmBbPhase *ph = &me->phases[ch];
  ph->phase_deg = phase_deg;
  bsp_set_phase_shift(me->bsp_cfg.handle, ph->timer_a, phase_deg);
  if (ph->leg_b_used) {
    bsp_set_phase_shift(me->bsp_cfg.handle, ph->timer_b, phase_deg);
  }
}

static void bb_emergency_stop(PwmBase *base) {
  PwmBuckBoost *me = container_of(base, PwmBuckBoost, base);
  uint32_t output_mask = 0u;
  for (uint8_t i = 0; i < me->num_phases; i++) {
    PwmBbPhase *ph = &me->phases[i];
    output_mask |= ph->mask_a;
    if (ph->leg_b_used) {
      output_mask |= ph->mask_b;
    }
  }
  bsp_emergency_stop(me->bsp_cfg.handle, output_mask);
}

// ======== 虚表 ========
static const PwmOps bb_ops = {
  .start          = bb_start,
  .stop           = bb_stop,
  .set_duty       = bb_set_duty,
  .set_freq       = bb_set_freq,
  .set_deadtime   = bb_set_deadtime,
  .set_phase      = bb_set_phase,
  .emergency_stop = bb_emergency_stop,
};

// ======== 构造 ========

void pwm_bb_init_phases(PwmBuckBoost *me, uint32_t freq_hz,
                        uint8_t num_phases, const PwmBbPhaseCfg *cfg,
                        PwmMode mode, bool sync_rect) {
  pwm_base_init(&me->base);

  me->bsp_cfg.handle  = NULL;
  me->bsp_cfg.clk_hz  = 0u;
  me->bsp_cfg.use_dll = false;

  if (num_phases == 0u) num_phases = 1u;
  if (num_phases > PWM_BB_MAX_PHASES) num_phases = PWM_BB_MAX_PHASES;
  me->num_phases = num_phases;

  me->ratio        = 1.0f;
  me->mode         = mode;
  me->sync_rect    = sync_rect;
  me->center_aligned = true;
  me->deadtime_ns  = 0u;
  me->period       = 0u;

  me->duty_base      = 0.96f;
  me->buckboost_gain = 0.33f;
  me->ratio_lo       = 0.73f;
  me->ratio_hi       = 1.37f;
  me->duty_max       = 0.97f;

  for (uint8_t i = 0; i < num_phases; i++) {
    PwmBbPhase *ph = &me->phases[i];
    ph->timer_a    = cfg[i].timer_a;
    ph->mask_a     = cfg[i].mask_a;
    ph->timer_b    = cfg[i].timer_b;
    ph->mask_b     = cfg[i].mask_b;
    ph->leg_b_used = cfg[i].leg_b_used;
    ph->duty_a     = 0.0f;
    ph->duty_b     = 0.0f;
    ph->alpha      = 1.0f;
    ph->phase_deg  = (360.0f / num_phases) * i;   // 均匀错相 (N=1 → 0°)
  }

  me->base.mode     = mode;
  me->base.num_ch   = num_phases;
  me->base.duty_min = 0.0f;
  me->base.duty_max = me->duty_max;
  me->base.ops      = &bb_ops;
  me->base.freq_hz  = freq_hz;

  // 初始占空比律 (纯计算, 不写硬件 — handle 由 App 注入, init 阶段为 NULL)
  for (uint8_t i = 0; i < num_phases; i++) {
    bb_recompute_phase(me, &me->phases[i]);
  }
}

void pwm_bb_init(PwmBuckBoost *me, uint32_t freq_hz,
                 BspPwmTimer timer, uint32_t output_mask,
                 PwmMode mode, bool sync_rect) {
  PwmBbPhaseCfg cfg = {
    .timer_a    = timer,
    .mask_a     = output_mask,
    .timer_b    = timer,      // 单腿, B 腿不启用
    .mask_b     = 0u,
    .leg_b_used = false,
  };
  pwm_bb_init_phases(me, freq_hz, 1u, &cfg, mode, sync_rect);
}

void pwm_bb_deinit(PwmBuckBoost *me) {
  if (me->base.running) {
    bb_stop(&me->base);
  }
  me->bsp_cfg.handle = NULL;
  me->num_phases = 0u;
  pwm_base_init(&me->base);
}

// ======== 运行时调参 ========

void pwm_bb_set_ratio(PwmBuckBoost *me, float ratio) {
  me->ratio = clamp_f(ratio, me->ratio_lo, me->ratio_hi);
  bb_recompute_all(me);
}

void pwm_bb_set_mode(PwmBuckBoost *me, PwmMode mode) {
  me->mode = mode;
  me->base.mode = mode;
  bb_recompute_all(me);
}

void pwm_bb_set_alpha(PwmBuckBoost *me, uint8_t phase, float alpha) {
  if (phase >= me->num_phases) return;
  PwmBbPhase *ph = &me->phases[phase];
  ph->alpha = alpha;
  bb_recompute_phase(me, ph);
  bb_write_phase(me, ph);
}

void pwm_bb_set_law_constants(PwmBuckBoost *me, float duty_base, float bb_gain,
                              float ratio_lo, float ratio_hi, float duty_max) {
  me->duty_base      = duty_base;
  me->buckboost_gain = bb_gain;
  me->ratio_lo       = ratio_lo;
  me->ratio_hi       = ratio_hi;
  me->duty_max       = duty_max;
  me->base.duty_max  = duty_max;
  bb_recompute_all(me);
}

void pwm_bb_set_duty(PwmBuckBoost *me, float duty) {
  pwm_set_duty(&me->base, 0, duty);
}

void pwm_bb_set_freq(PwmBuckBoost *me, uint32_t freq_hz) {
  pwm_set_freq(&me->base, freq_hz);
}

void pwm_bb_set_deadtime(PwmBuckBoost *me, uint32_t deadtime_ns) {
  pwm_set_deadtime(&me->base, deadtime_ns);
}

uint8_t pwm_bb_get_num_phases(const PwmBuckBoost *me) {
  return me->num_phases;
}
