// 单路 Buck/Boost PWM —— PwmBase 子类
//
// 最简单的电力电子 PWM 拓扑:
//   异步模式: 仅高侧管 PWM, 低侧由二极管续流
//   同步整流: 高侧管 PWM + 低侧管互补 (加死区), 降低导通损耗
//
// 占空比转换:
//   Buck:   duty = Vout / Vin, 高侧管按 duty 导通
//   Boost:  duty = 1 - Vin / Vout, 低侧管按 (1-duty) 导通
//   升降压: 两管交替,  模式由上层切换

#include "pwm_buckboost.h"
#include "container_of.h"
#include <stddef.h>

// ======== 内部辅助 ========

static inline float clamp_duty(float d) {
  if (d < 0.0f) return 0.0f;
  if (d > 1.0f) return 1.0f;
  return d;
}

// ======== ops 实现 ========

static void bb_start(PwmBase *base) {
  PwmBuckBoost *me = container_of(base, PwmBuckBoost, base);

  uint32_t half = me->period / 2;
  BspPwmTimerConfig tcfg = {
    .timer         = me->timer,
    .period        = me->period,
    .output_mask   = me->output_mask,
    .complementary = me->sync_rect,  // 同步整流时才启用互补+死区
    .cmp1          = me->center_aligned ? half * (1.0f - me->duty)
                                        : me->period * me->duty,
    .cmp2          = me->center_aligned ? half : 0,
    .cmp3          = me->center_aligned ? half * (1.0f + me->duty) : 0,
  };

  if (me->sync_rect) {
    tcfg.deadtime_rising  = me->deadtime_ns;
    tcfg.deadtime_falling = me->deadtime_ns;
  }

  bsp_config_timer(me->bsp_cfg.handle, &tcfg);

  uint32_t timer_mask = (1u << me->timer);
  bsp_start(me->bsp_cfg.handle, timer_mask, me->output_mask);
}

static void bb_stop(PwmBase *base) {
  PwmBuckBoost *me = container_of(base, PwmBuckBoost, base);
  uint32_t timer_mask = (1u << me->timer);
  bsp_stop(me->bsp_cfg.handle, timer_mask, me->output_mask);
}

static void bb_set_duty(PwmBase *base, uint8_t ch, float duty) {
  (void)ch;
  PwmBuckBoost *me = container_of(base, PwmBuckBoost, base);
  duty = clamp_duty(duty);
  me->duty = duty;

  uint32_t cmp1, cmp3;
  if (me->center_aligned) {
    uint32_t half = me->period / 2;
    cmp1 = half * (1.0f - duty);
    cmp3 = half * (1.0f + duty);
  } else {
    cmp1 = me->period * duty;
    cmp3 = 0;
  }

  bsp_update_duty(me->bsp_cfg.handle, me->timer, cmp1, cmp3);
}

static void bb_set_freq(PwmBase *base, uint32_t freq_hz) {
  PwmBuckBoost *me = container_of(base, PwmBuckBoost, base);
  me->period = me->bsp_cfg.clk_hz / freq_hz;

  bsp_update_period(me->bsp_cfg.handle, me->timer, me->period);

  // 保持占空比重映射
  bb_set_duty(base, 0, me->duty);
}

static void bb_set_deadtime(PwmBase *base, uint32_t deadtime_ns) {
  PwmBuckBoost *me = container_of(base, PwmBuckBoost, base);
  me->deadtime_ns = deadtime_ns;

  if (me->sync_rect) {
    bsp_update_deadtime(me->bsp_cfg.handle, me->timer,
                            deadtime_ns, deadtime_ns);
  }
}

static void bb_set_phase(PwmBase *base, uint8_t ch, float phase_deg) {
  (void)base; (void)ch; (void)phase_deg;
  // 单路拓扑无需相位
}

static void bb_emergency_stop(PwmBase *base) {
  PwmBuckBoost *me = container_of(base, PwmBuckBoost, base);
  bsp_emergency_stop(me->bsp_cfg.handle, me->output_mask);
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

void pwm_bb_init(PwmBuckBoost *me, uint32_t freq_hz,
                 BspPwmTimer timer, uint32_t output_mask,
                 PwmMode mode, bool sync_rect) {
  pwm_base_init(&me->base);

  me->bsp_cfg.handle       = NULL;
  me->bsp_cfg.clk_hz = 0;
  me->bsp_cfg.use_dll      = false;

  me->timer       = timer;
  me->output_mask = output_mask;

  me->duty        = 0.0f;
  me->deadtime_ns = 0;
  me->period      = 0;
  me->mode        = mode;
  me->sync_rect   = sync_rect;
  me->center_aligned = true;

  me->base.mode     = mode;
  me->base.num_ch   = 1;
  me->base.duty_min = 0.0f;
  me->base.duty_max = sync_rect ? 0.95f : 0.98f;  // 同步整流留死区裕量
  me->base.ops      = &bb_ops;

  bsp_init(&me->bsp_cfg);
  bb_set_freq(&me->base, freq_hz);
}

void pwm_bb_deinit(PwmBuckBoost *me) {
  if (me->base.running) {
    bb_stop(&me->base);
  }
  me->bsp_cfg.handle = NULL;
  pwm_base_init(&me->base);
}

// ======== 运行时调参 ========

void pwm_bb_set_duty(PwmBuckBoost *me, float duty) {
  pwm_set_duty(&me->base, 0, duty);
}

void pwm_bb_set_freq(PwmBuckBoost *me, uint32_t freq_hz) {
  pwm_set_freq(&me->base, freq_hz);
}

void pwm_bb_set_deadtime(PwmBuckBoost *me, uint32_t deadtime_ns) {
  pwm_set_deadtime(&me->base, deadtime_ns);
}

void pwm_bb_set_mode(PwmBuckBoost *me, PwmMode mode) {
  me->mode = mode;
  me->base.mode = mode;
}
