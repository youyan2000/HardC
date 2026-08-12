// 多相交错并联 PWM —— PwmBase 子类
//
// N 相均匀错相 360°/N, 每相独立半桥 (互补+死区).
// 所有相共用同一个频率和死区, 占空比可逐相微调以实现均流.
//
// 相位实现: 各相定时器复位触发源不同
//   第0相 → 复位=Master PERIOD  (0° 参考)
//   第1相 → 复位=Master CMP 偏移 (360/N °)
//   第2相 → 复位=Master CMP 偏移 (720/N °)
//   ...

#include "pwm_interleaved.h"
#include "container_of.h"
#include <stddef.h>

// ======== 内部辅助 ========

static inline float clamp_duty(float d) {
  if (d < 0.0f) return 0.0f;
  if (d > 1.0f) return 1.0f;
  return d;
}

// ======== ops 实现 ========

static void il_start(PwmBase *base) {
  PwmInterleaved *me = container_of(base, PwmInterleaved, base);

  uint32_t timer_mask  = 0;
  uint32_t output_mask = 0;

  // 逐相配置定时器
  for (uint8_t i = 0; i < me->num_phases; i++) {
    PwmInterleavedPhase *ph = &me->phases[i];
    uint32_t half = me->period / 2;

    BspPwmTimerConfig tcfg = {
      .timer         = ph->timer,
      .period        = me->period,
      .output_mask   = ph->output_mask,
      .complementary = true,
      .cmp1          = half * (1.0f - ph->duty),
      .cmp2          = half,  // 中心参考 (ADC 触发点)
      .cmp3          = half * (1.0f + ph->duty),
      .deadtime_rising  = me->deadtime_ns,
      .deadtime_falling = me->deadtime_ns,
    };
    bsp_config_timer(me->bsp_cfg.handle, &tcfg);

    timer_mask  |= (1u << ph->timer);
    output_mask |= ph->output_mask;
  }

  bsp_start(me->bsp_cfg.handle, timer_mask, output_mask);
}

static void il_stop(PwmBase *base) {
  PwmInterleaved *me = container_of(base, PwmInterleaved, base);

  uint32_t timer_mask  = 0;
  uint32_t output_mask = 0;
  for (uint8_t i = 0; i < me->num_phases; i++) {
    timer_mask  |= (1u << me->phases[i].timer);
    output_mask |= me->phases[i].output_mask;
  }
  bsp_stop(me->bsp_cfg.handle, timer_mask, output_mask);
}

// 设置某相占空比: ch = phase_idx
static void il_set_duty(PwmBase *base, uint8_t ch, float duty) {
  PwmInterleaved *me = container_of(base, PwmInterleaved, base);

  if (ch >= me->num_phases) return;

  duty = clamp_duty(duty);
  me->phases[ch].duty = duty;

  uint32_t half = me->period / 2;
  uint32_t cmp1 = half * (1.0f - duty);
  uint32_t cmp3 = half * (1.0f + duty);

  bsp_update_duty(me->bsp_cfg.handle, me->phases[ch].timer, cmp1, cmp3);
}

static void il_set_freq(PwmBase *base, uint32_t freq_hz) {
  PwmInterleaved *me = container_of(base, PwmInterleaved, base);
  me->period = me->bsp_cfg.clk_hz / freq_hz;

  // 所有相更新周期
  for (uint8_t i = 0; i < me->num_phases; i++) {
    bsp_update_period(me->bsp_cfg.handle, me->phases[i].timer, me->period);
  }

  // 重算所有相占空比
  for (uint8_t i = 0; i < me->num_phases; i++) {
    il_set_duty(base, i, me->phases[i].duty);
  }
}

static void il_set_deadtime(PwmBase *base, uint32_t deadtime_ns) {
  PwmInterleaved *me = container_of(base, PwmInterleaved, base);
  me->deadtime_ns = deadtime_ns;

  for (uint8_t i = 0; i < me->num_phases; i++) {
    bsp_update_deadtime(me->bsp_cfg.handle, me->phases[i].timer,
                            deadtime_ns, deadtime_ns);
  }
}

// 交错并联中相位由初始化时硬件复位触发源决定, 运行时不动态调整
static void il_set_phase(PwmBase *base, uint8_t ch, float phase_deg) {
  (void)base; (void)ch; (void)phase_deg;
  // 交错并联的相位在 init 时通过定时器复位触发源固定, 运行时不改变
}

static void il_emergency_stop(PwmBase *base) {
  PwmInterleaved *me = container_of(base, PwmInterleaved, base);
  uint32_t output_mask = 0;
  for (uint8_t i = 0; i < me->num_phases; i++) {
    output_mask |= me->phases[i].output_mask;
  }
  bsp_emergency_stop(me->bsp_cfg.handle, output_mask);
}

// ======== 虚表 ========
static const PwmOps il_ops = {
  .start          = il_start,
  .stop           = il_stop,
  .set_duty       = il_set_duty,
  .set_freq       = il_set_freq,
  .set_deadtime   = il_set_deadtime,
  .set_phase      = il_set_phase,       // 空操作 (相位在 init 固定)
  .emergency_stop = il_emergency_stop,
};

// ======== 构造 ========

void pwm_il_init(PwmInterleaved *me, uint32_t freq_hz, uint32_t deadtime_ns,
                 uint8_t num_phases,
                 const BspPwmTimer *timers,
                 const uint32_t *out_masks) {
  pwm_base_init(&me->base);

  if (num_phases > PWM_INTERLEAVED_MAX_PHASES) {
    num_phases = PWM_INTERLEAVED_MAX_PHASES;
  }
  if (num_phases == 0) {
    num_phases = 1;
  }

  me->bsp_cfg.handle       = NULL;
  me->bsp_cfg.clk_hz = 0;
  me->bsp_cfg.use_dll      = false;

  me->num_phases  = num_phases;
  me->deadtime_ns = deadtime_ns;
  me->period      = 0;
  me->center_aligned = true;

  // 初始化每相, 自动计算均匀相位偏移
  for (uint8_t i = 0; i < num_phases; i++) {
    me->phases[i].timer       = timers[i];
    me->phases[i].output_mask = out_masks[i];
    me->phases[i].duty        = 0.0f;
    me->phases[i].phase_deg   = (360.0f / num_phases) * i;  // 均匀错相
  }

  me->base.mode     = PwmMode_Interleaved;
  me->base.num_ch   = num_phases;
  me->base.duty_min = 0.0f;
  me->base.duty_max = 0.97f;            // 交错并联留 3% 死区裕量
  me->base.ops      = &il_ops;

  bsp_init(&me->bsp_cfg);
  il_set_freq(&me->base, freq_hz);
}

void pwm_il_deinit(PwmInterleaved *me) {
  if (me->base.running) {
    il_stop(&me->base);
  }
  me->num_phases = 0;
  me->bsp_cfg.handle = NULL;
  pwm_base_init(&me->base);
}

// ======== 运行时调参 ========

void pwm_il_set_duty(PwmInterleaved *me, uint8_t phase_idx, float duty) {
  pwm_set_duty(&me->base, phase_idx, duty);
}

void pwm_il_set_duty_all(PwmInterleaved *me, float duty) {
  for (uint8_t i = 0; i < me->num_phases; i++) {
    pwm_set_duty(&me->base, i, duty);
  }
}

void pwm_il_set_freq(PwmInterleaved *me, uint32_t freq_hz) {
  pwm_set_freq(&me->base, freq_hz);
}

void pwm_il_set_deadtime(PwmInterleaved *me, uint32_t deadtime_ns) {
  pwm_set_deadtime(&me->base, deadtime_ns);
}

uint8_t pwm_il_get_num_phases(PwmInterleaved *me) {
  return me->num_phases;
}
