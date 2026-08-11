// 全桥移相 PWM —— PwmBase 子类
//
// 两腿 H 桥 + 移相控制: A腿和 B腿各自独立输出互补 PWM,
// 功率传输由两腿之间的相位差控制: P ∝ sin(phase_deg)
//
// 移相实现: B腿的定时器复位触发源链接到 A腿的相位参考点 (CMP2),
// 而非 Master Period. 这样 B腿波形相对于 A腿整体偏移.
//
// A腿: 复位 = Master PERIOD  (0° 参考)
// B腿: 复位 = A腿 CMP2       (phase_deg 偏移)
//
// 占空比公式 (中心对齐):
//   CMP1 = period/2 * (1 - duty)   — 上升沿 (SET)
//   CMP3 = period/2 * (1 + duty)   — 下降沿 (RESET)
//   CMP2 = period/2                — 中心参考 (也是 B腿的复位触发源)

#include "pwm_full_bridge.h"
#include "container_of.h"
#include <stddef.h>
#include <math.h>

// ======== 内部辅助 ========

static inline float clamp_duty(float duty) {
  if (duty < 0.0f) return 0.0f;
  if (duty > 1.0f) return 1.0f;
  return duty;
}

static inline float clamp_phase(float deg) {
  // 移相角限于 0~180 度
  if (deg < 0.0f)   return 0.0f;
  if (deg > 180.0f) return 180.0f;
  return deg;
}

// B腿相位偏移量 (计数值): phase_counts = period * (phase_deg / 360)
// 实际: B腿 CMP2 = period/2 + period * (phase_deg / 360)
// 当 phase=0:   CMP2=period/2, B腿与 A腿同相
// 当 phase=180: CMP2=period, B腿完全反相
static inline uint32_t calc_phase_cmp2(uint32_t period, float phase_deg) {
  return (uint32_t)(period / 2 + period * (phase_deg / 360.0f));
}

// ======== ops 实现 ========

static void fb_start(PwmBase *base) {
  PwmFullBridge *me = container_of(base, PwmFullBridge, base);

  // === A腿配置 ===
  BspPwmTimerConfig tcfg_a = {
    .timer         = me->timer_a,
    .period        = me->period,
    .output_mask   = me->output_mask_a,
    .complementary = true,
    .cmp1          = (me->period / 2) * (1.0f - me->duty_a),
    .cmp2          = me->period / 2,                                 // 中心点
    .cmp3          = (me->period / 2) * (1.0f + me->duty_a),
    .deadtime_rising  = me->deadtime_ns,
    .deadtime_falling = me->deadtime_ns,
  };
  bsp_config_timer(me->bsp_cfg.handle, &tcfg_a);

  // === B腿配置 (CMP2 偏移实现移相) ===
  BspPwmTimerConfig tcfg_b = {
    .timer         = me->timer_b,
    .period        = me->period,
    .output_mask   = me->output_mask_b,
    .complementary = true,
    .cmp1          = (me->period / 2) * (1.0f - me->duty_b),
    .cmp2          = calc_phase_cmp2(me->period, me->phase_deg),  // 移相
    .cmp3          = (me->period / 2) * (1.0f + me->duty_b),
    .deadtime_rising  = me->deadtime_ns,
    .deadtime_falling = me->deadtime_ns,
  };
  bsp_config_timer(me->bsp_cfg.handle, &tcfg_b);

  // 启 A腿和 B腿
  uint32_t timer_mask = (1u << me->timer_a) | (1u << me->timer_b);
  uint32_t output_mask = me->output_mask_a | me->output_mask_b;
  bsp_start(me->bsp_cfg.handle, timer_mask, output_mask);
}

static void fb_stop(PwmBase *base) {
  PwmFullBridge *me = container_of(base, PwmFullBridge, base);
  uint32_t timer_mask  = (1u << me->timer_a) | (1u << me->timer_b);
  uint32_t output_mask = me->output_mask_a | me->output_mask_b;
  bsp_stop(me->bsp_cfg.handle, timer_mask, output_mask);
}

// 设置占空比: ch=0→A腿, ch=1→B腿
static void fb_set_duty(PwmBase *base, uint8_t ch, float duty) {
  PwmFullBridge *me = container_of(base, PwmFullBridge, base);
  duty = clamp_duty(duty);

  uint32_t half = me->period / 2;
  uint32_t cmp1 = half * (1.0f - duty);
  uint32_t cmp3 = half * (1.0f + duty);

  BspPwmTimer timer = (ch == 0) ? me->timer_a : me->timer_b;

  if (ch == 0) {
    me->duty_a = duty;
  } else {
    me->duty_b = duty;
  }

  bsp_update_duty(me->bsp_cfg.handle, timer, cmp1, cmp3);
}

static void fb_set_freq(PwmBase *base, uint32_t freq_hz) {
  PwmFullBridge *me = container_of(base, PwmFullBridge, base);
  me->period = me->bsp_cfg.clk_hz / freq_hz;

  // 两腿同频
  bsp_update_period(me->bsp_cfg.handle, me->timer_a, me->period);
  bsp_update_period(me->bsp_cfg.handle, me->timer_b, me->period);

  // 重算占空比比较值
  fb_set_duty(base, 0, me->duty_a);
  fb_set_duty(base, 1, me->duty_b);

  // 重算移相
  uint32_t phase_cmp2 = calc_phase_cmp2(me->period, me->phase_deg);
  // BSP: 通过更新 B腿 CMP2 实现移相调整
  // (具体实现由 BSP 将 CMP2 写入对应硬件寄存器)
}

static void fb_set_deadtime(PwmBase *base, uint32_t deadtime_ns) {
  PwmFullBridge *me = container_of(base, PwmFullBridge, base);
  me->deadtime_ns = deadtime_ns;
  bsp_update_deadtime(me->bsp_cfg.handle, me->timer_a,
                          deadtime_ns, deadtime_ns);
  bsp_update_deadtime(me->bsp_cfg.handle, me->timer_b,
                          deadtime_ns, deadtime_ns);
}

// 移相: 更新 B腿相对于 A腿的相位
static void fb_set_phase(PwmBase *base, uint8_t ch, float phase_deg) {
  (void)ch;
  PwmFullBridge *me = container_of(base, PwmFullBridge, base);
  me->phase_deg = clamp_phase(phase_deg);

  // 移相通过调整 B腿的 CMP2 (相位参考/复位触发源) 实现
  uint32_t phase_cmp2 = calc_phase_cmp2(me->period, me->phase_deg);
  // BSP: bsp_set_phase_shift 内部写 B腿 CMP2 寄存器
  bsp_set_phase_shift(me->bsp_cfg.handle, me->timer_b, me->phase_deg);
}

static void fb_emergency_stop(PwmBase *base) {
  PwmFullBridge *me = container_of(base, PwmFullBridge, base);
  uint32_t output_mask = me->output_mask_a | me->output_mask_b;
  bsp_emergency_stop(me->bsp_cfg.handle, output_mask);
}

// ======== 虚表 ========
static const PwmOps fb_ops = {
  .start          = fb_start,
  .stop           = fb_stop,
  .set_duty       = fb_set_duty,
  .set_freq       = fb_set_freq,
  .set_deadtime   = fb_set_deadtime,
  .set_phase      = fb_set_phase,
  .emergency_stop = fb_emergency_stop,
};

// ======== 构造 ========

void pwm_fb_init(PwmFullBridge *me, uint32_t freq_hz, uint32_t deadtime_ns,
                 BspPwmTimer timer_a, BspPwmTimer timer_b,
                 uint32_t out_mask_a, uint32_t out_mask_b) {
  pwm_base_init(&me->base);

  me->bsp_cfg.handle       = NULL;
  me->bsp_cfg.clk_hz = 0;
  me->bsp_cfg.use_dll      = false;

  me->timer_a       = timer_a;
  me->timer_b       = timer_b;
  me->output_mask_a = out_mask_a;
  me->output_mask_b = out_mask_b;

  me->duty_a       = 0.0f;
  me->duty_b       = 0.0f;
  me->deadtime_ns  = deadtime_ns;
  me->phase_deg    = 0.0f;
  me->period       = 0;
  me->center_aligned = true;

  me->base.mode     = PwmMode_FullBridge;
  me->base.num_ch   = 2;               // A腿 + B腿 = 2 个可独立控制通道
  me->base.duty_min = 0.0f;
  me->base.duty_max = 0.95f;           // 全桥需留死区裕量
  me->base.ops      = &fb_ops;

  bsp_init(&me->bsp_cfg);
  fb_set_freq(&me->base, freq_hz);
}

void pwm_fb_deinit(PwmFullBridge *me) {
  if (me->base.running) {
    fb_stop(&me->base);
  }
  me->bsp_cfg.handle = NULL;
  pwm_base_init(&me->base);
}

// ======== 运行时调参 ========

void pwm_fb_set_duty_a(PwmFullBridge *me, float duty) {
  pwm_set_duty(&me->base, 0, duty);
}

void pwm_fb_set_duty_b(PwmFullBridge *me, float duty) {
  pwm_set_duty(&me->base, 1, duty);
}

void pwm_fb_set_phase(PwmFullBridge *me, float phase_deg) {
  pwm_set_phase(&me->base, 0, phase_deg);
}

void pwm_fb_set_freq(PwmFullBridge *me, uint32_t freq_hz) {
  pwm_set_freq(&me->base, freq_hz);
}

void pwm_fb_set_deadtime(PwmFullBridge *me, uint32_t deadtime_ns) {
  pwm_set_deadtime(&me->base, deadtime_ns);
}
