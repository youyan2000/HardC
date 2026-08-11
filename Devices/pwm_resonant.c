// 谐振变换器变频 PWM —— PwmBase 子类
//
// 核心: 变频控制, 占空比固定 50%, 死区保证 ZVS 软开关
//
// 频率换算: period = f_clk / freq
//   如: HRTIM 5.44GHz → 100kHz = 54400, 300kHz = 18133
//
// 50% 占空比 (中心对齐):
//   CMP1 = period/2 * 0.5 = period/4   (上升沿)
//   CMP3 = period/2 * 1.5 = 3*period/4 (下降沿)
//   脉冲宽度 = period/2 = 50%
//
// 死区: 上下管切换间隙的死区时间 = deadtime_ns (上升沿和下降沿各插入)

#include "pwm_resonant.h"
#include "container_of.h"
#include <stddef.h>

// ======== 内部辅助 ========

// 限制频率在允许范围内
static inline uint32_t clamp_freq(uint32_t freq, uint32_t min, uint32_t max) {
  if (freq < min) return min;
  if (freq > max) return max;
  return freq;
}

// 频率 → BSP 周期计数值
static inline uint32_t freq_to_period(BspPwmConfig *cfg, uint32_t freq_hz) {
  // period = f_clk / freq
  // HRTIM with DLL x32: f_clk = CPU_CLK * 32, 如 170MHz * 32 = 5.44GHz
  // C2000 ePWM:       f_clk = SYSCLK / prescaler
  return cfg->clk_hz / freq_hz;
}

// ======== ops 实现 ========

static void res_start(PwmBase *base) {
  PwmResonant *me = container_of(base, PwmResonant, base);

  uint32_t half = me->period / 2;
  uint32_t quarter = me->period / 4;

  BspPwmTimerConfig tcfg = {
    .timer         = me->timer,
    .period        = me->period,
    .output_mask   = me->output_mask,
    .complementary = true,   // 谐振必须互补输出 + 死区
    .cmp1          = quarter,             // 25% 处 SET (高侧导通起点)
    .cmp2          = half,                // 50% 中心参考
    .cmp3          = quarter * 3,         // 75% 处 RESET (高侧关断点)
    .deadtime_rising  = me->deadtime_ns,  // 上升沿死区 = ZVS 过渡时间
    .deadtime_falling = me->deadtime_ns,  // 下降沿死区
  };
  bsp_config_timer(me->bsp_cfg.handle, &tcfg);

  uint32_t timer_mask = (1u << me->timer);
  bsp_start(me->bsp_cfg.handle, timer_mask, me->output_mask);
}

static void res_stop(PwmBase *base) {
  PwmResonant *me = container_of(base, PwmResonant, base);
  uint32_t timer_mask = (1u << me->timer);
  bsp_stop(me->bsp_cfg.handle, timer_mask, me->output_mask);
}

// 谐振模式占空比固定 50%, 此接口允许微调 (如猝发模式 / 软启动)
static void res_set_duty(PwmBase *base, uint8_t ch, float duty) {
  (void)ch;
  PwmResonant *me = container_of(base, PwmResonant, base);

  // 限制偏离 50% 的范围 (谐振拓扑对占空比偏移敏感)
  if (duty < 0.40f) duty = 0.40f;
  if (duty > 0.60f) duty = 0.60f;

  uint32_t half = me->period / 2;
  uint32_t cmp1 = half * (1.0f - duty);
  uint32_t cmp3 = half * (1.0f + duty);

  bsp_update_duty(me->bsp_cfg.handle, me->timer, cmp1, cmp3);
}

// 变频: 谐振变换器的核心控制接口
static void res_set_freq(PwmBase *base, uint32_t freq_hz) {
  PwmResonant *me = container_of(base, PwmResonant, base);

  // 频率限幅: 不能超过谐振腔物理约束
  freq_hz = clamp_freq(freq_hz, me->freq_min, me->freq_max);
  me->freq_hz = freq_hz;
  me->period  = freq_to_period(&me->bsp_cfg, freq_hz);

  // 更新定时器周期
  bsp_update_period(me->bsp_cfg.handle, me->timer, me->period);

  // 保持 50% 占空比: period 变了, 重新计算 CMP1/CMP3
  uint32_t half    = me->period / 2;
  uint32_t quarter = me->period / 4;
  bsp_update_duty(me->bsp_cfg.handle, me->timer, quarter, quarter * 3);
}

static void res_set_deadtime(PwmBase *base, uint32_t deadtime_ns) {
  PwmResonant *me = container_of(base, PwmResonant, base);
  me->deadtime_ns = deadtime_ns;
  bsp_update_deadtime(me->bsp_cfg.handle, me->timer,
                          deadtime_ns, deadtime_ns);
}

static void res_set_phase(PwmBase *base, uint8_t ch, float phase_deg) {
  (void)base; (void)ch; (void)phase_deg;
  // 谐振拓扑: 单路互补输出, 无相位概念
}

static void res_emergency_stop(PwmBase *base) {
  PwmResonant *me = container_of(base, PwmResonant, base);
  bsp_emergency_stop(me->bsp_cfg.handle, me->output_mask);
}

// ======== 虚表 ========
static const PwmOps res_ops = {
  .start          = res_start,
  .stop           = res_stop,
  .set_duty       = res_set_duty,
  .set_freq       = res_set_freq,
  .set_deadtime   = res_set_deadtime,
  .set_phase      = res_set_phase,
  .emergency_stop = res_emergency_stop,
};

// ======== 构造 ========

void pwm_res_init(PwmResonant *me,
                  uint32_t freq_start, uint32_t freq_min, uint32_t freq_max,
                  uint32_t deadtime_ns,
                  BspPwmTimer timer, uint32_t output_mask) {
  pwm_base_init(&me->base);

  me->bsp_cfg.handle       = NULL;
  me->bsp_cfg.clk_hz = 0;
  me->bsp_cfg.use_dll      = false;

  me->timer       = timer;
  me->output_mask = output_mask;

  me->freq_hz    = freq_start;
  me->freq_min   = freq_min;
  me->freq_max   = freq_max;
  me->deadtime_ns = deadtime_ns;
  me->period     = 0;

  me->resonant_freq   = 0.0f;       // 可选: 由上层设置
  me->below_resonant  = false;      // 默认 ZVS 区 (高于谐振点)

  // 谐振模式: 占空比固定 50%
  me->base.mode     = PwmMode_Resonant;
  me->base.num_ch   = 1;
  me->base.duty_min = 0.48f;        // 谐振拓扑占空比基本锁定 50%
  me->base.duty_max = 0.52f;        // 仅允许微调 (±2%)
  me->base.ops      = &res_ops;

  bsp_init(&me->bsp_cfg);

  // 首设频率 (计算 period)
  res_set_freq(&me->base, freq_start);
}

void pwm_res_deinit(PwmResonant *me) {
  if (me->base.running) {
    res_stop(&me->base);
  }
  me->bsp_cfg.handle = NULL;
  pwm_base_init(&me->base);
}

// ======== 运行时调参 ========

void pwm_res_set_freq(PwmResonant *me, uint32_t freq_hz) {
  pwm_set_freq(&me->base, freq_hz);
}

void pwm_res_set_deadtime(PwmResonant *me, uint32_t deadtime_ns) {
  pwm_set_deadtime(&me->base, deadtime_ns);
}

uint32_t pwm_res_get_freq(PwmResonant *me) {
  return me->freq_hz;
}

void pwm_res_get_freq_range(PwmResonant *me, uint32_t *freq_min, uint32_t *freq_max) {
  *freq_min = me->freq_min;
  *freq_max = me->freq_max;
}

void pwm_res_set_resonant_params(PwmResonant *me, float resonant_freq, bool below_resonant) {
  me->resonant_freq  = resonant_freq;
  me->below_resonant = below_resonant;
}
