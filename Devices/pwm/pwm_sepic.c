// SEPIC 单开关变频 PWM 实现
//
// 来源: TI controlSUITE DPLib PWMDRV_1ch (digital_power)
// 翻译为 HardC 风格
//
// 核心机制:
//   - 单通道 PWM, 占空比控制输出/输入电压比: Vout/Vin = D/(1-D)
//   - 变频控制: 重载升频 (小纹波), 轻载降频 (降损耗)
//   - BCM 模式下谷值开关, 进一步降低开关损耗

#include "pwm_sepic.h"
#include "container_of.h"
#include <stddef.h>

// ======== 内部辅助 ========

static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// 占空比 → 比较值 (边沿对齐: CMP1 = period * duty)
static inline uint32_t duty_to_cmp(uint32_t period, float duty) {
  float d = clampf(duty, 0.0f, 1.0f);
  return (uint32_t)(period * d);
}

// ======== ops 实现 ========

static void sepic_start_fn(PwmBase *base) {
  PwmSepic *me = container_of(base, PwmSepic, base);
  (void)me;
  // BSP: 配置单通道边沿对齐 PWM
  // bsp_config_timer(...) + bsp_start(...)
}

static void sepic_stop_fn(PwmBase *base) {
  PwmSepic *me = container_of(base, PwmSepic, base);
  (void)me;
  // bsp_stop(...)
}

static void sepic_set_duty_fn(PwmBase *base, uint8_t ch, float duty) {
  (void)ch;  // 单通道, 忽略通道号
  PwmSepic *me = container_of(base, PwmSepic, base);
  float d = clampf(duty, 0.0f, 1.0f);
  me->duty = d;

  uint32_t cmp1 = duty_to_cmp(me->period, d);
  // bsp_update_duty(bsp->handle, me->timer, cmp1, 0);
  (void)cmp1;
}

static void sepic_set_freq_fn(PwmBase *base, uint32_t freq_hz) {
  PwmSepic *me = container_of(base, PwmSepic, base);

  if (freq_hz < me->freq_min_hz) freq_hz = me->freq_min_hz;
  if (freq_hz > me->freq_max_hz) freq_hz = me->freq_max_hz;

  me->freq_curr_hz = freq_hz;
  // period = clk_hz / freq_hz (BSP 内部换算)
  // bsp_update_period(bsp->handle, me->timer, period);
}

static void sepic_set_deadtime_fn(PwmBase *base, uint32_t deadtime_ns) {
  // SEPIC 单开关不需要死区 (无互补输出)
  (void)base; (void)deadtime_ns;
}

static void sepic_set_phase_fn(PwmBase *base, uint8_t ch, float phase_deg) {
  // 单通道无相位概念
  (void)base; (void)ch; (void)phase_deg;
}

static void sepic_emergency_stop_fn(PwmBase *base) {
  PwmSepic *me = container_of(base, PwmSepic, base);
  (void)me;
  // bsp_emergency_stop(bsp->handle, me->output_mask);
}

// ======== 虚表 ========
static const PwmOps sepic_ops = {
  .start          = sepic_start_fn,
  .stop           = sepic_stop_fn,
  .set_duty       = sepic_set_duty_fn,
  .set_freq       = sepic_set_freq_fn,
  .set_deadtime   = sepic_set_deadtime_fn,
  .set_phase      = sepic_set_phase_fn,
  .emergency_stop = sepic_emergency_stop_fn,
};

// ======== 构造 ========

void sepic_init(PwmSepic *me, uint32_t freq_hz, float duty,
                BspPwmTimer timer, uint32_t output_mask) {
  pwm_base_init(&me->base);

  me->bsp_cfg.handle  = NULL;
  me->bsp_cfg.clk_hz  = 0;
  me->bsp_cfg.use_dll = false;

  me->timer       = timer;
  me->output_mask = output_mask;

  me->freq_min_hz  = 20000;      // 默认最低 20kHz (人耳听阈以上)
  me->freq_max_hz  = 200000;     // 默认最高 200kHz
  me->freq_curr_hz = freq_hz;
  me->freq_step_hz = 1000.0f;    // 默认 1kHz 步长

  me->duty    = clampf(duty, 0.0f, 0.9f);  // SEPIC 占空比建议 ≤ 90%
  me->period  = 0;  // 由 BSP 初始化后计算

  me->mode           = SepicMode_CCM;
  me->freq_variable  = false;     // 默认定频, 需显式启用

  me->i_inductor       = 0.0f;
  me->i_threshold_dcm  = 0.1f;    // 默认 0.1A 以下判定 DCM

  // 基类字段
  me->base.mode     = PwmMode_BuckBoost;   // SEPIC 属升降压拓扑
  me->base.num_ch   = 1;                    // 单通道
  me->base.freq_hz  = freq_hz;
  me->base.duty_min = 0.0f;                // SEPIC 可输出 0%
  me->base.duty_max = 0.90f;               // 上限 90%, 留耦合电容充放电裕量
  me->base.ops      = &sepic_ops;
}

void sepic_deinit(PwmSepic *me) {
  if (me->base.running) {
    sepic_stop_fn(&me->base);
  }
  me->timer       = 0;
  me->output_mask = 0;
  me->duty        = 0.0f;
  me->period      = 0;
  pwm_base_init(&me->base);
}

// ======== 运行时调参 ========

void sepic_set_duty(PwmSepic *me, float duty) {
  pwm_set_duty(&me->base, 0, duty);
}

void sepic_set_point(PwmSepic *me, float duty, uint32_t freq_hz) {
  // 先调频率 (可能改 period), 再调占空比 (依赖 period)
  if (me->freq_variable && freq_hz != me->freq_curr_hz) {
    sepic_set_freq_fn(&me->base, freq_hz);
  }
  sepic_set_duty_fn(&me->base, 0, duty);
}

uint32_t sepic_auto_freq(PwmSepic *me, float i_inductor) {
  me->i_inductor = i_inductor;

  // 自动变频策略:
  //   - 大电流 (重载): 升到最高频 → 小纹波, 低 EMI
  //   - 中电流 (中载): 线性插值
  //   - 小电流 (轻载): 降到最低频 → 降开关损耗, 提轻载效率
  float i_nom = me->i_threshold_dcm * 20.0f;  // 标称电流 = 20×DCM阈值
  float ratio;

  if (i_nom <= 0.0f) {
    ratio = 0.0f;
  } else {
    ratio = clampf(i_inductor / i_nom, 0.0f, 1.0f);
  }

  // 频率线性插值: 轻载→freq_min, 重载→freq_max
  uint32_t freq = me->freq_min_hz
                + (uint32_t)(ratio * (me->freq_max_hz - me->freq_min_hz));

  // 按步长取整
  freq = (freq / (uint32_t)me->freq_step_hz) * (uint32_t)me->freq_step_hz;

  if (freq != me->freq_curr_hz) {
    sepic_set_freq_fn(&me->base, freq);
  }

  return freq;
}

void sepic_detect_mode(PwmSepic *me, float i_inductor) {
  me->i_inductor = i_inductor;

  if (i_inductor <= me->i_threshold_dcm && i_inductor > 0.0f) {
    me->mode = SepicMode_DCM;
  } else if (i_inductor <= 0.0f) {
    me->mode = SepicMode_BCM;  // 电感电流过零 = 临界模式
  } else {
    me->mode = SepicMode_CCM;
  }
}
