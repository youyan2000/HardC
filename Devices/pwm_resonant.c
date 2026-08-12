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
  if (freq_hz == 0) return 0;
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

  // LLC burst mode 默认值
  me->burst_mode      = LlcBurstMode_Off;
  me->burst_threshold = 0.1f;       // 10% 功率阈值
  me->burst_on_cycles = 10;
  me->burst_off_cycles = 5;
  me->burst_counter   = 0;
  me->burst_active    = false;

  // ZVS 检测默认值
  me->zvs_state         = LlcZvsState_Unknown;
  me->zvs_fail_count    = 0;
  me->zvs_fail_limit    = 50;       // 连续 50 次硬开关则停机
  me->zvs_sense_enable  = false;
  me->zvs_vds_threshold = 10.0f;    // 10V 阈值

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

// ======== LLC burst mode (来源: TI controlSUITE PWMDRV_LLC) ========

void pwm_resonant_set_burst(PwmResonant *me, LlcBurstMode mode,
                             float threshold_pu, int on_cycles, int off_cycles) {
  me->burst_mode = mode;
  me->burst_threshold = threshold_pu;
  me->burst_on_cycles = on_cycles;
  me->burst_off_cycles = off_cycles;
  me->burst_counter = 0;
  me->burst_active = false;
}

// ISR 中每周期调用 — 决定是否跳过当前 PWM 周期
bool pwm_resonant_burst_tick(PwmResonant *me) {
  if (me->burst_mode == LlcBurstMode_Off) {
    return true;  // 永远输出
  }

  me->burst_counter++;

  if (me->burst_active) {
    // Burst ON 阶段
    if (me->burst_counter >= me->burst_on_cycles) {
      me->burst_active = false;
      me->burst_counter = 0;
    }
    return true;
  } else {
    // Burst OFF 阶段
    if (me->burst_counter >= me->burst_off_cycles) {
      me->burst_active = true;
      me->burst_counter = 0;
    }
    return false;  // 跳过本周期
  }
}

// ======== 自动 burst 模式 (来源: TI controlSUITE PWMDRV_LLC) ========

// 根据负载功率自动切换 burst / 连续模式
// 进入 burst: power_pu < burst_threshold 且当前为连续模式
// 退出 burst: power_pu > burst_threshold * 1.5 (迟滞 50%) 且当前为 burst 模式
bool pwm_resonant_burst_auto_update(PwmResonant *me, float power_pu) {
  if (me->burst_mode != LlcBurstMode_Auto) {
    return false;
  }

  bool was_active = me->burst_active;

  // 迟滞判断: 用不同的进出阈值防止振荡
  float exit_threshold = me->burst_threshold * 1.5f;
  if (exit_threshold > 0.9f) {
    exit_threshold = 0.9f;  // 上限 90%
  }

  if (!was_active && power_pu < me->burst_threshold) {
    // 轻载 → 进入 burst
    me->burst_active = true;
    me->burst_counter = 0;
    return true;
  } else if (was_active && power_pu > exit_threshold) {
    // 重载 → 退出 burst
    me->burst_active = false;
    me->burst_counter = 0;
    return true;
  }

  return false;  // 无变化
}

// ======== ZVS 检测 (来源: TI controlSUITE PWMDRV_LLC ZVS sense) ========

void pwm_resonant_zvs_config(PwmResonant *me, bool enable,
                              float vds_threshold_v, int fail_limit) {
  me->zvs_sense_enable  = enable;
  me->zvs_vds_threshold = vds_threshold_v;
  me->zvs_fail_limit    = fail_limit;
  me->zvs_fail_count    = 0;
  me->zvs_state         = LlcZvsState_Unknown;
}

// ZVS 检测: 在开关管开通前瞬间 (死区结束时) 采样 Vds
// Vds ≈ 0V → ZVS 成功 (体二极管已导通)
// Vds > threshold → 硬开关 (体二极管未导通, 有余压)
//
// 硬件实现:
//   STM32: HRTIM CMP4 触发 ADC 在死区末尾采样 Vds
//   C2000: ePWM CMPB 触发 S/H, 比较器输出直接连 TZ 子模块
//
// 调用时机: ISR 中, 在每个 PWM 周期的死区结束前 (由硬件触发)
LlcZvsState pwm_resonant_zvs_detect(PwmResonant *me, float vds_sample) {
  if (!me->zvs_sense_enable) {
    return LlcZvsState_Unknown;
  }

  // 判断 ZVS 状态
  if (vds_sample < me->zvs_vds_threshold * 0.3f) {
    // Vds 接近 0: ZVS 成功, 裕量充足
    me->zvs_state = LlcZvsState_Achieved;
    me->zvs_fail_count = 0;  // 复位失败计数
  } else if (vds_sample < me->zvs_vds_threshold) {
    // Vds 在中间范围: ZVS 临界 (裕量不足, 但未完全硬开关)
    me->zvs_state = LlcZvsState_Marginal;
    me->zvs_fail_count++;
  } else {
    // Vds > threshold: 硬开关 (ZVS 失败)
    me->zvs_state = LlcZvsState_HardSwitch;
    me->zvs_fail_count++;
  }

  // 硬开关保护: 连续失败超过上限 → 紧急停机
  if (me->zvs_fail_count >= me->zvs_fail_limit) {
    // 触发紧急停机: 关断所有 PWM 输出
    bsp_emergency_stop(me->bsp_cfg.handle, me->output_mask);
    me->base.running = false;
  }

  return me->zvs_state;
}

// 自适应死区: 根据 ZVS 状态动态调整死区时间
// ZVS 裕量不足 → 加长死区 (给励磁电流更多时间对 Coss 充放电)
// ZVS 成功且裕量充足 → 缩短死区 (降低体二极管导通损耗)
uint32_t pwm_resonant_adaptive_deadtime(PwmResonant *me) {
  if (!me->zvs_sense_enable) {
    return me->deadtime_ns;  // 未使能, 保持当前死区
  }

  uint32_t dt = me->deadtime_ns;

  switch (me->zvs_state) {
  case LlcZvsState_HardSwitch:
    // 硬开关: 大幅增加死区 (励磁电流需要更多时间完成 ZVS 过渡)
    dt = dt + dt / 2;  // +50%
    if (dt > me->deadtime_ns * 2) dt = me->deadtime_ns * 2;  // 上限 2x
    break;
  case LlcZvsState_Marginal:
    // 临界: 微调死区
    dt = dt + dt / 8;  // +12.5%
    break;
  case LlcZvsState_Achieved:
    // ZVS 成功: 逐步缩短死区以降低损耗
    if (dt > 50) {  // 绝对下限 50ns
      dt = dt - dt / 16;  // -6.25%
    }
    break;
  case LlcZvsState_Unknown:
  default:
    break;
  }

  // 硬件限幅
  if (dt < 50) dt = 50;
  if (dt > 2000) dt = 2000;

  // 应用到硬件
  if (dt != me->deadtime_ns) {
    me->deadtime_ns = dt;
    bsp_update_deadtime(me->bsp_cfg.handle, me->timer, dt, dt);
  }

  return dt;
}
