// 全桥移相 PWM —— PwmBase 子类
//
// 两腿 H 桥 + 移相控制: A腿和 B腿各自独立输出互补 PWM,
// 功率传输由两腿之间的相位差控制: P ∝ sin(phase_deg)
//
// 移相实现: B腿相对 A腿滞后 phase_deg. F334 无 TIMACMP (Timer A 的 CMP 不能复位其它定时器),
// 跨定时器相位走主定时器 CMP 锚点 (bsp_set_phase_shift): B腿 RSTxR = MSTCMPn,
// MCMPnR = MPER×deg/360; phase 0 腿不接线, 随 Master PERIOD 自由运行即 0° 基准.
// 同定时器 CMP2xR = period×deg/360 为 CMP2 移相值 (calc_phase_cmp2, 与 BSP 写寄存器同式).
//
// A腿: 复位 = Master PERIOD     (0° 参考, 自由运行)
// B腿: 复位 = MSTCMPn (deg/360) (滞后 phase_deg)
//
// 占空比公式 (中心对齐):
//   CMP1 = period/2 * (1 - duty)   — 上升沿 (SET)
//   CMP3 = period/2 * (1 + duty)   — 下降沿 (RESET)
//   CMP2 = period * phase_deg/360  — 移相值 (bsp_set_phase_shift 写 CMP2xR 同式)

#include "pwm_full_bridge.h"
#include "container_of.h"
#include <stddef.h>
#include <math.h>

// ======== 内部辅助 ========

static inline float clamp_duty(float duty) {
  if (duty < 0.0f)
    return 0.0f;
  if (duty > 1.0f)
    return 1.0f;
  return duty;
}

static inline float clamp_phase(float deg) {
  // 移相角限于 0~180 度
  if (deg < 0.0f)
    return 0.0f;
  if (deg > 180.0f)
    return 180.0f;
  return deg;
}

static inline uint32_t calc_phase_cmp2(uint32_t period, float phase_deg) {
  // 与 bsp_set_phase_shift (bsp_hrtim.c) 写 CMP2xR 同公式: period×deg/360
  return (uint32_t) ((float) period * phase_deg / 360.0f);
}

// ======== ops 实现 ========

static void fb_start(PwmBase *base) {
  PwmFullBridge *me = container_of(base, PwmFullBridge, base);

  // === A腿配置 ===
  BspPwmTimerConfig tcfg_a = {
      .timer = me->timer_a,
      .period = me->period,
      .output_mask = me->output_mask_a,
      .complementary = true,
      .cmp1 = (me->period / 2) * (1.0f - me->duty_a),
      .cmp2 = me->period / 2,  // 中心点
      .cmp3 = (me->period / 2) * (1.0f + me->duty_a),
      .deadtime_rising = me->deadtime_ns,
      .deadtime_falling = me->deadtime_ns,
  };
  bsp_config_timer(me->bsp_cfg.handle, &tcfg_a);

  // === B腿配置 (CMP2 偏移实现移相) ===
  BspPwmTimerConfig tcfg_b = {
      .timer = me->timer_b,
      .period = me->period,
      .output_mask = me->output_mask_b,
      .complementary = true,
      .cmp1 = (me->period / 2) * (1.0f - me->duty_b),
      .cmp2 = calc_phase_cmp2(me->period, me->phase_deg),  // 移相
      .cmp3 = (me->period / 2) * (1.0f + me->duty_b),
      .deadtime_rising = me->deadtime_ns,
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
  uint32_t timer_mask = (1u << me->timer_a) | (1u << me->timer_b);
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
  if (freq_hz == 0)
    return;  // 防除零
  me->period = me->bsp_cfg.clk_hz / freq_hz;

  // 两腿同频
  bsp_update_period(me->bsp_cfg.handle, me->timer_a, me->period);
  bsp_update_period(me->bsp_cfg.handle, me->timer_b, me->period);

  // 重算占空比比较值
  fb_set_duty(base, 0, me->duty_a);
  fb_set_duty(base, 1, me->duty_b);

  // 重算移相 — period 变了, B腿 CMP2 相位偏移需同步更新
  bsp_set_phase_shift(me->bsp_cfg.handle, me->timer_b, me->phase_deg);
}

static void fb_set_deadtime(PwmBase *base, uint32_t deadtime_ns) {
  PwmFullBridge *me = container_of(base, PwmFullBridge, base);
  me->deadtime_ns = deadtime_ns;
  bsp_update_deadtime(me->bsp_cfg.handle, me->timer_a, deadtime_ns, deadtime_ns);
  bsp_update_deadtime(me->bsp_cfg.handle, me->timer_b, deadtime_ns, deadtime_ns);
}

// 移相: 更新 B腿相对于 A腿的相位
static void fb_set_phase(PwmBase *base, uint8_t ch, float phase_deg) {
  (void) ch;
  PwmFullBridge *me = container_of(base, PwmFullBridge, base);
  me->phase_deg = clamp_phase(phase_deg);

  // 移相通过调整 B腿的 CMP2 (相位参考/复位触发源) 实现
  // BSP: bsp_set_phase_shift 内部计算 CMP2 偏移并写硬件寄存器
  bsp_set_phase_shift(me->bsp_cfg.handle, me->timer_b, me->phase_deg);
}

static void fb_emergency_stop(PwmBase *base) {
  PwmFullBridge *me = container_of(base, PwmFullBridge, base);
  uint32_t output_mask = me->output_mask_a | me->output_mask_b;
  bsp_emergency_stop(me->bsp_cfg.handle, output_mask);
}

// ======== 虚表 ========
static const PwmOps fb_ops = {
    .start = fb_start,
    .stop = fb_stop,
    .set_duty = fb_set_duty,
    .set_freq = fb_set_freq,
    .set_deadtime = fb_set_deadtime,
    .set_phase = fb_set_phase,
    .emergency_stop = fb_emergency_stop,
};

// ======== 构造 ========

void pwm_fb_init(PwmFullBridge *me, uint32_t freq_hz, uint32_t deadtime_ns, BspPwmTimer timer_a, BspPwmTimer timer_b,
                 uint32_t out_mask_a, uint32_t out_mask_b) {
  pwm_base_init(&me->base);

  me->bsp_cfg.handle = NULL;
  me->bsp_cfg.clk_hz = 0;
  me->bsp_cfg.use_dll = false;

  me->timer_a = timer_a;
  me->timer_b = timer_b;
  me->output_mask_a = out_mask_a;
  me->output_mask_b = out_mask_b;

  me->duty_a = 0.0f;
  me->duty_b = 0.0f;
  me->deadtime_ns = deadtime_ns;
  me->phase_deg = 0.0f;
  me->period = 0;
  me->center_aligned = true;

  // PSFB ZVS 自适应默认值
  me->zvs_adaptive_enable = false;
  me->zvs_min_deadtime_ns = 100.0f;  // 最小死区 100ns
  me->zvs_max_deadtime_ns = 500.0f;  // 最大死区 500ns
  me->zvs_current_threshold = 1.0f;  // 1A 阈值
  me->duty_loss_comp = 0.0f;         // 无补偿
  me->zvs_state = PsfbZvsState_Unknown;
  me->zvs_margin_pu = 1.0f;  // 初始裕量充足

  me->base.mode = PwmMode_FullBridge;
  me->base.num_ch = 2;  // A腿 + B腿 = 2 个可独立控制通道
  me->base.duty_min = 0.0f;
  me->base.duty_max = 0.95f;  // 全桥需留死区裕量
  me->base.ops = &fb_ops;

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

// ======== PSFB ZVS 自适应 (来源: TI controlSUITE PWMDRV_PSFB) ========

void pwm_fb_set_zvs_adaptive(PwmFullBridge *me, bool enable, float min_ns, float max_ns, float i_threshold_a) {
  me->zvs_adaptive_enable = enable;
  me->zvs_min_deadtime_ns = min_ns;
  me->zvs_max_deadtime_ns = max_ns;
  me->zvs_current_threshold = i_threshold_a;
}

void pwm_fb_set_duty_loss_comp(PwmFullBridge *me, float comp) {
  if (comp < 0.0f)
    comp = 0.0f;
  if (comp > 0.2f)
    comp = 0.2f;
  me->duty_loss_comp = comp;
}

// 自适应死区: 轻载→长死区(确保 ZVS), 重载→短死区(降导通损耗)
float pwm_fb_adaptive_deadtime(PwmFullBridge *me, float i_load) {
  if (!me->zvs_adaptive_enable) {
    return me->zvs_max_deadtime_ns;  // 禁用→最大死区 (安全侧, 防直通)
  }

  float i_abs = (i_load < 0.0f) ? -i_load : i_load;

  // 结合 ZVS 裕量: 裕量越低, 死区越偏 max
  //   dt = dt_basic + (1 - zvs_margin) * (max - min) * 0.5
  float dt_basic;
  if (i_abs >= me->zvs_current_threshold) {
    dt_basic = me->zvs_min_deadtime_ns;
  } else if (i_abs <= 0.0f) {
    dt_basic = me->zvs_max_deadtime_ns;
  } else {
    float ratio = i_abs / me->zvs_current_threshold;
    dt_basic = me->zvs_max_deadtime_ns - ratio * (me->zvs_max_deadtime_ns - me->zvs_min_deadtime_ns);
  }

  // ZVS 裕量修正: 裕量不足时加长死区
  float margin_corr = (1.0f - me->zvs_margin_pu) * (me->zvs_max_deadtime_ns - me->zvs_min_deadtime_ns) * 0.5f;
  float dt_final = dt_basic + margin_corr;

  // 硬件限幅
  if (dt_final < me->zvs_min_deadtime_ns)
    dt_final = me->zvs_min_deadtime_ns;
  if (dt_final > me->zvs_max_deadtime_ns)
    dt_final = me->zvs_max_deadtime_ns;

  return dt_final;
}

// ======== PSFB ZVS 裕量估计 (来源: TI controlSUITE PWMDRV_PSFB) ========
//
// PSFB ZVS 条件:
//   超前腿 ZVS: 输出滤波电感能量 (大) → 全负载范围易实现 ZVS
//   滞后腿 ZVS: 谐振电感能量 (小) → 轻载时 ZVS 丢失
//
// 判断逻辑:
//   超前腿: i_leading > I_ZVS_min → ZVS OK, 否则丢失
//   滞后腿: i_lagging > I_ZVS_min → ZVS OK, 否则丢失
//   I_ZVS_min = sqrt(Coss * Vbus^2 / L_resonant) (典型 ~0.5A)
PsfbZvsState pwm_fb_zvs_margin_update(PwmFullBridge *me, float i_leading, float i_lagging, float vds_sample) {
  // 最小 ZVS 电流 (粗略估计, 典型值 0.3~0.8A)
  float i_zvs_min = me->zvs_current_threshold * 0.5f;

  // 电流幅值判断
  float i_lead_abs = (i_leading < 0.0f) ? -i_leading : i_leading;
  float i_lag_abs = (i_lagging < 0.0f) ? -i_lagging : i_lagging;

  bool lead_zvs = (i_lead_abs > i_zvs_min);
  bool lag_zvs = (i_lag_abs > i_zvs_min);

  // Vds 硬开关检测 (硬件比较器/ADC)
  bool vds_ok = (vds_sample < 10.0f);  // Vds < 10V → ZVS 实现

  if (lead_zvs && lag_zvs && vds_ok) {
    me->zvs_state = PsfbZvsState_Achieved;
    me->zvs_margin_pu = 1.0f;
  } else if (!lead_zvs && !lag_zvs) {
    me->zvs_state = PsfbZvsState_AllLost;
    me->zvs_margin_pu = 0.0f;
  } else if (!lead_zvs) {
    me->zvs_state = PsfbZvsState_LeadingLost;
    // 超前腿丢失: 大多数情况不会发生 (输出电感能量大)
    me->zvs_margin_pu = 0.2f;
  } else {
    me->zvs_state = PsfbZvsState_LaggingLost;
    // 滞后腿丢失: 轻载时典型问题
    // 裕量 = 电流相对于 ZVS 最小电流的比例
    float margin = i_lag_abs / i_zvs_min;
    if (margin > 1.0f)
      margin = 1.0f;
    me->zvs_margin_pu = margin * 0.8f;  // 上限 0.8 (滞后腿天然裕量较低)
  }

  return me->zvs_state;
}

// ======== 占空比丢失补偿 (来源: TI controlSUITE PWMDRV_PSFB) ========
//
// 原因: 谐振电感 + 变压器漏感导致 di/dt 有限,
//       在电流换向期间有效占空比丢失
//
//   D_eff = D_cmd - D_loss
//   D_loss ≈ (2 * L_res * I_load * Fsw) / (n * V_in)
//          = duty_loss_comp * I_load  (简化线性模型)
//
// 补偿: D_cmd = D_target + D_loss
//   使有效占空比达到目标值
float pwm_fb_duty_loss_compensate(PwmFullBridge *me, float duty_target, float i_load) {
  if (me->duty_loss_comp <= 0.0f) {
    return duty_target;  // 补偿未配置, 直通
  }

  // 占空比丢失 = comp * I_load (线性模型)
  float i_abs = (i_load < 0.0f) ? -i_load : i_load;
  float d_loss = me->duty_loss_comp * i_abs;

  // 补偿后的命令占空比
  float d_cmd = duty_target + d_loss;

  // 硬件限幅 (含补偿后, 不能超过物理最大值)
  float d_max = me->base.duty_max - d_loss;  // 确保补偿后不超过 duty_max
  if (d_max < me->base.duty_min)
    d_max = me->base.duty_min;
  if (d_cmd > me->base.duty_max)
    d_cmd = me->base.duty_max;
  if (d_cmd < me->base.duty_min)
    d_cmd = me->base.duty_min;

  return d_cmd;
}
