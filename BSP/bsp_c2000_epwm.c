// BSP PWM C2000 后端 — bsp_pwm.h 的 TMS320F28004x 实现 (driverlib ePWM)
//
// BspPwmTimer → ePWM 模块: BSP_TIMER_A..F = EPWM1..EPWM6 (与 bsp_pwm.h 注释一致).
// C2000 无 HAL 句柄对象 → handle 忽略, 模块基址由 timer 枚举推导.
//
// 前提: SysConfig board.c 已完成 ePWM 引脚 mux + 系统时钟配置 (本层不碰 GPIO).
// 本层只做 ePWM 模块配置: 时基/CMP/AQ/DB/TZ + ISR 热路径占空比更新.
//
// 语义约定 (与 bsp_hrtim.c / Device 层一致):
//   period = 完整开关周期计数 (clk_hz / freq_hz)
//   边沿对齐: TBPRD=period, 计数 UP;      CMPA=cmp1=period*duty,   AQ: A 在 TB=0 高 / CMPA up 低
//   中心对齐: TBPRD=period/2, 计数 UP_DOWN; CMPA=cmp1=half*(1-duty), AQ: A 在 CMPA up 高 / down 低
//             高宽度 = period - 2*CMPA = period*duty ✓ (与 HRTIM CMP1/CMP3 语义等价)
//   deadtime: 纳秒 → tick (ePWM 时钟 = SYSCLK 无分频, 1 tick = 1e9/clk_hz ns)
//
// 急停: TZ OST 锁存强制输出低; bsp_start 清 OST flag 恢复.
// 停止: AQ 一次性软件强制低 (下个周期边界自动恢复).

#include "bsp_pwm.h"
#include "driverlib.h"

// ======== 模块级状态 ========

static uint32_t s_epwm_clk_hz;  // SYSCLK (bsp_init 注入), 用于 ns→tick / 频率换算

// ======== 内部辅助 ========

// BspPwmTimer → ePWM 基址; 越界返回 0 (拒绝操作)
static uint32_t epwm_base(BspPwmTimer timer) {
  static const uint32_t base[BSP_TIMER_COUNT] = {EPWM1_BASE, EPWM2_BASE, EPWM3_BASE,
                                                 EPWM4_BASE, EPWM5_BASE, EPWM6_BASE};
  if ((uint32_t) timer >= BSP_TIMER_COUNT)
    return 0u;
  return base[(uint32_t) timer];
}

// 死区 ns → tick (DBRED/DBFED 为 14 位, 钳位 0x3FFF)
static uint32_t db_tick_from_ns(uint32_t ns) {
  if (s_epwm_clk_hz == 0u)
    return 0u;
  uint64_t t = ((uint64_t) ns * s_epwm_clk_hz) / 1000000000u;
  return (t > 0x3FFFu) ? 0x3FFFu : (uint32_t) t;
}

// 中心对齐检测: 配置时中心对齐会写 CMPB=cmp3≠0, 边沿对齐写 cmp3=0
static bool is_center(uint32_t base) {
  return EPWM_getCounterCompareValue(base, EPWM_COUNTER_COMPARE_B) != 0u;
}

// 死区生成器: 互补时 EPWMB = DB(EPWMA) 派生互补, 禁用时 EPWMB = AQ B = 低 (DEM).
// IN_MODE 决定旁路输出源: 互补=EPWMA (DB 派生), DEM=EPWMB (原始 AQ B 恒低).
// 若 DEM 下仍选 EPWMA, OUT_MODE 旁路会让 EPWMB 镜像高侧 → 上下管同开直通.
static void db_config(uint32_t base, bool enable, uint32_t rise_ns, uint32_t fall_ns) {
  EPWM_setRisingEdgeDeadBandDelayInput(base, enable ? EPWM_DB_INPUT_EPWMA : EPWM_DB_INPUT_EPWMB);
  EPWM_setFallingEdgeDeadBandDelayInput(base, enable ? EPWM_DB_INPUT_EPWMA : EPWM_DB_INPUT_EPWMB);
  EPWM_setDeadBandDelayMode(base, EPWM_DB_RED, enable);
  EPWM_setDeadBandDelayMode(base, EPWM_DB_FED, enable);
  if (!enable)
    return;
  // 学自 TI buck_hal: RED 高有效 / FED 低有效 → 互补低侧管两侧插入死区
  EPWM_setDeadBandDelayPolarity(base, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
  EPWM_setDeadBandDelayPolarity(base, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
  EPWM_setRisingEdgeDelayCount(base, db_tick_from_ns(rise_ns));
  EPWM_setFallingEdgeDelayCount(base, db_tick_from_ns(fall_ns));
}

// Action Qualifier: A 按对齐方式输出占空比, B 恒低 (DEM 基准, 互补时被 DB 覆盖)
static void aq_config(uint32_t base, bool center) {
  EPWM_setActionQualifierShadowLoadMode(base, EPWM_ACTION_QUALIFIER_A, EPWM_AQ_LOAD_ON_CNTR_PERIOD);
  EPWM_setActionQualifierShadowLoadMode(base, EPWM_ACTION_QUALIFIER_B, EPWM_AQ_LOAD_ON_CNTR_PERIOD);
  if (center) {
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
  } else {
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
  }
  EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
}

// ======== 生命周期 ========

void bsp_init(BspPwmConfig *cfg) {
  if (!cfg)
    return;
  s_epwm_clk_hz = cfg->clk_hz;
  // C2000 ePWM 时钟常开, 无需外设时钟门控; GPIO mux 归 SysConfig board_init
}

// ======== 下层 API (寄存器级) ========

void bsp_config_timer(BspPwmHandle *h, const BspPwmTimerConfig *tcfg) {
  (void) h;
  if (!tcfg)
    return;
  uint32_t base = epwm_base(tcfg->timer);
  if (!base)
    return;

  bool center = (tcfg->cmp3 != 0u);  // Device 层约定: 边沿 cmp3=0, 中心 cmp3=half*(1+duty)
  uint32_t period = center ? (tcfg->period / 2u) : tcfg->period;

  // 时基: 无分频 (ePWM 时钟 = SYSCLK, 死区分辨率最大化)
  EPWM_setClockPrescaler(base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
  EPWM_setTimeBaseCounter(base, 0);
  EPWM_setTimeBasePeriod(base, period);
  EPWM_setPeriodLoadMode(base, EPWM_PERIOD_SHADOW_LOAD);
  EPWM_setTimeBaseCounterMode(base, center ? EPWM_COUNTER_MODE_UP_DOWN : EPWM_COUNTER_MODE_UP);
  EPWM_disablePhaseShiftLoad(base);

  // CMPA = 占空比沿, CMPB = 中心参考 (预留 ADC 同步点)
  EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A, tcfg->cmp1);
  EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_B, tcfg->cmp3);
  EPWM_setCounterCompareShadowLoadMode(base, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_PERIOD);
  EPWM_setCounterCompareShadowLoadMode(base, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_PERIOD);

  aq_config(base, center);
  db_config(base, tcfg->complementary, tcfg->deadtime_rising, tcfg->deadtime_falling);
}

void bsp_start(BspPwmHandle *h, uint32_t timer_mask, uint32_t output_mask) {
  (void) output_mask;
  for (uint32_t t = 0u; t < BSP_TIMER_COUNT; t++) {
    if (!(timer_mask & (1u << t)))
      continue;
    uint32_t base = epwm_base((BspPwmTimer) t);
    if (!base)
      continue;
    // 清 TZ flag 恢复输出 (急停 OST 锁存后), 还原计数模式 (bsp_stop 冻结过), 重置计数起点
    EPWM_clearTripZoneFlag(base, EPWM_TZ_FLAG_OST);
    EPWM_clearTripZoneFlag(base, EPWM_TZ_FLAG_CBC);
    EPWM_setTimeBaseCounterMode(base, is_center(base) ? EPWM_COUNTER_MODE_UP_DOWN
                                                       : EPWM_COUNTER_MODE_UP);
    EPWM_setTimeBaseCounter(base, 0);
  }
}

void bsp_stop(BspPwmHandle *h, uint32_t timer_mask, uint32_t output_mask) {
  (void) output_mask;
  for (uint32_t t = 0u; t < BSP_TIMER_COUNT; t++) {
    if (!(timer_mask & (1u << t)))
      continue;
    uint32_t base = epwm_base((BspPwmTimer) t);
    if (!base)
      continue;
    // 封输出: AQ 一次性软件强制 A/B 低 + 冻结计数器 → 无后续周期边界, 强制保持 (graceful stop)
    // 注意: 冻结时 AQ 强制停在当前电平; 互补通道 EPWMB 会停在高电平 (续流), 反灌不阻断
    // (与 HRTIM ODISR 语义有差异: C2000 无单边输出使能, 需 TZ 才真封 — 见 bsp_emergency_stop)
    EPWM_setActionQualifierSWAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW);
    EPWM_setActionQualifierSWAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW);
    EPWM_forceActionQualifierSWAction(base, EPWM_AQ_OUTPUT_A);
    EPWM_forceActionQualifierSWAction(base, EPWM_AQ_OUTPUT_B);
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_STOP_FREEZE);
  }
}

void bsp_update_duty(BspPwmHandle *h, BspPwmTimer timer, uint32_t cmp1, uint32_t cmp3) {
  (void) h;
  uint32_t base = epwm_base(timer);
  if (!base)
    return;
  // CMPA/CMPB 阴影装载, 周期边界生效 → ISR 安全
  EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A, cmp1);
  EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_B, cmp3);
}

void bsp_update_period(BspPwmHandle *h, BspPwmTimer timer, uint32_t period) {
  (void) h;
  uint32_t base = epwm_base(timer);
  if (!base)
    return;
  EPWM_setTimeBasePeriod(base, is_center(base) ? period / 2u : period);
}

void bsp_update_deadtime(BspPwmHandle *h, BspPwmTimer timer, uint32_t rising_tick, uint32_t falling_tick) {
  (void) h;
  uint32_t base = epwm_base(timer);
  if (!base)
    return;
  // Device 层传"纳秒", 此处统一 ns→tick 换算
  db_config(base, true, rising_tick, falling_tick);
}

void bsp_emergency_stop(BspPwmHandle *h, uint32_t output_mask) {
  (void) h;
  for (uint32_t t = 0u; t < BSP_TIMER_COUNT; t++) {
    if (!(output_mask & (0x3u << (2u * t))))  // 该模块任一输出
      continue;
    uint32_t base = epwm_base((BspPwmTimer) t);
    if (!base)
      continue;
    // TZ OST 锁存强制 A/B 输出低 (直写故障路径, 与 HRTIM ODISR 语义一致)
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);
    EPWM_forceTripZoneEvent(base, EPWM_TZ_FORCE_EVENT_OST);
  }
}

// ======== 上层 API (物理参数) ========

// 占空比 → CMPA/CMPB (与 Device 层 bb_set_duty 公式一致)
static void duty_to_cmp(float duty, uint32_t period, bool center, uint32_t *cmp1, uint32_t *cmp3) {
  if (duty < 0.0f)
    duty = 0.0f;
  if (duty > 1.0f)
    duty = 1.0f;
  if (center) {
    *cmp1 = (uint32_t) ((float) period * (1.0f - duty) / 2.0f);
    *cmp3 = (uint32_t) ((float) period * (1.0f + duty) / 2.0f);
  } else {
    *cmp1 = (uint32_t) ((float) period * duty);
    *cmp3 = 0u;
  }
}

void bsp_pwm_config_ch(BspPwmHandle *h, const BspPwmChCfg *cfg) {
  (void) h;
  if (!cfg || s_epwm_clk_hz == 0u)
    return;

  uint32_t period = s_epwm_clk_hz / cfg->freq_hz;
  bool center = (cfg->align == BSP_PWM_CENTER_ALIGNED);
  uint32_t cmp1, cmp3;
  duty_to_cmp(cfg->duty, period, center, &cmp1, &cmp3);

  BspPwmTimerConfig tcfg = {
      .timer = cfg->timer,
      .period = period,
      .cmp1 = cmp1,
      .cmp3 = cmp3,
      .deadtime_rising = cfg->deadtime_ns,
      .deadtime_falling = cfg->deadtime_ns,
      .output_mask = cfg->output_mask,
      .complementary = cfg->complementary,
  };
  bsp_config_timer(h, &tcfg);

  if (cfg->phase_deg > 0.0f)
    bsp_set_phase_shift(h, cfg->timer, cfg->phase_deg);
}

void bsp_pwm_set_duty_f(BspPwmHandle *h, BspPwmTimer timer, float duty) {
  (void) h;
  uint32_t base = epwm_base(timer);
  if (!base)
    return;
  // TBPRD 在中心对齐时为半周期, 还原完整周期再换算 (duty_to_cmp 以完整周期为基准)
  uint32_t tbprd = EPWM_getTimeBasePeriod(base);
  bool center = is_center(base);
  uint32_t full = center ? tbprd * 2u : tbprd;
  uint32_t cmp1, cmp3;
  duty_to_cmp(duty, full, center, &cmp1, &cmp3);
  bsp_update_duty(h, timer, cmp1, cmp3);
}

void bsp_pwm_set_freq_hz(BspPwmHandle *h, BspPwmTimer timer, uint32_t freq_hz) {
  if (freq_hz == 0u || s_epwm_clk_hz == 0u)
    return;
  bsp_update_period(h, timer, s_epwm_clk_hz / freq_hz);
}

void bsp_pwm_set_deadtime_ns(BspPwmHandle *h, BspPwmTimer timer, uint32_t ns) {
  if (timer >= BSP_TIMER_COUNT)
    return;
  bsp_update_deadtime(h, timer, ns, ns);
}

void bsp_pwm_set_phase_deg(BspPwmHandle *h, BspPwmTimer timer, float phase_deg) {
  bsp_set_phase_shift(h, timer, phase_deg);
}

// ======== 通用操作 ========

void bsp_set_phase_shift(BspPwmHandle *h, BspPwmTimer timer, float phase_deg) {
  (void) h;
  uint32_t base = epwm_base(timer);
  if (!base)
    return;
  // 相移 = 完整开关周期 * deg/360; 中心对齐时完整周期 = 2*TBPRD
  uint32_t period = EPWM_getTimeBasePeriod(base);
  uint32_t full = is_center(base) ? period * 2u : period;
  uint32_t tbp = (uint32_t) ((float) full * phase_deg / 360.0f);
  if (tbp > 0xFFFFu)  // TBPHS 16 位, 中心对齐 full=2*TBPRD 在 >180° 时钳位
    tbp = 0xFFFFu;
  // F28004x 同步级联固定 (EPWM1 SYNCOUT → EPWM2..6 SYNCIN, 无需显式配置, 学自 SDK cla_ex7).
  // 主模块 (EPWM1) 计数为零时发同步脉冲, 从模块在 SYNCIN 处装载 TBPHS.
  // 无条件配置: 即使 shift 的是 timer_b (ePWM2), EPWM1 也要发脉冲, 否则相移静默失效
  EPWM_setSyncOutPulseMode(EPWM1_BASE, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);
  if (timer != BSP_TIMER_A) {  // 主模块自身不装载 (它是同步基准)
    EPWM_setPhaseShift(base, (uint16_t) tbp);
    EPWM_enablePhaseShiftLoad(base);
    // 一次性同步: 立即装载 TBPHS, 不等主模块下个周期 (学自 cla_ex7 两模块对齐)
    EPWM_enableOneShotSync(base);
  }
}

void bsp_pwm_set_complementary(BspPwmHandle *h, BspPwmTimer timer, bool enable) {
  (void) h;
  uint32_t base = epwm_base(timer);
  if (!base)
    return;
  // enable: 恢复互补 (DB 派生 EPWMB, IN_MODE=EPWMA, OUT_MODE 生效).
  // disable (DEM): 必须把 IN_MODE 切到 EPWMB (原始 AQ B 恒低) 再清 OUT_MODE —
  //   否则 OUT_MODE 旁路会让 EPWMB 镜像 EPWMA (高侧), 上下管同开 → 直通.
  // 死区计数值/极性寄存器保留, 重开即恢复 (轻量, ISR 安全)
  EPWM_setRisingEdgeDeadBandDelayInput(base, enable ? EPWM_DB_INPUT_EPWMA : EPWM_DB_INPUT_EPWMB);
  EPWM_setFallingEdgeDeadBandDelayInput(base, enable ? EPWM_DB_INPUT_EPWMA : EPWM_DB_INPUT_EPWMB);
  EPWM_setDeadBandDelayMode(base, EPWM_DB_RED, enable);
  EPWM_setDeadBandDelayMode(base, EPWM_DB_FED, enable);
}

void bsp_pwm_isr(BspPwmHandle *h) {
  // ePWM ISR 由 App 直接调 App_OnControlTick; 此处无后端专属事务
  (void) h;
}
