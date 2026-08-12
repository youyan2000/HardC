// BSP PWM HRTIM 后端 — bsp_pwm.h 的 STM32 实现 (F334/F3/G4/H7 通用寄存器级)
//
// 移植自 WEILAI SuperCap bsp_pwm.c (旧 API) → 新 API (bsp_pwm.h):
//   bsp_init / bsp_config_timer / bsp_start / bsp_stop
//   bsp_update_duty / bsp_update_period / bsp_update_deadtime / bsp_emergency_stop
//   + 上层 API: bsp_pwm_config_ch / bsp_pwm_set_duty_f / bsp_pwm_set_freq_hz ...
//
// 前提: CubeMX MX_HRTIM1_Init() 已完成时基/极性/GPIO/NVIC 配置.
// 本层只做 ISR 热路径操作: 直写 CMP1xR/CMP3xR/PERxR/DTxR + 启停输出 + 急停.
//
// 死区转换: F334 HCLK=72MHz, CubeMX DTG 分频 → f_DTG=9MHz, 1 tick≈111ns
//   (旧代码 `dt_ns * 9 / 1000` 在 WEILAI 硬件验证有效)
//
// 注意: 寄存器级 API 的 deadtime 入参是"纳秒" (Device 层直接传 deadtime_ns),
//       与本头文件注释 "BSP 内部 tick" 不同 — 本文件内做 ns→tick 换算.

#include "bsp_pwm.h"
#include "bsp_stm32_hal.h"

// ======== 内部辅助 ========

// BspPwmTimer → HRTIM_TIMERINDEX. F334 仅 A..E (无 F); 越界返回 0xFF (COMMON) 并拒写.
static inline uint32_t hrtim_idx(BspPwmTimer timer) {
  return (uint32_t) timer;  // BSP_TIMER_A=0..E=4 == HRTIM_TIMERINDEX_TIMER_A..E
}

static inline bool timer_valid(BspPwmTimer timer) {
  return (uint32_t) timer <= HRTIM_TIMERINDEX_TIMER_E;  // 0..4, F334 无 Timer F
}

// 死区 ns → DTG tick (f_DTG = 9MHz, 钳位 0x1FF)
static inline uint32_t dt_tick_from_ns(uint32_t ns) {
  uint32_t t = (uint32_t) ((uint64_t) ns * 9u / 1000u);
  return (t > 0x1FFu) ? 0x1FFu : t;
}

// 写入单定时器死区寄存器 (保留未用位, 只改 DTR/DTF 字段)
static void dt_write(HRTIM_HandleTypeDef *h, BspPwmTimer timer, uint32_t rising_ns, uint32_t falling_ns) {
  if (!timer_valid(timer))
    return;
  uint32_t dt_r = dt_tick_from_ns(rising_ns);
  uint32_t dt_f = dt_tick_from_ns(falling_ns);
  uint32_t idx = hrtim_idx(timer);
  uint32_t dtr = h->Instance->sTimerxRegs[idx].DTxR;
  dtr &= ~(0x1FFu | (0x1FFu << 16));  // 清 DTR[8:0] + DTF[24:16]
  dtr |= (dt_r << 0) | (dt_f << 16);
  h->Instance->sTimerxRegs[idx].DTxR = dtr;
}

// ======== 生命周期 ========

void bsp_init(BspPwmConfig *cfg) {
  if (!cfg || !cfg->handle)
    return;
  // F334: 无 DLL; use_dll 仅 G4 有意义. clk_hz 由调用方填入 (App 层注入).
  (void) cfg->use_dll;
}

// ======== 下层 API (寄存器级) ========

void bsp_config_timer(BspPwmHandle *h, const BspPwmTimerConfig *tcfg) {
  HRTIM_HandleTypeDef *hh = (HRTIM_HandleTypeDef *) h;
  if (!hh || !tcfg || !timer_valid(tcfg->timer))
    return;

  uint32_t idx = hrtim_idx(tcfg->timer);
  hh->Instance->sTimerxRegs[idx].CMP1xR = tcfg->cmp1;
  hh->Instance->sTimerxRegs[idx].CMP3xR = tcfg->cmp3;

  // 互补输出时写入死区 (Device 层传入 ns)
  if (tcfg->complementary) {
    dt_write(hh, tcfg->timer, tcfg->deadtime_rising, tcfg->deadtime_falling);
  }
}

void bsp_start(BspPwmHandle *h, uint32_t timer_mask, uint32_t output_mask) {
  HRTIM_HandleTypeDef *hh = (HRTIM_HandleTypeDef *) h;
  if (!hh)
    return;
  // 先启计数器 → 等波形稳定 → 再开输出 (WEILAI 模式)
  HAL_HRTIM_WaveformCountStart(hh, timer_mask);
  for (volatile uint32_t i = 0; i < 100u; i++) { /* 等待 ≈1 个周期 */
  }
  HAL_HRTIM_WaveformOutputStart(hh, output_mask);
}

void bsp_stop(BspPwmHandle *h, uint32_t timer_mask, uint32_t output_mask) {
  HRTIM_HandleTypeDef *hh = (HRTIM_HandleTypeDef *) h;
  if (!hh)
    return;
  // 先封输出, 再停计数器 (防止关断瞬间电平不确定)
  HAL_HRTIM_WaveformOutputStop(hh, output_mask);
  HAL_HRTIM_WaveformCountStop(hh, timer_mask);
}

void bsp_update_duty(BspPwmHandle *h, BspPwmTimer timer, uint32_t cmp1, uint32_t cmp3) {
  HRTIM_HandleTypeDef *hh = (HRTIM_HandleTypeDef *) h;
  if (!hh || !timer_valid(timer))
    return;
  uint32_t idx = hrtim_idx(timer);
  hh->Instance->sTimerxRegs[idx].CMP1xR = cmp1;
  hh->Instance->sTimerxRegs[idx].CMP3xR = cmp3;
}

void bsp_update_period(BspPwmHandle *h, BspPwmTimer timer, uint32_t period) {
  HRTIM_HandleTypeDef *hh = (HRTIM_HandleTypeDef *) h;
  if (!hh || !timer_valid(timer))
    return;
  uint32_t idx = hrtim_idx(timer);
  // Master 时基 (同步基准) + 本定时器周期
  hh->Instance->sMasterRegs.MPER = period;
  hh->Instance->sTimerxRegs[idx].PERxR = period;
}

void bsp_update_deadtime(BspPwmHandle *h, BspPwmTimer timer, uint32_t rising_tick, uint32_t falling_tick) {
  HRTIM_HandleTypeDef *hh = (HRTIM_HandleTypeDef *) h;
  if (!hh || !timer_valid(timer))
    return;
  // Device 层传的是"纳秒"; 此处统一 ns→tick 换算
  dt_write(hh, timer, rising_tick, falling_tick);
}

void bsp_emergency_stop(BspPwmHandle *h, uint32_t output_mask) {
  HRTIM_HandleTypeDef *hh = (HRTIM_HandleTypeDef *) h;
  if (!hh)
    return;
  // ODISR 置位 → 输出立即强制 inactive (急停, 与 HAL_HRTIM_WaveformOutputStop 同路径;
  // 清 OENR 无效 — HRTIM 输出使能是 set/reset 寄存器对, 禁用必须置 ODISR)
  hh->Instance->sCommonRegs.ODISR |= output_mask;
}

// ======== 上层 API (物理参数) ========

// 从当前 PERxR 读周期 (中心对齐/边沿对齐公用)
static uint32_t period_of(HRTIM_HandleTypeDef *hh, BspPwmTimer timer) {
  if (!timer_valid(timer))
    return 0u;
  return hh->Instance->sTimerxRegs[hrtim_idx(timer)].PERxR;
}

// 占空比 → CMP1/CMP3 (与 Device 层 bb_set_duty 公式一致)
static void duty_to_cmp(float duty, uint32_t period, bool center, uint32_t *cmp1, uint32_t *cmp3) {
  if (duty < 0.0f)
    duty = 0.0f;
  if (duty > 1.0f)
    duty = 1.0f;
  if (center) {
    uint32_t half = period / 2u;
    *cmp1 = (uint32_t) ((float) half * (1.0f - duty));
    *cmp3 = (uint32_t) ((float) half * (1.0f + duty));
  } else {
    *cmp1 = (uint32_t) ((float) period * duty);
    *cmp3 = 0u;
  }
}

void bsp_pwm_config_ch(BspPwmHandle *h, const BspPwmChCfg *cfg) {
  HRTIM_HandleTypeDef *hh = (HRTIM_HandleTypeDef *) h;
  if (!hh || !cfg || !timer_valid(cfg->timer))
    return;

  // 周期: F334 HRTIM 时钟 = HCLK = SystemCoreClock
  uint32_t period = SystemCoreClock / cfg->freq_hz;
  uint32_t cmp1, cmp3;
  duty_to_cmp(cfg->duty, period, (cfg->align == BSP_PWM_CENTER_ALIGNED), &cmp1, &cmp3);

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

  // 相位偏移 → CMP2 (全桥移相)
  if (cfg->phase_deg > 0.0f) {
    uint32_t idx = hrtim_idx(cfg->timer);
    hh->Instance->sTimerxRegs[idx].CMP2xR = (uint32_t) ((float) period * cfg->phase_deg / 360.0f);
  }
}

void bsp_pwm_set_duty_f(BspPwmHandle *h, BspPwmTimer timer, float duty) {
  HRTIM_HandleTypeDef *hh = (HRTIM_HandleTypeDef *) h;
  if (!hh || !timer_valid(timer))
    return;
  uint32_t period = period_of(hh, timer);
  uint32_t cmp1, cmp3;
  // 中心对齐检测: 配置时中心对齐会写 CMP3 = half*(1+duty) ≠ 0, 边沿对齐写 CMP3 = 0
  bool center = (hh->Instance->sTimerxRegs[hrtim_idx(timer)].CMP3xR != 0u);
  duty_to_cmp(duty, period, center, &cmp1, &cmp3);
  bsp_update_duty(h, timer, cmp1, cmp3);
}

void bsp_pwm_set_freq_hz(BspPwmHandle *h, BspPwmTimer timer, uint32_t freq_hz) {
  if (!h || freq_hz == 0u)
    return;
  bsp_update_period(h, timer, SystemCoreClock / freq_hz);
}

void bsp_pwm_set_deadtime_ns(BspPwmHandle *h, BspPwmTimer timer, uint32_t ns) {
  if (!h)
    return;
  bsp_update_deadtime(h, timer, ns, ns);
}

void bsp_pwm_set_phase_deg(BspPwmHandle *h, BspPwmTimer timer, float phase_deg) {
  bsp_set_phase_shift(h, timer, phase_deg);
}

// ======== 通用操作 ========

void bsp_set_phase_shift(BspPwmHandle *h, BspPwmTimer timer, float phase_deg) {
  HRTIM_HandleTypeDef *hh = (HRTIM_HandleTypeDef *) h;
  if (!hh || !timer_valid(timer))
    return;
  uint32_t period = period_of(hh, timer);
  hh->Instance->sTimerxRegs[hrtim_idx(timer)].CMP2xR = (uint32_t) ((float) period * phase_deg / 360.0f);
}

void bsp_pwm_set_complementary(BspPwmHandle *h, BspPwmTimer timer, bool enable) {
  HRTIM_HandleTypeDef *hh = (HRTIM_HandleTypeDef *) h;
  if (!hh || !timer_valid(timer))
    return;
  // OENR 位布局 == BSP_OUT_* 掩码 (TA1=bit0, TA2=bit1, TB1=bit2, TB2=bit3 ...)
  uint32_t pair = (1u << (2u * (uint32_t) timer)) | (1u << (2u * (uint32_t) timer + 1u));
  uint32_t out2 = (1u << (2u * (uint32_t) timer + 1u));  // 互补腿 (低侧/DEM)
  if (enable) {
    hh->Instance->sCommonRegs.OENR |= pair;  // 恢复互补输出
  } else {
    hh->Instance->sCommonRegs.ODISR |= out2;  // 关互补腿 (二极管仿真 DEM; 同急停语义, 必须置 ODISR)
  }
}

void bsp_pwm_isr(BspPwmHandle *h) {
  // F334: HAL_HRTIM_IRQHandler 已处理 (App 层回调 App_OnControlTick);
  // 此处无后端专属事务
  (void) h;
}
