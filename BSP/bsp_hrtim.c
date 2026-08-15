// BSP PWM HRTIM 后端 — bsp_pwm.h 的 STM32 实现 (F334/F3/G4/H7 通用寄存器级)
//
// bsp_pwm.c (旧 API) → 新 API (bsp_pwm.h):
//   bsp_init / bsp_config_timer / bsp_start / bsp_stop
//   bsp_update_duty / bsp_update_period / bsp_update_deadtime / bsp_emergency_stop
//   + 上层 API: bsp_pwm_config_ch / bsp_pwm_set_duty_f / bsp_pwm_set_freq_hz ...
//
// 前提: CubeMX MX_HRTIM1_Init() 已完成时基/极性/GPIO/NVIC 配置.
// 本层只做 ISR 热路径操作: 直写 CMP1xR/CMP3xR/PERxR/DTxR + 启停输出 + 急停.
//
// 死区转换: F334 HCLK=72MHz, CubeMX DTG 分频 → f_DTG=9MHz, 1 tick≈111ns
//   (旧代码 `dt_ns * 9 / 1000` 硬件验证有效)
//
// 注意: 寄存器级 API 的 deadtime 入参是"纳秒" (Device 层直接传 deadtime_ns),
//       与本头文件注释 "BSP 内部 tick" 不同 — 本文件内做 ns→tick 换算.

#include "bsp_pwm.h"
#include "bsp_stm32_hal.h"

// ======== 内部辅助 ========

// 相位交错内部 (定义在文件尾部交错 section; 前向声明供 bsp_update_period 变频后重算锚点)
static void hrtim_ilv_resync(HRTIM_TypeDef *inst);

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

// Device 侧 timer_mask = 1<<BSP_TIMER (bit = 定时器序号, A=bit0..F=bit5); HRTIM 计数使能位
// 集中在 MCR (MCEN=bit16, TACEN..TFCEN=bit17..22). 翻译 bit0-5 → bit17-22, 并恒开主定时器
// 作相位基准 (bsp_update_period 已写 MPER, 主定时器应始终随子定时器运行).
static uint32_t hrtim_count_mask(uint32_t bsp_mask) {
  return ((bsp_mask & 0x3Fu) << 17u) | HRTIM_MCR_MCEN;
}

void bsp_start(BspPwmHandle *h, uint32_t timer_mask, uint32_t output_mask) {
  HRTIM_HandleTypeDef *hh = (HRTIM_HandleTypeDef *) h;
  if (!hh)
    return;
  // 主定时器随子定时器同一 MCR 写启动 → 全部从 0 锁步 (交错锚点基准, 见 bsp_set_phase_shift)
  HAL_HRTIM_WaveformCountStart(hh, hrtim_count_mask(timer_mask));
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
  HAL_HRTIM_WaveformCountStop(hh, hrtim_count_mask(timer_mask));
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
  hrtim_ilv_resync(hh->Instance);  // 变频后重算已接线的 MSTCMPn (保持相位角, 见交错 section)
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

// 相位交错 (CMP2 主复位连线)
//
// F334 HRTIM 子定时器复位源 = {自身 UPDATE/CMP2/CMP4, 主定时器 MSTPER/MSTCMP1-4,
// EEV1-10, TIMB/C/D/E CMP1/2/4} — 没有 TIMACMP: Timer A 的 CMP 不能复位其它定时器
// (G4 才支持 TIMACMP 级联). 因此跨定时器真交错唯一通用路径 = 主定时器 CMP 锚点:
//   phase 0 定时器: 不接线 — 自由运行 (PERxR 回绕) 且与主定时器同起同频 → 天然对齐
//   phase>0 定时器: RSTxR = MSTCMPn, MCMPnR = MPER × phase_deg/360
//     (F334 无 up-down 计数, 边沿对齐下周期 = PER tick, 无 ×2)
// 主定时器在 bsp_start 随子定时器同时启动 (MCEN), 保证全部从 0 锁步.
// 槽分配: 每实例 4 个 master CMP 槽, 按相位去重 (同相两腿共用一槽).
#define BSP_HRTIM_ILV_INST_MAX 2  // G474 双 HRTIM (HRTIM1/HRTIM2)
#define BSP_HRTIM_ILV_SLOT_MAX 4  // MSTCMP1..4
typedef struct {
  HRTIM_TypeDef *inst;                     // NULL = 空闲
  float slot_deg[BSP_HRTIM_ILV_SLOT_MAX];  // 槽 → 相位 (度), <0 = 空闲
} BspHrtimIlv;
static BspHrtimIlv s_ilv[BSP_HRTIM_ILV_INST_MAX];

// MSTCMPn 寄存器写 (slot 0..3 = MCMP1R..4R)
static void hrtim_mcmp_write(HRTIM_TypeDef *hr, int slot, uint32_t mcmp) {
  switch (slot) {
  case 0:
    hr->sMasterRegs.MCMP1R = mcmp;
    break;
  case 1:
    hr->sMasterRegs.MCMP2R = mcmp;
    break;
  case 2:
    hr->sMasterRegs.MCMP3R = mcmp;
    break;
  default:
    hr->sMasterRegs.MCMP4R = mcmp;
    break;
  }
}

// 定位实例状态 (首个空闲分配; 超 2 实例返回 NULL)
static BspHrtimIlv *hrtim_ilv_find(HRTIM_TypeDef *inst) {
  for (int i = 0; i < BSP_HRTIM_ILV_INST_MAX; i++) {
    if (s_ilv[i].inst == inst)
      return &s_ilv[i];
  }
  for (int i = 0; i < BSP_HRTIM_ILV_INST_MAX; i++) {
    if (s_ilv[i].inst == NULL) {
      s_ilv[i].inst = inst;
      for (int j = 0; j < BSP_HRTIM_ILV_SLOT_MAX; j++)
        s_ilv[i].slot_deg[j] = -1.0f;
      return &s_ilv[i];
    }
  }
  return NULL;
}

// 只查实例状态 (不分配) — resync 查询路径用, 避免误占槽位
static BspHrtimIlv *hrtim_ilv_lookup(HRTIM_TypeDef *inst) {
  for (int i = 0; i < BSP_HRTIM_ILV_INST_MAX; i++) {
    if (s_ilv[i].inst == inst)
      return &s_ilv[i];
  }
  return NULL;
}

// phase_deg → master CMP 槽 (0..3 = MSTCMP1..4); 同相位 (就近取整识别) 复用, 槽满接管最后槽.
// 槽相位一律按整度规范化存储 — 去重键与后续 mcmp 计算同源 (同相两腿共用一槽不漂移).
static int hrtim_ilv_slot(BspHrtimIlv *ctx, float phase_deg) {
  uint32_t deg = (uint32_t) (phase_deg + 0.5f);
  for (int i = 0; i < BSP_HRTIM_ILV_SLOT_MAX; i++) {
    if (ctx->slot_deg[i] >= 0.0f && (uint32_t) (ctx->slot_deg[i] + 0.5f) == deg)
      return i;
  }
  for (int i = 0; i < BSP_HRTIM_ILV_SLOT_MAX; i++) {
    if (ctx->slot_deg[i] < 0.0f) {
      ctx->slot_deg[i] = (float) deg;  // 规范化整度 → 槽键与 mcmp 同源
      return i;
    }
  }
  ctx->slot_deg[BSP_HRTIM_ILV_SLOT_MAX - 1] = (float) deg;  // 槽满: 最后槽接管新相位
  return BSP_HRTIM_ILV_SLOT_MAX - 1;
}

void bsp_set_phase_shift(BspPwmHandle *h, BspPwmTimer timer, float phase_deg) {
  HRTIM_HandleTypeDef *hh = (HRTIM_HandleTypeDef *) h;
  if (!hh || !timer_valid(timer))
    return;
  // 真交错: phase>0 定时器在主定时器 CMP 事件上复位 (相位差 = MCMP/MPER×360°).
  // phase 0 定时器不接线 (Device 侧也跳过 0°) — 自由运行即基准; 负角非法 (负 float→uint 转换 UB).
  if (phase_deg <= 0.0f)
    return;
  uint32_t period = period_of(hh, timer);
  hh->Instance->sTimerxRegs[hrtim_idx(timer)].CMP2xR = (uint32_t) ((float) period * phase_deg / 360.0f);
  HRTIM_TypeDef *hr = hh->Instance;
  uint32_t mper = hr->sMasterRegs.MPER;
  if (mper == 0u)
    return;  // 周期未设置 (App 须先 set_freq 再 start)
  BspHrtimIlv *ctx = hrtim_ilv_find(hr);
  if (ctx == NULL)
    return;
  int slot = hrtim_ilv_slot(ctx, phase_deg);
  uint32_t mcmp = (uint32_t) ((float) mper * ctx->slot_deg[slot] / 360.0f);
  hrtim_mcmp_write(hr, slot, mcmp);
  // RSTxR 位布局 (F334 RM0330 与 G474 RM0440 一致, CMSIS 权威 HRTIM_RSTR_*):
  //   bit4=MSTPER, bit5..8=MSTCMP1..4, bit9+=EEV/跨定时器 — 槽 0..3 映射 MSTCMP(1+n)
  // ⚠ 勿套用 RSTx1R/RSTx2R (输出复位寄存器) 的 bit8 起步布局 — 计数器复位必须用 RSTxR/bit5
  uint32_t rst = hr->sTimerxRegs[hrtim_idx(timer)].RSTxR & ~0x1FFu;
  hr->sTimerxRegs[hrtim_idx(timer)].RSTxR = rst | (1u << (HRTIM_RSTR_MSTCMP1_Pos + (uint32_t) slot));
}

// 变频后重算已接线的 MSTCMPn — 相位角不变 (MCMP = MPER × 槽相位/360), 避免运行中角漂移
static void hrtim_ilv_resync(HRTIM_TypeDef *inst) {
  BspHrtimIlv *ctx = hrtim_ilv_lookup(inst);
  if (ctx == NULL)
    return;
  uint32_t mper = inst->sMasterRegs.MPER;
  for (int i = 0; i < BSP_HRTIM_ILV_SLOT_MAX; i++) {
    if (ctx->slot_deg[i] < 0.0f)
      continue;
    uint32_t mcmp = (uint32_t) ((float) mper * ctx->slot_deg[i] / 360.0f);
    hrtim_mcmp_write(inst, i, mcmp);
  }
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
