// 无线充电 (WPT) 线圈驱动 —— PwmBase 子类 (单相半桥实现)
//
// 中心对齐对称双沿: CMP1 = period/2 × (1-duty) [SET], CMP3 = period/2 × (1+duty) [RESET].
//   周期由主频派生: period = clk_hz / freq_hz (不缓存, 无独立 period 字段).
//   占空比钳位在基类 (duty_min/duty_max), ops 收到的是已钳位的 duty.
//
// 接线契约: init 阶段不写硬件 (bsp_cfg.handle 为 NULL, clk_hz=0). 所有写硬件
//   路径先 guard clk_hz==0: 未注入句柄时仅更新字段, 不触碰寄存器.
//   deadtime 不持久化 (字段集固定): start 用死区 0, set_deadtime 直接转发 BSP.
//
// 线圈分频 freq_div4: 本层只持有分频值 + 提供 coil_freq 查询; 实际 DMA/CMP
//   周期分频调度是 BSP/工具链 AI 缺口 (见 AGENT-SYNC), 本层不写分频寄存器.

#include "pwm_wpt.h"
#include "container_of.h"
#include <stddef.h>

// ======== 内部辅助 ========

static inline float clamp_f(float d, float lo, float hi) {
  if (d < lo)
    return lo;
  if (d > hi)
    return hi;
  return d;
}

// 半桥腿占空比 → CMP1/CMP3 (中心对齐对称双沿)
static void wpt_leg_cmp(uint32_t period, float duty, uint32_t *cmp1, uint32_t *cmp3) {
  uint32_t half = period / 2;
  *cmp1 = (uint32_t) (half * (1.0f - duty));
  *cmp3 = (uint32_t) (half * (1.0f + duty));
}

// ======== ops 实现 (全部 container_of 下溯) ========

static void wpt_start(PwmBase *base) {
  PwmWpt *me = container_of(base, PwmWpt, base);
  if (me->bsp_cfg.clk_hz == 0u || me->base.freq_hz == 0u) {
    return;  // handle 未注入, 不配置硬件
  }

  uint32_t period = me->bsp_cfg.clk_hz / me->base.freq_hz;
  uint32_t cmp1, cmp3;
  wpt_leg_cmp(period, me->duty, &cmp1, &cmp3);

  BspPwmTimerConfig tcfg = {
      .timer = me->timer,
      .period = period,
      .cmp1 = cmp1,
      .cmp2 = period / 2,  // 中心参考
      .cmp3 = cmp3,
      .deadtime_rising = 0u,  // deadtime 不持久化: App 在 start 后调 set_deadtime
      .deadtime_falling = 0u,
      .output_mask = me->output_mask,
      .complementary = true,
  };

  bsp_config_timer(me->bsp_cfg.handle, &tcfg);
  uint32_t timer_mask = (1u << me->timer);
  bsp_start(me->bsp_cfg.handle, timer_mask, me->output_mask);
}

static void wpt_stop(PwmBase *base) {
  PwmWpt *me = container_of(base, PwmWpt, base);
  uint32_t timer_mask = (1u << me->timer);
  bsp_stop(me->bsp_cfg.handle, timer_mask, me->output_mask);
}

static void wpt_set_duty(PwmBase *base, uint8_t ch, float duty) {
  (void) ch;  // 单相半桥, 忽略通道号
  PwmWpt *me = container_of(base, PwmWpt, base);
  me->duty = clamp_f(duty, 0.0f, 1.0f);  // 防御性 [0,1] 钳位 (基类已钳 [duty_min, duty_max])
  if (me->bsp_cfg.clk_hz == 0u || me->base.freq_hz == 0u) {
    return;  // handle 未注入, 仅更新字段
  }
  uint32_t period = me->bsp_cfg.clk_hz / me->base.freq_hz;
  uint32_t cmp1, cmp3;
  wpt_leg_cmp(period, duty, &cmp1, &cmp3);
  bsp_update_duty(me->bsp_cfg.handle, me->timer, cmp1, cmp3);
}

static void wpt_set_freq(PwmBase *base, uint32_t freq_hz) {
  PwmWpt *me = container_of(base, PwmWpt, base);
  if (freq_hz == 0u || me->bsp_cfg.clk_hz == 0u) {
    return;  // 无效频率或 handle 未注入
  }
  uint32_t period = me->bsp_cfg.clk_hz / freq_hz;
  bsp_update_period(me->bsp_cfg.handle, me->timer, period);
  pwm_set_duty(&me->base, 0, me->duty);  // 周期变化后重映射占空比
}

static void wpt_set_deadtime(PwmBase *base, uint32_t deadtime_ns) {
  PwmWpt *me = container_of(base, PwmWpt, base);
  bsp_update_deadtime(me->bsp_cfg.handle, me->timer, deadtime_ns, deadtime_ns);
}

static void wpt_emergency_stop(PwmBase *base) {
  PwmWpt *me = container_of(base, PwmWpt, base);
  bsp_emergency_stop(me->bsp_cfg.handle, me->output_mask);
}

// ======== 虚表 (无 set_phase —— pwm_set_phase 对 NULL 安全检查) ========
static const PwmOps wpt_ops = {
    .start = wpt_start,
    .stop = wpt_stop,
    .set_duty = wpt_set_duty,
    .set_freq = wpt_set_freq,
    .set_deadtime = wpt_set_deadtime,
    .emergency_stop = wpt_emergency_stop,
};

// ======== 构造 ========

void pwm_wpt_init(PwmWpt *me, uint32_t freq_hz, BspPwmTimer timer, uint32_t output_mask, float duty_emin) {
  pwm_base_init(&me->base);

  me->bsp_cfg.handle = NULL;
  me->bsp_cfg.clk_hz = 0u;
  me->bsp_cfg.use_dll = false;

  me->timer = timer;
  me->output_mask = output_mask;
  me->duty_emin = duty_emin;

  me->duty = 0.0f;
  me->vb_limit_by_duty = 0.0f;
  me->freq_div4 = 1u;
  me->st = WPT_OFF;

  me->base.mode = PwmMode_HalfBridge;
  me->base.num_ch = 1u;
  me->base.duty_min = duty_emin;
  me->base.duty_max = 0.99f;
  me->base.freq_hz = freq_hz;
  me->base.ops = &wpt_ops;
}

// ======== 运行时调参 ========

void pwm_wpt_set_duty(PwmWpt *me, float duty) {
  pwm_set_duty(&me->base, 0, duty);  // 基类钳位 [duty_min, duty_max] + 分派写硬件
}

void pwm_wpt_set_vb_limit(PwmWpt *me, float duty_lo) {
  me->vb_limit_by_duty = duty_lo;
  me->base.duty_min = (duty_lo > me->duty_emin) ? duty_lo : me->duty_emin;
  if (me->duty < me->base.duty_min) {
    pwm_set_duty(&me->base, 0, me->duty);  // 已写的过小 duty 重新钳位
  }
}

void pwm_wpt_set_freq_div(PwmWpt *me, uint8_t div) {
  me->freq_div4 = div;
}

uint32_t pwm_wpt_coil_freq_hz(const PwmWpt *me) {
  if (me->freq_div4 == 0u) {
    return me->base.freq_hz;
  }
  return me->base.freq_hz / me->freq_div4;
}

// ======== 状态机 ========

void pwm_wpt_set_state(PwmWpt *me, WptSt st) {
  me->st = st;
}

WptSt pwm_wpt_state(const PwmWpt *me) {
  return me->st;
}

// ======== 生命周期 (薄包装: 底层启停 + 同步状态机) ========

void pwm_wpt_start(PwmWpt *me) {
  pwm_start(&me->base);
  me->st = WPT_CHARGING;
}

void pwm_wpt_stop(PwmWpt *me) {
  pwm_stop(&me->base);
  me->st = WPT_OFF;
}

void pwm_wpt_emergency(PwmWpt *me) {
  pwm_emergency_stop(&me->base);
  me->st = WPT_ERROR;
}

void pwm_wpt_set_freq(PwmWpt *me, uint32_t freq_hz) {
  pwm_set_freq(&me->base, freq_hz);
}

void pwm_wpt_set_deadtime(PwmWpt *me, uint32_t deadtime_ns) {
  pwm_set_deadtime(&me->base, deadtime_ns);
}
