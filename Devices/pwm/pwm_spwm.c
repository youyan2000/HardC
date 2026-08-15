// 正弦脉宽调制 SPWM —— PwmBase 子类实现
//
// 调制律 (正弦调制波 × 三角载波比较), Device 内部实现, host 可测:
//   上管占空比: duty_i = 0.5 × (1 + m × sin(θ_i))
//   THI (可选): duty_i = 0.5 × (1 + m × (sin(θ_i) + k × sin(3θ_i)))
//   m = 调制比, θ_i = 调制波相位 = θ + 臂相位偏移 φ_i
//
// BSP 只接收物理 CMP 值; 占空比 → CMP1/CMP3 换算在本层 (中心对齐双沿).
// 各臂相位差由 init 固定: 单相 (N=2) → φ = [0, 180], 三相 (N=3) → φ = [0, 120, 240].
// 互补输出 + 死区均由 BSP 硬件单元生成, 本层只写 duty 与死区时间.

#include "pwm_spwm.h"
#include "container_of.h"
#include <stddef.h>
#include <math.h>
#include "comp_math.h"  // M_PI (全库唯一 π float 常量源)

// ======== 内部辅助 ========

static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline float deg2rad(float deg) {
  return deg * (float)M_PI / 180.0f;
}

// 按调制比 m 与单臂调制波相位 (rad) 计算上管占空比
// THI 开启时注入 k·sin(3θ) 零序分量 (三相三臂差 120° → 3θ 差 360° 同相, 即零序)
static inline float spwm_duty_from_phase(const PwmSpwm *me, float m, float theta_rad) {
  float s = sinf(theta_rad);
  if (me->thi_enable) {
    s += me->thi_coeff * sinf(3.0f * theta_rad);
  }
  return 0.5f * (1.0f + m * s);
}

// 单臂占空比 → CMP1/CMP3 (中心对齐: 对称双沿; 不支持边沿对齐)
static void spwm_arm_cmp(const PwmSpwm *me, float duty, uint32_t *cmp1, uint32_t *cmp3) {
  uint32_t half = me->period / 2;
  *cmp1 = (uint32_t) ((float) half * (1.0f - duty));
  *cmp3 = (uint32_t) ((float) half * (1.0f + duty));
}

// 写某臂占空比到硬件 (热路径)
static void spwm_write_arm(PwmSpwm *me, PwmSpwmArm *arm) {
  uint32_t c1, c3;
  spwm_arm_cmp(me, arm->duty, &c1, &c3);
  bsp_update_duty(me->bsp_cfg.handle, arm->timer, c1, c3);
}

// 重算全部臂占空比并写硬件 (给定各臂调制波相位 rad)
static void spwm_write_all(PwmSpwm *me, float theta_rad, float m) {
  for (uint8_t i = 0; i < me->num_arms; i++) {
    PwmSpwmArm *arm = &me->arms[i];
    arm->duty = clampf(spwm_duty_from_phase(me, m, theta_rad + deg2rad(arm->phase_deg)),
                       0.0f, me->base.duty_max);
    spwm_write_arm(me, arm);
  }
}

// ======== ops 实现 ========

static void spwm_start(PwmBase *base) {
  PwmSpwm *me = container_of(base, PwmSpwm, base);
  if (!me->bsp_cfg.handle)
    return;
  // 载波周期未初始化 (App 未注入 clk_hz/未调 set_freq) 时拒绝启动,
  // 否则 cmp1/cmp3 全 0 → 上下管近似直通, 逆变炸桥风险.
  if (me->period == 0u)
    return;

  uint32_t timer_mask = 0u, output_mask = 0u;
  for (uint8_t i = 0; i < me->num_arms; i++) {
    PwmSpwmArm *arm = &me->arms[i];
    BspPwmTimerConfig tcfg = {
      .timer           = arm->timer,
      .period          = me->period,
      .cmp1            = 0u,
      .cmp2            = me->center_aligned ? (me->period / 2) : 0u,
      .cmp3            = 0u,
      .output_mask     = arm->output_mask,
      .complementary   = true,          // 半桥互补 + 死区
      .deadtime_rising = me->deadtime_ns,
      .deadtime_falling = me->deadtime_ns,
    };
    spwm_arm_cmp(me, arm->duty, &tcfg.cmp1, &tcfg.cmp3);
    bsp_config_timer(me->bsp_cfg.handle, &tcfg);

    timer_mask  |= (1u << arm->timer);
    output_mask |= arm->output_mask;
  }
  bsp_start(me->bsp_cfg.handle, timer_mask, output_mask);
}

static void spwm_stop(PwmBase *base) {
  PwmSpwm *me = container_of(base, PwmSpwm, base);
  if (!me->bsp_cfg.handle)
    return;

  uint32_t timer_mask = 0u, output_mask = 0u;
  for (uint8_t i = 0; i < me->num_arms; i++) {
    timer_mask  |= (1u << me->arms[i].timer);
    output_mask |= me->arms[i].output_mask;
  }
  bsp_stop(me->bsp_cfg.handle, timer_mask, output_mask);
}

// 调试/开环: 写某臂上管绝对占空比 (ch = 臂索引)
static void spwm_set_duty(PwmBase *base, uint8_t ch, float duty) {
  PwmSpwm *me = container_of(base, PwmSpwm, base);
  if (ch >= me->num_arms)
    return;
  PwmSpwmArm *arm = &me->arms[ch];
  arm->duty = clampf(duty, 0.0f, me->base.duty_max);
  spwm_write_arm(me, arm);
}

static void spwm_set_freq(PwmBase *base, uint32_t freq_hz) {
  PwmSpwm *me = container_of(base, PwmSpwm, base);
  if (freq_hz == 0u)
    return;
  // 需 clk_hz + handle 均已注入才可换算 period 并写硬件 (接线契约); 否则仅记录.
  if (!me->bsp_cfg.handle || me->bsp_cfg.clk_hz == 0u)
    return;
  me->period = me->bsp_cfg.clk_hz / freq_hz;
  for (uint8_t i = 0; i < me->num_arms; i++) {
    bsp_update_period(me->bsp_cfg.handle, me->arms[i].timer, me->period);
  }
  spwm_write_all(me, 0.0f, me->mod_index);  // 保持当前占空比重映射
}

static void spwm_set_deadtime(PwmBase *base, uint32_t deadtime_ns) {
  PwmSpwm *me = container_of(base, PwmSpwm, base);
  me->deadtime_ns = deadtime_ns;
  for (uint8_t i = 0; i < me->num_arms; i++) {
    bsp_update_deadtime(me->bsp_cfg.handle, me->arms[i].timer,
                        deadtime_ns, deadtime_ns);
  }
}

// 各臂调制波相位差由 init 固定 (spwm_set_point 内部直接用), 运行时不可改
static void spwm_set_phase(PwmBase *base, uint8_t ch, float phase_deg) {
  (void)base; (void)ch; (void)phase_deg;
}

static void spwm_emergency_stop(PwmBase *base) {
  PwmSpwm *me = container_of(base, PwmSpwm, base);
  if (!me->bsp_cfg.handle)
    return;
  uint32_t output_mask = 0u;
  for (uint8_t i = 0; i < me->num_arms; i++) {
    output_mask |= me->arms[i].output_mask;
  }
  bsp_emergency_stop(me->bsp_cfg.handle, output_mask);
}

// ======== 虚表 ========
static const PwmOps spwm_ops = {
    .start          = spwm_start,
    .stop           = spwm_stop,
    .set_duty       = spwm_set_duty,
    .set_freq       = spwm_set_freq,
    .set_deadtime   = spwm_set_deadtime,
    .set_phase      = spwm_set_phase,       // 空操作 (相位在 init 固定)
    .emergency_stop = spwm_emergency_stop,
};

// ======== 构造 ========

void pwm_spwm_init(PwmSpwm *me, uint32_t freq_hz, uint32_t deadtime_ns,
                   uint8_t num_arms, const PwmSpwmArmCfg *cfg) {
  pwm_base_init(&me->base);

  me->bsp_cfg.handle = NULL;
  me->bsp_cfg.clk_hz = 0u;
  me->bsp_cfg.use_dll = false;

  if (num_arms == 0u)
    num_arms = 2u;                       // 默认单相全桥
  if (num_arms > PWM_SPWM_MAX_ARMS)
    num_arms = PWM_SPWM_MAX_ARMS;
  me->num_arms = num_arms;

  me->period        = 0u;
  me->deadtime_ns   = deadtime_ns;
  me->center_aligned = true;
  me->mod_index     = 0.0f;               // 安全: 启动时零调制
  me->thi_enable    = false;
  me->thi_coeff     = PWM_SPWM_THI_6th;

  // 各臂固定相位差: 单相 180°, 三相 120°
  for (uint8_t i = 0; i < num_arms; i++) {
    PwmSpwmArm *arm = &me->arms[i];
    arm->timer       = cfg[i].timer;
    arm->output_mask = cfg[i].output_mask;
    arm->phase_deg   = (360.0f / num_arms) * i;
    arm->duty        = 0.5f;              // 静止 50% (零调制)
  }

  me->base.mode     = PwmMode_SinePwm;
  me->base.num_ch   = num_arms;
  me->base.duty_min = 0.0f;
  me->base.duty_max = 0.97f;              // 留 3% 死区裕量 (半桥互补)
  me->base.ops      = &spwm_ops;
  me->base.freq_hz  = freq_hz;   // 记录目标载波频率

  // 接线契约: init 阶段不写硬件 (bsp_cfg.handle/clk_hz 均 NULL/0).
  // App 注入 bsp_cfg.handle + clk_hz 后调 pwm_spwm_set_freq 计算 period 并生效.
}

void pwm_spwm_deinit(PwmSpwm *me) {
  if (me->base.running) {
    spwm_stop(&me->base);
  }
  me->bsp_cfg.handle = NULL;
  me->num_arms = 0u;
  pwm_base_init(&me->base);
}

// ======== 核心调制接口 ========

void spwm_set_point(PwmSpwm *me, float m, float theta_deg) {
  if (m < 0.0f)
    m = 0.0f;
  if (m > 1.0f)
    m = 1.0f;
  me->mod_index = m;
  spwm_write_all(me, deg2rad(theta_deg), m);
}

void spwm_set_thi(PwmSpwm *me, bool enable, float coeff) {
  // 三次谐波注入仅三相有意义; 单相注入零序无益, 强制关闭
  if (me->num_arms != 3u)
    enable = false;
  me->thi_enable = enable;
  // coeff > 0 时更新注入系数; coeff == 0 表示保留当前 (默认 1/6) —— 开关由 enable 决定
  if (coeff > 0.0f) {
    me->thi_coeff = coeff;
  }
}

// ======== 运行时调参 ========

void pwm_spwm_set_duty(PwmSpwm *me, uint8_t ch, float duty) {
  pwm_set_duty(&me->base, ch, duty);
}

void pwm_spwm_set_freq(PwmSpwm *me, uint32_t freq_hz) {
  pwm_set_freq(&me->base, freq_hz);
}

void pwm_spwm_set_deadtime(PwmSpwm *me, uint32_t deadtime_ns) {
  pwm_set_deadtime(&me->base, deadtime_ns);
}

uint8_t pwm_spwm_get_num_arms(const PwmSpwm *me) {
  return me->num_arms;
}
