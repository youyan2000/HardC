// 六开关 SVPWM 实现 — 空间矢量调制核心算法
//
// 来源: TI controlSUITE SVGEN (motor_control/math_blocks/v4.3)
// 翻译为 C-OOP 纯C float 版本
//
// 算法:
//   1. 三相参考电压投影 (Va, Vb, Vc)
//   2. 扇区判断 (1~6)
//   3. 相邻矢量作用时间 (t1, t2)
//   4. 过调制处理 (t1+t2 > T → 等比例缩放)
//   5. 三相占空比计算 (Ta, Tb, Tc)
//   6. 7段对称: V0→V1→V2→V7→V2→V1→V0

#include "pwm_svpwm.h"
#include "container_of.h"
#include "comp_transform.h"   // SQRT3_OVER_2
#include <stddef.h>
#include <math.h>

// ======== 内部辅助 ========

static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// 计算单相腿的比较值 (中心对齐: cmp1=rise, cmp3=fall)
static void update_one_leg(BspPwmConfig *bsp, BspPwmTimer timer,
                           uint32_t period, float duty) {
  uint32_t half = period / 2;
  uint32_t cmp1 = half * (1.0f - duty);
  uint32_t cmp3 = half * (1.0f + duty);
  bsp_update_duty(bsp->handle, timer, cmp1, cmp3);
}

// ======== ops 实现 ========

static void svpwm_start(PwmBase *base) {
  PwmSvpwm *me = container_of(base, PwmSvpwm, base);
  if (!me->bsph) return;  // BSP 未绑定, 无操作

  // 顺序启动三相 (A→B→C), 每相延时对齐
  BspPwmTimer timers[3] = { me->timer_a, me->timer_b, me->timer_c };
  uint32_t masks[3] = { me->output_mask_a, me->output_mask_b, me->output_mask_c };

  for (int i = 0; i < 3; i++) {
    bsp_start(me->bsph, (1u << timers[i]), masks[i]);
  }
}

static void svpwm_stop(PwmBase *base) {
  PwmSvpwm *me = container_of(base, PwmSvpwm, base);
  if (!me->bsph) return;

  uint32_t timers[3] = { (1u << me->timer_a), (1u << me->timer_b), (1u << me->timer_c) };
  for (int i = 0; i < 3; i++) {
    bsp_stop(me->bsph, timers[i], 0);
  }
}

// set_duty: 单相占空比设置 (用于调试/开环, 正常闭环用 svpwm_set_vector)
static void svpwm_set_duty(PwmBase *base, uint8_t ch, float duty) {
  PwmSvpwm *me = container_of(base, PwmSvpwm, base);

  duty = clampf(duty, 0.0f, 1.0f);

  switch (ch) {
    case 0: me->duty_a = duty; break;
    case 1: me->duty_b = duty; break;
    case 2: me->duty_c = duty; break;
    default: return;
  }
}

static void svpwm_set_freq(PwmBase *base, uint32_t freq_hz) {
  PwmSvpwm *me = container_of(base, PwmSvpwm, base);
  if (!me->bsph) return;

  // 三相同步更新频率
  bsp_set_freq(me->bsph, me->timer_a, freq_hz);
  bsp_set_freq(me->bsph, me->timer_b, freq_hz);
  bsp_set_freq(me->bsph, me->timer_c, freq_hz);
}

static void svpwm_set_deadtime(PwmBase *base, uint32_t deadtime_ns) {
  PwmSvpwm *me = container_of(base, PwmSvpwm, base);
  if (!me->bsph) return;

  bsp_set_deadtime(me->bsph, me->timer_a, deadtime_ns);
  bsp_set_deadtime(me->bsph, me->timer_b, deadtime_ns);
  bsp_set_deadtime(me->bsph, me->timer_c, deadtime_ns);
}

static void svpwm_set_phase(PwmBase *base, uint8_t ch, float phase_deg) {
  // SVPWM 三相相位由矢量算法自动确定, 外部相位设置无意义
  (void)base; (void)ch; (void)phase_deg;
}

static void svpwm_emergency_stop(PwmBase *base) {
  PwmSvpwm *me = container_of(base, PwmSvpwm, base);
  if (!me->bsph) return;

  // 三相全部封波
  bsp_emergency_stop(me->bsph, me->output_mask_a | me->output_mask_b | me->output_mask_c);
}

// ======== 虚表 ========
static const PwmOps svpwm_ops = {
  .start          = svpwm_start,
  .stop           = svpwm_stop,
  .set_duty       = svpwm_set_duty,
  .set_freq       = svpwm_set_freq,
  .set_deadtime   = svpwm_set_deadtime,
  .set_phase      = svpwm_set_phase,
  .emergency_stop = svpwm_emergency_stop,
};

// ======== SVPWM 核心算法 ========

void svpwm_set_vector(PwmSvpwm *me, float v_alpha, float v_beta, float v_dc_bus) {
  me->v_alpha = v_alpha;
  me->v_beta = v_beta;
  me->v_dc_bus = v_dc_bus;

  // ---- 阶段 1: 三相参考电压投影 ----
  // Va = Vβ,  Vb = (√3·Vα - Vβ)/2,  Vc = -(√3·Vα + Vβ)/2
  float va = v_beta;
  float vb = 0.5f * (SQRT3_OVER_2 * 2.0f * v_alpha - v_beta);
  float vc = -0.5f * (SQRT3_OVER_2 * 2.0f * v_alpha + v_beta);

  // ---- 阶段 2: 扇区判断 ----
  // sector = (Va>0)*1 + (Vb>0)*2 + (Vc>0)*4  →  扇区 1~6
  int sector = 0;
  if (va > 0.0f) sector |= 1;
  if (vb > 0.0f) sector |= 2;
  if (vc > 0.0f) sector |= 4;

  // ---- 阶段 3: 矢量作用时间 ----
  // X = Vβ,  Y = (√3·Vα + Vβ)/2,  Z = (-√3·Vα + Vβ)/2
  float x = v_beta;
  float y = 0.5f * (SQRT3_OVER_2 * 2.0f * v_alpha + v_beta);
  float z = 0.5f * (-SQRT3_OVER_2 * 2.0f * v_alpha + v_beta);

  float t1, t2;
  // 按扇区查表: t1 = 第一矢量作用时间, t2 = 第二矢量作用时间
  switch (sector) {
    case 0:  t1 = 0.5f;  t2 = 0.5f;  break;  // 零矢量 (仅 V0/V7)
    case 1:  t1 =  z;    t2 =  y;    break;  // sector I
    case 2:  t1 =  y;    t2 = -x;    break;  // sector II
    case 3:  t1 = -z;    t2 =  x;    break;  // sector III
    case 4:  t1 = -x;    t2 =  z;    break;  // sector IV
    case 5:  t1 =  x;    t2 = -y;    break;  // sector V
    case 6:  t1 = -y;    t2 = -z;    break;  // sector VI
    default: t1 = 0.0f;  t2 = 0.0f;  break;
  }

  me->t1 = t1;
  me->t2 = t2;
  me->sector = (uint8_t)(sector > 0 ? sector : 0);

  // ---- 阶段 4: 过调制处理 ----
  if (me->overmod_enable) {
    // 饱和: 等比例缩放保持矢量方向
    if ((t1 + t2) > me->overmod_limit) {
      float scale = me->overmod_limit / (t1 + t2);
      t1 *= scale;
      t2 *= scale;
    }
  } else {
    // 线性调制区限制: |V_ref| ≤ 0.577 (= 1/√3, 对应内切圆)
    if ((t1 + t2) > 1.0f) {
      float scale = 1.0f / (t1 + t2);
      t1 *= scale;
      t2 *= scale;
    }
  }

  // ---- 阶段 5: 三相占空比 ----
  // 7 段对称: taon=(1-t1-t2)/4, tbon=taon+t1/2, tcon=tbon+t2/2
  float ta_on, tb_on, tc_on;
  float half_t1 = 0.5f * t1;
  float half_t2 = 0.5f * t2;
  float t0_half = 0.5f * (1.0f - t1 - t2);  // zero vector time per side

  switch (sector) {
    case 0:  // 零矢量
      ta_on = 0.5f; tb_on = 0.5f; tc_on = 0.5f;
      break;
    case 1:  // V4(100) → V6(110)
      ta_on = t0_half + t1 + t2;
      tb_on = t0_half + t2;
      tc_on = t0_half;
      break;
    case 2:  // V6(110) → V2(010)
      ta_on = t0_half + t1;
      tb_on = t0_half + t1 + t2;
      tc_on = t0_half;
      break;
    case 3:  // V2(010) → V3(011)
      ta_on = t0_half;
      tb_on = t0_half + t1 + t2;
      tc_on = t0_half + t2;
      break;
    case 4:  // V3(011) → V1(001)
      ta_on = t0_half;
      tb_on = t0_half + t1;
      tc_on = t0_half + t1 + t2;
      break;
    case 5:  // V1(001) → V5(101)
      ta_on = t0_half + t2;
      tb_on = t0_half;
      tc_on = t0_half + t1 + t2;
      break;
    case 6:  // V5(101) → V4(100)
      ta_on = t0_half + t1 + t2;
      tb_on = t0_half;
      tc_on = t0_half + t1;
      break;
    default:
      ta_on = 0.5f; tb_on = 0.5f; tc_on = 0.5f;
      break;
  }

  me->duty_a = clampf(ta_on, 0.0f, 1.0f);
  me->duty_b = clampf(tb_on, 0.0f, 1.0f);
  me->duty_c = clampf(tc_on, 0.0f, 1.0f);
}

float svpwm_get_modulation_index(const PwmSvpwm *me) {
  // 调制比 = |V_ref| / (Vdc/√3) = sqrt(alpha²+beta²) * √3 / Vdc
  // 注: v_alpha/v_beta 已是标幺值 (相对 0.5*Vdc)
  float mag = sqrtf(me->v_alpha * me->v_alpha + me->v_beta * me->v_beta);
  return mag * 2.0f;  // 标幺值 * 2 = 相对内切圆的调制比
}

void svpwm_set_mode(PwmSvpwm *me, SvpwmMode mode) {
  me->mode = mode;
  // 5 段式: 每 60° 不切换一相, 减少 1/3 开关损耗
  // 实现: 调整零矢量分配, 将一个 t0_half 置 0 (V0-only 或 V7-only)
  // 具体实现留给硬件性能优化阶段
}

// ======== 构造 ========

void svpwm_init(PwmSvpwm *me, uint32_t freq_hz, uint32_t deadtime_ns,
                BspPwmTimer timer_a, uint32_t output_mask_a,
                BspPwmTimer timer_b, uint32_t output_mask_b,
                BspPwmTimer timer_c, uint32_t output_mask_c) {
  pwm_base_init(&me->base);

  me->bsph = NULL;          // BSP 句柄由 board_init() 绑定
  me->timer_a = timer_a;
  me->timer_b = timer_b;
  me->timer_c = timer_c;
  me->output_mask_a = output_mask_a;
  me->output_mask_b = output_mask_b;
  me->output_mask_c = output_mask_c;

  me->v_alpha = 0.0f;
  me->v_beta = 0.0f;
  me->v_dc_bus = 1.0f;    // 默认 1V, 由 App 层更新

  me->duty_a = 0.5f;      // 启动时三相均输出 50% (零矢量)
  me->duty_b = 0.5f;
  me->duty_c = 0.5f;
  me->sector = 0;

  me->mode = SvpwmMode_7Seg;
  me->overmod_limit = 1.0f;
  me->overmod_enable = false;

  // 基类字段
  me->base.mode     = PwmMode_FullBridge;  // 复用模式标识
  me->base.num_ch   = 3;                    // 3 个独立占空比通道
  me->base.freq_hz  = freq_hz;
  me->base.duty_min = 0.02f;               // 2% 下限, 留脉冲成形裕量
  me->base.duty_max = 0.98f;               // 2% 上限, 留死区裕量
  me->base.ops      = &svpwm_ops;
}

void svpwm_deinit(PwmSvpwm *me) {
  if (me->base.running) {
    svpwm_stop(&me->base);
  }
  me->timer_a = 0;
  me->timer_b = 0;
  me->timer_c = 0;
  me->output_mask_a = 0;
  me->output_mask_b = 0;
  me->output_mask_c = 0;
  pwm_base_init(&me->base);
}
