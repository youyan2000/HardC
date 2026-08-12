// 半桥 PWM —— PwmBase 子类
//
// 互补输出 + 死区: 上管按占空比导通, 下管自动互补 (硬件死区单元生成)
// 中心对齐 PWM: CMP1=上升沿 (ON), CMP3=下降沿 (OFF), 对称分布在 period/2 两侧
//
// 波形:
//   中心对齐:  period/2 * (1-duty) → SET,  period/2 * (1+duty) → RESET
//   边沿对齐:  period * duty       → SET,  period             → RESET
//
// 安全: emergency_stop 直接封波, 上下管均置为安全电平

#include "pwm_half_bridge.h"
#include "container_of.h"
#include <stddef.h>

// ======== 内部辅助: 占空比 → 比较值 ========

// 计算比较值的占空比系数
static inline float clamp_duty(float duty) {
  if (duty < 0.0f) return 0.0f;
  if (duty > 1.0f) return 1.0f;
  return duty;
}

// ======== ops 实现 ========

// 启动: 先启计数器 → 延时等波形对齐 → 再开输出 (防止上电瞬间电平不确定)
static void hb_start(PwmBase *base) {
  PwmHalfBridge *me = container_of(base, PwmHalfBridge, base);

  // 配置定时器
  BspPwmTimerConfig tcfg = {
    .timer         = me->timer,
    .period        = me->period,
    .output_mask   = me->output_mask,
    .complementary = true,
  };

  // 中心对齐: 上下计数, CMP2=period/2 作为中心参考
  if (me->center_aligned) {
    tcfg.cmp1 = (me->period / 2) * (1.0f - me->duty);
    tcfg.cmp2 = me->period / 2;  // 中心参考点
    tcfg.cmp3 = (me->period / 2) * (1.0f + me->duty);
  } else {
    tcfg.cmp1 = me->period * me->duty;
    tcfg.cmp2 = 0;
    tcfg.cmp3 = 0;  // 边沿对齐不用 CMP3
  }

  // 死区
  tcfg.deadtime_rising  = me->deadtime_ns;
  tcfg.deadtime_falling = me->deadtime_ns;

  bsp_config_timer(me->bsp_cfg.handle, &tcfg);

  uint32_t timer_mask = (1u << me->timer);
  bsp_start(me->bsp_cfg.handle, timer_mask, me->output_mask);
}

// 停止: 先封输出 → 再停计数器
static void hb_stop(PwmBase *base) {
  PwmHalfBridge *me = container_of(base, PwmHalfBridge, base);
  uint32_t timer_mask = (1u << me->timer);
  bsp_stop(me->bsp_cfg.handle, timer_mask, me->output_mask);
}

// 设置占空比 (热路径 —— 只写 CMP1/CMP3, 不重新配置定时器)
static void hb_set_duty(PwmBase *base, uint8_t ch, float duty) {
  (void)ch;  // 半桥只有一对互补输出, 忽略通道号
  PwmHalfBridge *me = container_of(base, PwmHalfBridge, base);
  duty = clamp_duty(duty);
  me->duty = duty;

  uint32_t cmp1, cmp3;
  if (me->center_aligned) {
    uint32_t half = me->period / 2;
    cmp1 = half * (1.0f - duty);
    cmp3 = half * (1.0f + duty);
  } else {
    cmp1 = me->period * duty;
    cmp3 = 0;
  }

  bsp_update_duty(me->bsp_cfg.handle, me->timer, cmp1, cmp3);
}

// 设置频率: 重算周期 → 更新时基 → 保持占空比不变
static void hb_set_freq(PwmBase *base, uint32_t freq_hz) {
  PwmHalfBridge *me = container_of(base, PwmHalfBridge, base);

  // period = f_clk / freq
  // BSP 根据 clk_hz 和是否 DLL 计算实际计数器周期
  // 简化: 这里只存标称值, BSP 内部做换算
  me->period = me->bsp_cfg.clk_hz / freq_hz;

  bsp_update_period(me->bsp_cfg.handle, me->timer, me->period);

  // 周期变了, 占空比需要重新映射到新比较值
  float duty = me->duty;  // 保存
  hb_set_duty(base, 0, duty);
}

// 设置死区: BSP 内部转换为 tick 值写入硬件
static void hb_set_deadtime(PwmBase *base, uint32_t deadtime_ns) {
  PwmHalfBridge *me = container_of(base, PwmHalfBridge, base);
  me->deadtime_ns = deadtime_ns;
  bsp_update_deadtime(me->bsp_cfg.handle, me->timer,
                          deadtime_ns, deadtime_ns);
}

// 半桥无相位概念, set_phase 为空操作
static void hb_set_phase(PwmBase *base, uint8_t ch, float phase_deg) {
  (void)base; (void)ch; (void)phase_deg;
  // 半桥拓扑不需要相位调整
}

// 紧急停机: 硬件级封波, 不经过软件判断
static void hb_emergency_stop(PwmBase *base) {
  PwmHalfBridge *me = container_of(base, PwmHalfBridge, base);
  bsp_emergency_stop(me->bsp_cfg.handle, me->output_mask);
}

// ======== 虚表 ========
static const PwmOps hb_ops = {
  .start          = hb_start,
  .stop           = hb_stop,
  .set_duty       = hb_set_duty,
  .set_freq       = hb_set_freq,
  .set_deadtime   = hb_set_deadtime,
  .set_phase      = hb_set_phase,       // 空操作
  .emergency_stop = hb_emergency_stop,
};

// ======== 构造 ========

void pwm_hb_init(PwmHalfBridge *me, uint32_t freq_hz, uint32_t deadtime_ns,
                 BspPwmTimer timer, uint32_t output_mask) {
  pwm_base_init(&me->base);

  // BSP 配置 (时钟频和 DLL 标志由 bsp_init 内部设置)
  me->bsp_cfg.handle       = NULL;  // bsp_init 内部分配
  me->bsp_cfg.clk_hz = 0;
  me->bsp_cfg.use_dll      = false;

  me->timer       = timer;
  me->output_mask = output_mask;

  // 电力电子参数
  me->duty         = 0.0f;       // 启动时占空比=0, 安全
  me->deadtime_ns  = deadtime_ns;
  me->period       = 0;          // bsp_init 后由 set_freq 更新

  // 默认: 中心对齐, 高有效
  me->center_aligned = true;
  me->active_high    = true;

  // 基类字段覆盖
  me->base.mode     = PwmMode_HalfBridge;
  me->base.num_ch   = 1;                    // 半桥 = 1 个可独立控制的通道
  me->base.duty_min = 0.0f;
  me->base.duty_max = 0.98f;               // 留 2% 死区裕量, 防止 100% 占空比直通
  me->base.ops      = &hb_ops;

  // 初始化 BSP
  bsp_init(&me->bsp_cfg);

  // 首设置频率 (计算 period)
  hb_set_freq(&me->base, freq_hz);
}

// ======== 反初始化 ========
void pwm_hb_deinit(PwmHalfBridge *me) {
  if (me->base.running) {
    hb_stop(&me->base);
  }
  me->bsp_cfg.handle = NULL;
  me->timer          = 0;
  me->output_mask    = 0;
  me->duty           = 0.0f;
  me->deadtime_ns    = 0;
  me->period         = 0;
  pwm_base_init(&me->base);  // 重置基类
}

// ======== 运行时调参 ========

void pwm_hb_set_duty(PwmHalfBridge *me, float duty) {
  pwm_set_duty(&me->base, 0, duty);  // 走基类限幅 + 分派
}

void pwm_hb_set_freq(PwmHalfBridge *me, uint32_t freq_hz) {
  pwm_set_freq(&me->base, freq_hz);
}

void pwm_hb_set_deadtime(PwmHalfBridge *me, uint32_t deadtime_ns) {
  pwm_set_deadtime(&me->base, deadtime_ns);
}

void pwm_hb_set_alignment(PwmHalfBridge *me, bool center_aligned) {
  me->center_aligned = center_aligned;
  // 切换对齐方式后需重写占空比 (比较值计算公式不同)
  hb_set_duty(&me->base, 0, me->duty);
}
