// 4 状态 PI 调节器 —— PidBase 子类实现 (TI pi_reg4: 设定值滤波 + P + I + 前馈 + clamping 抗饱和)

#include "pid_reg4.h"
#include "container_of.h"
#include "comp_math.h"   // M_2PI

// 默认配置
static const PidReg4Cfg PID_REG4_DEFAULT_CFG = { 1.0f, 0.0f, 0.0f, 0.0f };

// ======== ops 实现 ========

// 设定值一阶低通 (α = 2π·fc·dt / (1+2π·fc·dt))
static inline float sp_filter_step(PidReg4 *me, float setpoint) {
  const PidReg4Cfg *cfg = &me->cfg;
  if (!me->initialized) {
    me->sp_filtered = setpoint;
    me->initialized = true;
    return setpoint;
  }
  if (cfg->sp_fc <= 0.0f) {
    me->sp_filtered = setpoint;
    return setpoint;
  }
  float alpha = M_2PI * cfg->sp_fc * me->base.dt;
  alpha = alpha / (1.0f + alpha);
  me->sp_filtered = alpha * setpoint + (1.0f - alpha) * me->sp_filtered;
  return me->sp_filtered;
}

// 核心计算 (与 TI pi_reg4_run 位级对齐):
//   sp_filter → P + I (含 clamping 抗饱和) + FF → 返回未限幅和 (基类负责输出限幅)
static float reg4_compute(PidBase *base, float target, float measure) {
  PidReg4 *me = container_of(base, PidReg4, base);
  const PidReg4Cfg *cfg = &me->cfg;

  float sp_f = sp_filter_step(me, target);
  float error = sp_f - measure;

  float p_term = cfg->kp * error;       // 比例项
  float ff_term = cfg->kff * target;    // 前馈项 (未滤波)

  // 积分累加 (含 clamping 抗饱和: 输出越界时冻结)
  float out = p_term + me->integral + ff_term;
  bool saturated = (base->out_max <= base->out_min)
                || (out >= base->out_max && error > 0.0f)
                || (out <= base->out_min && error < 0.0f);
  if (!saturated) {
    me->integral += cfg->ki * me->base.dt * error;
  }

  return p_term + me->integral + ff_term;
}

// 清零
static void reg4_reset(PidBase *base) {
  PidReg4 *me = container_of(base, PidReg4, base);
  me->integral = 0.0f;
  me->sp_filtered = 0.0f;
}

static const PidOps reg4_ops = {
  .compute = reg4_compute,
  .reset = reg4_reset,
  .on_saturation = 0,   // clamping 抗饱和已在 compute 内完成, 基类只做输出限幅
};

// ======== 构造 ========
void pid_reg4_init(PidReg4 *me, float dt, float out_min, float out_max,
                   const PidReg4Cfg *cfg) {
  pid_base_init(&me->base);
  me->cfg = cfg ? *cfg : PID_REG4_DEFAULT_CFG;
  me->integral = 0.0f;
  me->sp_filtered = 0.0f;
  me->initialized = false;
  me->base.dt = dt;
  me->base.out_min = out_min;
  me->base.out_max = out_max;
  me->base.anti_windup = false;   // clamping 在 compute 内, 无需基类 on_saturation
  me->base.ops = &reg4_ops;
}

void pid_reg4_update_config(PidReg4 *me, const PidReg4Cfg *cfg) { me->cfg = *cfg; }
void pid_reg4_set_kp(PidReg4 *me, float v)   { me->cfg.kp = v; }
void pid_reg4_set_ki(PidReg4 *me, float v)   { me->cfg.ki = v; }
void pid_reg4_set_kff(PidReg4 *me, float v)  { me->cfg.kff = v; }
void pid_reg4_set_sp_fc(PidReg4 *me, float fc) { me->cfg.sp_fc = fc; }
