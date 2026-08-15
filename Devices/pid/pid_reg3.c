// 3 状态 PID 调节器 —— PidBase 子类实现 (TI pid_reg3: 反计算抗饱和)

#include "pid_reg3.h"
#include "container_of.h"

// 默认配置 (对齐 TI PIDREG3_DEFAULTS)
static const PidReg3Cfg PID_REG3_DEFAULT_CFG = { 1.3f, 0.02f, 0.5f, 1.05f };

// ======== ops 实现 ========

// 核心计算 (标准变体, 反计算抗饱和):
//   Ui += Ki×Up + Kc×SatErr (上拍的饱和差) ; 返回 (Up+Ui) 未限幅
static float reg3_compute(PidBase *base, float target, float measure) {
  PidReg3 *me = container_of(base, PidReg3, base);
  const PidReg3Cfg *cfg = &me->cfg;

  float error = target - measure;          // Err
  me->up = cfg->kp * error;                // Up

  // 积分 + 上一拍反计算校正
  me->ui = me->ui + cfg->ki * me->up + cfg->kc * me->sat_err;

  // 默认"本拍未饱和", 若基类 detect 到限幅会在 on_saturation 覆盖 sat_err
  me->sat_err = 0.0f;

  return me->up + me->ui;                  // OutPreSat (未限幅)
}

// 反计算抗饱和: 基类限幅后回调, 记录本拍饱和差 (供下拍积分校正)
static void reg3_on_saturation(PidBase *base, float raw, float clamped) {
  PidReg3 *me = container_of(base, PidReg3, base);
  me->sat_err = clamped - raw;   // SatErr = Out - OutPreSat
}

// 清零
static void reg3_reset(PidBase *base) {
  PidReg3 *me = container_of(base, PidReg3, base);
  me->ui = 0.0f;
  me->up = 0.0f;
  me->sat_err = 0.0f;
}

static const PidOps reg3_ops = {
  .compute = reg3_compute,
  .reset = reg3_reset,
  .on_saturation = reg3_on_saturation,
};

// ======== 位置变体 (直呼, 不通过虚表) ========
// 误差 ±0.5 回绕 + 微分项 U d = Kd×(Up−Up1) (作用在比例输出差分); 含反计算抗饱和
float pid_reg3_run_pos(PidReg3 *me, float ref, float fdb) {
  const PidReg3Cfg *cfg = &me->cfg;
  float error = ref - fdb;

  // 误差回绕到 [-0.5, 0.5]
  if (error >= 0.5f) error -= 1.0f;
  else if (error <= -0.5f) error += 1.0f;

  float up = cfg->kp * error;
  me->ui = me->ui + cfg->ki * up + cfg->kc * me->sat_err;
  float ud = cfg->kd * (up - me->up);      // 微分作用在比例差分
  me->up = up;

  float pre = up + me->ui + ud;

  // 限幅
  float out;
  if (pre > me->base.out_max) out = me->base.out_max;
  else if (pre < me->base.out_min) out = me->base.out_min;
  else out = pre;

  me->sat_err = out - pre;                 // 供下拍反计算
  return out;
}

// ======== 构造 ========
void pid_reg3_init(PidReg3 *me, float dt, float out_min, float out_max,
                   const PidReg3Cfg *cfg) {
  pid_base_init(&me->base);
  me->cfg = cfg ? *cfg : PID_REG3_DEFAULT_CFG;
  me->ui = 0.0f;
  me->up = 0.0f;
  me->sat_err = 0.0f;
  me->base.dt = dt;
  me->base.out_min = out_min;
  me->base.out_max = out_max;
  me->base.anti_windup = true;
  me->base.ops = &reg3_ops;
}

void pid_reg3_update_config(PidReg3 *me, const PidReg3Cfg *cfg) { me->cfg = *cfg; }
void pid_reg3_set_kp(PidReg3 *me, float v) { me->cfg.kp = v; }
void pid_reg3_set_ki(PidReg3 *me, float v) { me->cfg.ki = v; }
void pid_reg3_set_kc(PidReg3 *me, float v) { me->cfg.kc = v; }
void pid_reg3_set_kd(PidReg3 *me, float v) { me->cfg.kd = v; }
