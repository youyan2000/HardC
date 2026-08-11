// 标准 PID 控制器
//
// compute 算法流程 (每次调用依次执行):
//   1. 死区: |Err| < deadzone → 强制 Err=0 (抑制微小波动)
//   2. P 项: Kp × Err
//   3. I 项: 变速积分 + 积分分离 + 积分输出限幅
//   4. D 项: 支持微分先行 (2-DOF: D 只作用于反馈路径)
//   5. 前馈: Kf × ΔTarget (目标变化直接注入前向通道, 不等误差)
//   6. 返回原始输出 (限幅+抗饱和由基类 pid_compute 处理)
//
// on_saturation 回调:
//   反算积分器: integrator += (clamped - raw) / ki
//   本质是状态同步: 让虚拟积分值跟着物理执行器走, 杜绝积分饱和

#include "pid_standard.h"
#include "container_of.h"
#include "comp_math.h"

// ======== ops 实现 ========

// 核心计算: 内模原理 (1/s 跟踪阶跃, s 阻尼) + 2-DOF 路径分离
static float std_compute(PidBase *base, float target, float measure) {
  PidStandard *me = container_of(base, PidStandard, base);
  float error   = target - measure;
  float abs_err = math_abs_f(error);

  // 1. 死区: 误差过小视为零, 避免微振
  if (abs_err < me->cfg.deadzone) {
    error   = 0;
    abs_err = 0;
  }

  // 2. P 项: 比例直接响应当前误差
  float p_out = me->cfg.kp * error;

  // 3. I 项 —— 变速积分: |Err|小时全速, |Err|大时减速/停止, 防过冲
  float speed_ratio;
  if (me->cfg.i_var_a == 0 && me->cfg.i_var_b == 0) {
    speed_ratio = 1.0f;  // 未配置 → 恒速积分
  } else {
    if (abs_err <= me->cfg.i_var_a) {
      speed_ratio = 1.0f;
    } else if (abs_err < me->cfg.i_var_a + me->cfg.i_var_b) {
      speed_ratio = 1.0f - (abs_err - me->cfg.i_var_a) / me->cfg.i_var_b;
    } else {
      speed_ratio = 0;   // 误差太大, 停止积分
    }
  }

  // 积分分离: |Err| 超过阈值时清零积分并暂停, 防止大扰动下积分饱和
  float i_out;
  if (me->cfg.i_sep_threshold == 0 || abs_err < me->cfg.i_sep_threshold) {
    me->integrator += speed_ratio * base->dt * error;
    // 积分输出限幅: 限制 I 项贡献幅值
    if (me->cfg.i_limit > 0) {
      math_constrain_f(&me->integrator,
                       -(me->cfg.i_limit / (me->cfg.ki + 1e-6f)),
                        me->cfg.i_limit / (me->cfg.ki + 1e-6f));
    }
    i_out = me->cfg.ki * me->integrator;
  } else {
    me->integrator = 0;
    i_out = 0;
  }

  // 4. D 项 —— 路径分离 (2-DOF):
  //    微分先行: D 只对测量值求导 (避免目标阶跃引发 D 冲击)
  //    标准 D:    D 对误差求导
  float d_out;
  if (me->cfg.d_on_measurement) {
    // 微分先行: D 作用于反馈路径, 不响应目标跳变
    d_out = -me->cfg.kd * (measure - me->prev_measure) / base->dt;
  } else {
    // 标准 D: 误差变化率
    d_out = me->cfg.kd * (error - me->prev_error) / base->dt;
  }

  // 5. 前馈: 目标变化 × Kf, 走前向捷径, 不等反馈误差产生
  float f_out = me->cfg.kf * (target - me->prev_target) / base->dt;

  // 6. 保存历史 (供下拍 D/F 项使用)
  me->prev_measure = measure;
  me->prev_error   = error;
  me->prev_target  = target;

  // 7. 返回原始输出 (限幅 + 抗饱和由基类 pid_compute 统一处理)
  return p_out + i_out + d_out + f_out;
}

// 清零: 重置积分器 + 历史状态
static void std_reset(PidBase *base) {
  PidStandard *me = container_of(base, PidStandard, base);
  me->integrator   = 0;
  me->prev_error   = 0;
  me->prev_measure = 0;
  me->prev_target  = 0;
}

// 抗积分饱和: 反算积分器, 同步虚拟状态与物理现实
// raw = 限幅前计算值, clamped = 限幅后输出值
static void std_on_saturation(PidBase *base, float raw, float clamped) {
  PidStandard *me = container_of(base, PidStandard, base);
  if (me->cfg.ki != 0) {
    // 反算: 将截断量折算回积分器, 让积分值跟着真实输出走
    float back_calc = (clamped - raw) / me->cfg.ki;
    me->integrator += back_calc;

    // 二次积分限幅 (安全兜底)
    if (me->cfg.i_limit > 0) {
      math_constrain_f(&me->integrator,
                       -(me->cfg.i_limit / me->cfg.ki),
                        me->cfg.i_limit / me->cfg.ki);
    }
  }
}

static const PidOps std_ops = {
  .compute       = std_compute,
  .reset         = std_reset,
  .on_saturation = std_on_saturation,
};

// ======== 构造 ========

void pid_std_init(PidStandard *me, float dt, float out_min, float out_max,
                  const PidStdConfig *cfg) {
  pid_base_init(&me->base);
  me->cfg          = *cfg;
  me->integrator   = 0;
  me->prev_measure = 0;
  me->prev_error   = 0;
  me->prev_target  = 0;

  me->base.dt          = dt;
  me->base.out_min     = out_min;
  me->base.out_max     = out_max;
  me->base.anti_windup = true;   // 标准 PID 默认启用抗饱和
  me->base.ops         = &std_ops;
}

// ======== 运行时调参 ========

void pid_std_update_config(PidStandard *me, const PidStdConfig *cfg) {
  me->cfg = *cfg;
}

void pid_std_set_kp(PidStandard *me, float kp) { me->cfg.kp = kp; }
void pid_std_set_ki(PidStandard *me, float ki) { me->cfg.ki = ki; }
void pid_std_set_kd(PidStandard *me, float kd) { me->cfg.kd = kd; }
void pid_std_set_kf(PidStandard *me, float kf) { me->cfg.kf = kf; }
void pid_std_set_i_limit(PidStandard *me, float v) { me->cfg.i_limit = v; }
void pid_std_set_deadzone(PidStandard *me, float v) { me->cfg.deadzone = v; }
void pid_std_set_d_on_measurement(PidStandard *me, bool on) { me->cfg.d_on_measurement = on; }
