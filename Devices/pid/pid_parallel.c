// 并行形式 PID 控制器
//
// compute 算法流程 (每次调用依次执行):
//   1. 误差: Err = Target - Measure
//   2. P 项: Kp × Err (独立, 不受 I/D 影响)
//   3. I 项: 积分累加 Err×dt → Ki × ∫Err·dt → 积分项输出限幅
//   4. D 项: 支持微分先行 (2-DOF: D 只作用于反馈路径)
//   5. 总输出: K × (P + I + D)
//   6. 返回原始输出 (限幅+抗饱和由基类 pid_compute 处理)
//
// on_saturation 回调:
//   条件积分抗饱和: 当输出饱和方向与误差方向一致时, 撤销本次积分累积
//   本质是阻止积分器在饱和方向上继续增长, 杜绝积分饱和

#include "pid_parallel.h"
#include "container_of.h"

// ======== ops 实现 ========

// 核心计算: 并行结构 —— 各环节独立, 改 Kp 不影响 I/D 贡献
static float parallel_compute(PidBase *base, float target, float measure) {
  PidParallel *me = container_of(base, PidParallel, base);
  float error = target - measure;

  // 1. P 项: 比例独立响应当前误差
  float p_out = me->cfg.kp * error;

  // 2. I 项: 积分累加 → 乘增益 → 输出限幅
  me->integrator += error * base->dt;
  float i_out = me->cfg.ki * me->integrator;
  if (me->cfg.i_limit > 0) {
    if (i_out > me->cfg.i_limit)       i_out = me->cfg.i_limit;
    else if (i_out < -me->cfg.i_limit) i_out = -me->cfg.i_limit;
  }

  // 3. D 项 —— 路径分离 (2-DOF):
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

  // 4. 保存历史 (供下拍 D 项使用)
  me->prev_error   = error;
  me->prev_measure = measure;

  // 5. 总输出 = 总增益 × (独立 P + 独立 I + 独立 D)
  return me->cfg.k * (p_out + i_out + d_out);
}

// 清零: 重置积分器 + 历史状态
static void parallel_reset(PidBase *base) {
  PidParallel *me = container_of(base, PidParallel, base);
  me->integrator   = 0;
  me->prev_error   = 0;
  me->prev_measure = 0;
}

// 抗积分饱和: 条件积分法
// raw = 限幅前计算值, clamped = 限幅后输出值
// 仅当误差方向与饱和方向一致时撤销本次积分累积, 防止积分器在饱和方向上继续增长
static void parallel_on_saturation(PidBase *base, float raw, float clamped) {
  PidParallel *me = container_of(base, PidParallel, base);
  if (me->cfg.ki != 0) {
    // raw > clamped → 输出被上限截断 (正饱和); raw < clamped → 输出被下限截断 (负饱和)
    // prev_error 已在 compute 中更新为本拍误差, 用于判断误差方向
    if ((raw > clamped && me->prev_error > 0) ||
        (raw < clamped && me->prev_error < 0)) {
      // 误差方向与饱和方向一致 → 撤销本次积分累积, 阻止积分器继续增长
      me->integrator -= me->prev_error * base->dt;
    }
  }
}

static const PidOps parallel_ops = {
  .compute       = parallel_compute,
  .reset         = parallel_reset,
  .on_saturation = parallel_on_saturation,
};

// ======== 构造 ========

void pid_parallel_init(PidParallel *me, float dt, float out_min, float out_max,
                       const PidParallelConfig *cfg) {
  pid_base_init(&me->base);
  me->cfg          = *cfg;
  me->integrator   = 0;
  me->prev_error   = 0;
  me->prev_measure = 0;

  me->base.dt          = dt;
  me->base.out_min     = out_min;
  me->base.out_max     = out_max;
  me->base.anti_windup = true;   // 并行 PID 默认启用抗饱和
  me->base.ops         = &parallel_ops;
}

// ======== 运行时调参 ========

void pid_parallel_update_config(PidParallel *me, const PidParallelConfig *cfg) {
  me->cfg = *cfg;
}

void pid_parallel_set_kp(PidParallel *me, float kp) { me->cfg.kp = kp; }
void pid_parallel_set_ki(PidParallel *me, float ki) { me->cfg.ki = ki; }
void pid_parallel_set_kd(PidParallel *me, float kd) { me->cfg.kd = kd; }
void pid_parallel_set_k(PidParallel *me, float k)   { me->cfg.k  = k;  }
