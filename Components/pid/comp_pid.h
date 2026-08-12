#ifndef COMP_PID_H
#define COMP_PID_H

// PID 平台层 —— 极简基类: 只管理时钟 + 输出物理限幅 + 抗积分饱和调度
// 所有子类通过 PidOps 虚表绑定, 基类不假设任何状态变量 (误差/历史等)
// 统一入口 pid_compute(target, measure) 内部完成 计算 → 限幅 → 抗饱和回调

#include <stdint.h>
#include <stdbool.h>

// ======== 前向声明 ========
typedef struct PidBase PidBase;

// ======== 虚函数签名 (子类各自实现) ========

// 计算原始控制量 (子类内部执行 P/I/D/前馈 等所有算法)
typedef float (*pid_compute_fn)(PidBase *base, float target, float measure);

// 清零内部状态 (积分累加 / 历史误差, 用于模式切换 / 急停)
typedef void  (*pid_reset_fn)(PidBase *base);

// 抗积分饱和回调: 当 pid_compute 检测到输出被物理限幅截断时调用
// raw=限幅前, clamped=限幅后, 子类据此回退积分器状态
typedef void  (*pid_on_saturation_fn)(PidBase *base, float raw, float clamped);

// ======== 虚函数表 ========
typedef struct {
  pid_compute_fn        compute;
  pid_reset_fn          reset;
  pid_on_saturation_fn  on_saturation;
} PidOps;

// ======== 基类结构体 —— 只包含执行器物理契约 ========
struct PidBase {
  const PidOps *ops;
  float  dt;            // 控制周期 (秒), 如 0.01 = 10ms
  float  out_min;       // 执行器物理下限 (如 0V / 0% PWM)
  float  out_max;       // 执行器物理上限 (如 3.3V / 100% PWM)
  bool   anti_windup;   // true = 输出截断时自动回调 on_saturation
};

// ======== 基类构造 ========

// 初始化基类默认字段, ops 由子类 init 时绑定
void pid_base_init(PidBase *base);

// ======== 统一对外接口 (inline, 零开销) ========

// 单次控制周期入口: 设目标+测反馈 → 子类计算原始输出 → 限幅 → 抗饱和
// 返回限幅后的最终输出, 可直接写入执行器寄存器
static inline float pid_compute(PidBase *base, float target, float measure) {
  float raw = base->ops->compute(base, target, measure);
  float clamped = raw;

  // 输出限幅 (物理层约束)
  if (clamped > base->out_max) clamped = base->out_max;
  else if (clamped < base->out_min) clamped = base->out_min;

  // 抗积分饱和: 仅当使能且被截断时回调子类
  if (base->anti_windup && (raw != clamped) && base->ops->on_saturation) {
    base->ops->on_saturation(base, raw, clamped);
  }

  return clamped;
}

// 清零内部状态 (委托子类 reset)
static inline void pid_reset(PidBase *base) {
  if (base->ops->reset) {
    base->ops->reset(base);
  }
}

#endif
