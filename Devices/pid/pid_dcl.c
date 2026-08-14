// DCL 风格二自由度 PID 实现
//
// 来源: TI controlSUITE DCL_PID.asm (C2000 手写汇编)
// 翻译为 HardC 纯C float 版本
//
// 算法流程图:
//   输入: rk(目标), yk(反馈), lk(外部抗饱和增益, 默认1.0)
//
//   微分支路 (微分先行 — 只对反馈求导):
//     v1 = Kd * c1 * yk           ← 输入缩放
//     v4 = v1 - d2 - d3           ← 一阶低通差分
//     d2 = v1                      ← 保存输入延迟
//     d3 = c2 * v4                 ← 保存反馈延迟
//
//   比例支路 (2-DOF 设定点权重):
//     v5 = Kr * rk - yk           ← 加权误差
//     v5 = v5 - v4                 ← 减 D 项贡献 (v4 已经是滤波后的"微分"量)
//     v6 = Kp * v5                 ← P 项输出
//
//   积分支路 (乘法型抗饱和):
//     v7 = Ki * Kp * (rk - yk)    ← 原始积分增量 (用原始误差, 不经 Kr 加权)
//     v7 = v7 * i14               ← 乘以饱和度标记 (饱和→0, 冻结积分)
//     i_storage = i_storage + v7   ← 累加
//
//   输出合成 + 抗饱和:
//     v9 = v6 + i_storage          ← P + I (+ 隐含的 D 在 v5 中)
//     output = clamp(v9, out_min, out_max)
//     i14 = (v9 == output) ? 1.0f : 0.0f   ← 更新饱和度标记

#include "pid_dcl.h"
#include "container_of.h"
#include <math.h>
#include "comp_math.h"

// ======== 内部辅助 ========

static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// ======== ops 实现 ========

// DCL PidOps::compute — 按 DCL_PID.asm 的 5 阶段算法
static float dcl_compute(PidBase *base, float target, float measure) {
  PidDcl *me = container_of(base, PidDcl, base);

  float kp = me->cfg.kp;
  float ki = me->cfg.ki;
  float kd = me->cfg.kd;
  float kr = me->cfg.kr;
  float c1 = me->cfg.c1;
  float c2 = me->cfg.c2;

  // ---- 阶段 1: 微分支路 (滤波微分, 只对反馈 yk 求导) ----
  float v1 = kd * c1 * measure;          // 缩放反馈
  float v4 = v1 - me->d2 - me->d3;       // 一阶 IIR 低通差分
  me->d2 = v1;                            // 保存输入延迟
  me->d3 = c2 * v4;                       // 保存反馈延迟 (c2*v4 = d存储)

  // ---- 阶段 2: 比例支路 (2-DOF 设定点权重) ----
  float v5 = kr * target - measure;       // 加权误差 (Kr=0→I-PD, Kr=1→标准PID)
  v5 = v5 - v4;                            // 减微分贡献 (v4含负号, 实际加阻尼)
  float v6 = kp * v5;                      // P 项

  // ---- 阶段 3: 积分支路 (乘法型抗饱和) ----
  float v7 = ki * kp * (target - measure); // 积分增量 (用原始误差)
  v7 = v7 * me->i14;                        // 饱和冻结: i14=0时v7=0, 积分停止
  me->i_storage += v7;                      // 累积

  // 重置 i14 为本拍的乐观预设 (下拍默认允许积分)
  // 若本拍饱和, on_saturation 将覆盖 i14=0 冻结下拍积分
  me->i14 = 1.0f;

  // ---- 阶段 4: 输出合成 + 限幅 ----
  float v9 = v6 + me->i_storage;           // P + I (D 已隐含在 v5 中)
  me->prev_ref = target;
  me->prev_fbk = measure;

  return v9;  // 返回原始输出, 基类负责限幅 + 抗饱和回调
}

// DCL PidOps::reset — 清零积分器和滤波器状态
static void dcl_reset(PidBase *base) {
  PidDcl *me = container_of(base, PidDcl, base);
  me->i_storage = 0.0f;
  me->d2 = 0.0f;
  me->d3 = 0.0f;
  me->i14 = 1.0f;  // 默认允许积分
  me->prev_ref = 0.0f;
  me->prev_fbk = 0.0f;
}

// DCL PidOps::on_saturation — 乘法型抗饱和 (冻结不解算)
// 本拍饱和 → 下拍冻结积分 (i14=0)
static void dcl_on_saturation(PidBase *base, float raw, float clamped) {
  (void)raw; (void)clamped;
  PidDcl *me = container_of(base, PidDcl, base);
  me->i14 = 0.0f;
}

// ======== 虚表 ========
static const PidOps dcl_ops = {
  .compute       = dcl_compute,
  .reset         = dcl_reset,
  .on_saturation = dcl_on_saturation,
};

// ======== 构造 ========

void pid_dcl_init(PidDcl *me, float dt, float out_min, float out_max,
                  const PidDclConfig *cfg) {
  pid_base_init(&me->base);

  me->base.dt           = dt;
  me->base.out_min      = out_min;
  me->base.out_max      = out_max;
  me->base.anti_windup  = true;       // DCL 默认开启抗饱和
  me->base.ops          = &dcl_ops;

  me->i_storage = 0.0f;
  me->d2        = 0.0f;
  me->d3        = 0.0f;
  me->i14       = 1.0f;              // 初始允许积分
  me->prev_ref  = 0.0f;
  me->prev_fbk  = 0.0f;

  if (cfg) {
    me->cfg = *cfg;
  }
}

void pid_dcl_update_config(PidDcl *me, const PidDclConfig *cfg) {
  // 不重置积分器, 只替换系数
  float storage = me->i_storage;
  float d2 = me->d2;
  float d3 = me->d3;

  me->cfg = *cfg;

  // 恢复状态 (系数变了但状态不变)
  me->i_storage = storage;
  me->d2 = d2;
  me->d3 = d3;
}

// ======== 运行时调参 ========

void pid_dcl_set_kp(PidDcl *me, float kp) {
  me->cfg.kp = kp;
}

void pid_dcl_set_ki(PidDcl *me, float ki) {
  me->cfg.ki = ki;
}

void pid_dcl_set_kd(PidDcl *me, float kd) {
  me->cfg.kd = kd;
}

void pid_dcl_set_kr(PidDcl *me, float kr) {
  // Kr: 0=I-PD (P只对反馈), 1=标准PID
  me->cfg.kr = clampf(kr, 0.0f, 1.0f);
}

void pid_dcl_set_dfilt_freq(PidDcl *me, float fc_hz) {
  // 一阶低通: H(s) = 1/(τs+1) 其中 τ = 1/(2πfc)
  // 双线性: c1 = 2πfc·T/(1+2πfc·T), c2 = 1/(1+2πfc·T)
  float dt = me->base.dt;
  float wc_t = M_2PI * fc_hz * dt;   // ωc · T (实际用 2π, 即 wc*T = 2πfc·T)
  float den = 1.0f / (1.0f + wc_t);

  me->cfg.c1 = wc_t * den;   // = wc·T / (1 + wc·T)
  me->cfg.c2 = den;          // = 1 / (1 + wc·T)
}
