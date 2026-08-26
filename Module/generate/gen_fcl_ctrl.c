// 快速电流环 (Fast Current Loop) — dq 旋转坐标系高带宽电流控制实现
//
// 来源: TI controlSUITE motor_control/libs/FCL
// 翻译为 HardC Module 层纯C float 版本
//
// 算法细节:
//   v_d_ref = PI(i_d_ref - i_d) - ω·Lq·i_q                      ← d 轴 PI + 解耦
//   v_q_ref = PI(i_q_ref - i_q) + ω·(Ld·i_d + ψ_pm)              ← q 轴 PI + 解耦 + BEMF
//   有源阻尼 (可选): v_d_damp = -kp_damp·ω·i_q, v_q_damp = +kp_damp·ω·i_d
//   输出限幅: 电压圆限制 |v| ≤ v_dc_max, 等比例缩放
//
// 数值考虑:
//   - 积分器钳位: dq 轴 PI 的 i_limit 每 tick 设为 min(v_bus, v_dc_max), 防止 windup
//   - 过流检测用去抖计数 (5 次), 避免噪声误触发
//   - 电压圆限制用等比例缩放, 保证相位不变
//
// 迁移说明 (手搓 PI → PidLinear → PidParallel):
//   旧实现是内联积分 integral += ki·err·dt + 积分钳位 ±v_limit, 输出不钳 (电压圆在下方)。
//   映射: kp=cfg.kp_d/ki_q, i_limit=v_limit (动态, 每 tick 写), out_min/max 宽限 (无 PI 输出钳位),
//   aw=CLAMP (宽限幅下无门控/无回退, 积分只受 i_limit 钳位)。累加式 (ki·dt)·err 与旧式
//   (ki·err)·dt 仅末位舍入差 (IEEE 结合律), 属配置等价; FCL 零消费者, 无位级对拍要求。
//   PidParallel 对应: i_limit=v_limit (每 tick 写), base.out_min/max=±1e9 宽限 (pid_compute
//   恒不限幅 → 抗饱和回调不触发, 与旧 ±1e6 相同), kd=0, k=1, d_on_measurement=false。

#include "gen_fcl_ctrl.h"
#include <math.h>
#include "comp_math.h"

#define FCL_OVERCURRENT_DEBOUNCE 5  // 过流去抖次数

// ======== 初始化 & 重置 ========

void fcl_init(FclCtrl *me, const FclCfg *cfg) {
  me->cfg = *cfg;

  // dq 轴电流环 PI (PidParallel): 宽输出限幅 (电压圆在 fcl_run, 不是 PI 抗饱和), 增益由 FclCfg 派生
  PidParallelConfig pc;
  pc.kp = me->cfg.kp_d;
  pc.ki = me->cfg.ki_d;
  pc.kd = 0.0f;
  pc.k = 1.0f;
  pc.i_limit = 0.0f;  // 每 tick 由 fcl_run 动态写为 min(v_bus, v_dc_max)
  pc.d_on_measurement = false;
  pid_parallel_init(&me->pi_d, me->cfg.dt, -1e9f, 1e9f, &pc);
  pc.kp = me->cfg.kp_q;
  pc.ki = me->cfg.ki_q;
  pid_parallel_init(&me->pi_q, me->cfg.dt, -1e9f, 1e9f, &pc);

  fcl_reset(me);
}

void fcl_reset(FclCtrl *me) {
  me->mode = FclMode_Idle;
  pid_reset(&me->pi_d.base);
  pid_reset(&me->pi_q.base);
  me->v_d_ref = 0.0f;
  me->v_q_ref = 0.0f;
  me->i_d_meas = 0.0f;
  me->i_q_meas = 0.0f;
  me->i_d_ref = 0.0f;
  me->i_q_ref = 0.0f;
  me->fault_code = 0;
  me->overcurrent_cnt = 0;
}

// ======== 模式控制 ========

void fcl_enable(FclCtrl *me) {
  // 清积分器, 避免启动瞬态
  pid_reset(&me->pi_d.base);
  pid_reset(&me->pi_q.base);
  me->mode = FclMode_Enabled;
}

void fcl_disable(FclCtrl *me) {
  me->mode = FclMode_Idle;
}

// ======== ISR 快速路径 ========

void fcl_run(FclCtrl *me, float i_d, float i_q, float i_d_ref, float i_q_ref, float omega_e, float v_bus) {
  // 1. 缓存测量值 (调试可见)
  me->i_d_meas = i_d;
  me->i_q_meas = i_q;
  me->i_d_ref = i_d_ref;
  me->i_q_ref = i_q_ref;

  // 故障锁定: 不清零输出, 保持上一次值
  if (me->mode == FclMode_Fault) {
    return;
  }

  // 空闲模式: 输出 0
  if (me->mode == FclMode_Idle) {
    me->v_d_ref = 0.0f;
    me->v_q_ref = 0.0f;
    pid_reset(&me->pi_d.base);
    pid_reset(&me->pi_q.base);
    return;
  }

  // 2. 过流检测 (去抖)
  if (MATH_ABS(i_d) > me->cfg.i_max || MATH_ABS(i_q) > me->cfg.i_max) {
    me->overcurrent_cnt++;
    if (me->overcurrent_cnt >= FCL_OVERCURRENT_DEBOUNCE) {
      me->mode = FclMode_Fault;
      me->fault_code = 1;  // bit0 = 过流
      me->v_d_ref = 0.0f;
      me->v_q_ref = 0.0f;
      pid_reset(&me->pi_d.base);
      pid_reset(&me->pi_q.base);
      return;
    }
  } else {
    me->overcurrent_cnt = 0;
  }

  // 3. dq 轴电流环 PI (积分器钳位 = 动态电压限, 输出不限幅 — 电压圆在 step 8)
  float v_limit = (v_bus < me->cfg.v_dc_max) ? v_bus : me->cfg.v_dc_max;
  me->pi_d.cfg.i_limit = v_limit;
  me->pi_q.cfg.i_limit = v_limit;
  float v_d_pi = pid_compute(&me->pi_d.base, i_d_ref, i_d);
  float v_q_pi = pid_compute(&me->pi_q.base, i_q_ref, i_q);

  // 5. dq 交叉解耦 + 反电动势前馈
  float v_d_ff = -omega_e * me->cfg.lq * i_q;  // d 轴解耦: -ω·Lq·i_q
  float v_q_ff = omega_e * me->cfg.ld * i_d    // q 轴解耦: +ω·Ld·i_d
                 + omega_e * me->cfg.flux_pm;  // 反电动势: +ω·ψ_pm

  // 6. 有源阻尼 (可选, kp_damp > 0 时生效)
  float v_d_damp = 0.0f;
  float v_q_damp = 0.0f;
  if (me->cfg.kp_damp > 0.0f) {
    v_d_damp = -me->cfg.kp_damp * omega_e * i_q;  // 高频振荡抑制
    v_q_damp = me->cfg.kp_damp * omega_e * i_d;
  }

  // 7. 合成输出电压
  float vd = v_d_pi + v_d_ff + v_d_damp;
  float vq = v_q_pi + v_q_ff + v_q_damp;

  // 8. 电压圆限制: |v| ≤ v_limit (动态取 v_bus 和 cfg.v_dc_max 较小者, 等比例缩放)
  float v_mag_sq = vd * vd + vq * vq;
  float v_limit_sq = v_limit * v_limit;

  if (v_mag_sq > v_limit_sq) {
    float scale = v_limit / MATH_SQRT(v_mag_sq);
    vd *= scale;
    vq *= scale;
  }

  me->v_d_ref = vd;
  me->v_q_ref = vq;
}

// ======== 故障管理 ========

void fcl_clear_fault(FclCtrl *me) {
  me->fault_code = 0;
  me->overcurrent_cnt = 0;
  me->mode = FclMode_Idle;
}

uint16_t fcl_get_fault(const FclCtrl *me) {
  return me->fault_code;
}
