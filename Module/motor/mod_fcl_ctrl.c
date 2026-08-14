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
//   - 积分器钳位到 [-v_dc_max, +v_dc_max], 防止 windup
//   - 过流检测用去抖计数 (5 次), 避免噪声误触发
//   - 电压圆限制用等比例缩放, 保证相位不变

#include "mod_fcl_ctrl.h"
#include <math.h>
#include "comp_math.h"

#define FCL_OVERCURRENT_DEBOUNCE 5   // 过流去抖次数

// ======== 初始化 & 重置 ========

void fcl_init(FclCtrl *me, const FclCfg *cfg) {
  me->cfg = *cfg;
  fcl_reset(me);
}

void fcl_reset(FclCtrl *me) {
  me->mode             = FclMode_Idle;
  me->pi_d_integral    = 0.0f;
  me->pi_q_integral    = 0.0f;
  me->v_d_ref          = 0.0f;
  me->v_q_ref          = 0.0f;
  me->i_d_meas         = 0.0f;
  me->i_q_meas         = 0.0f;
  me->i_d_ref          = 0.0f;
  me->i_q_ref          = 0.0f;
  me->fault_code       = 0;
  me->overcurrent_cnt  = 0;
}

// ======== 模式控制 ========

void fcl_enable(FclCtrl *me) {
  // 清积分器, 避免启动瞬态
  me->pi_d_integral = 0.0f;
  me->pi_q_integral = 0.0f;
  me->mode = FclMode_Enabled;
}

void fcl_disable(FclCtrl *me) {
  me->mode = FclMode_Idle;
}

// ======== ISR 快速路径 ========

void fcl_run(FclCtrl *me, float i_d, float i_q,
             float i_d_ref, float i_q_ref,
             float omega_e, float v_bus) {
  // 1. 缓存测量值 (调试可见)
  me->i_d_meas = i_d;
  me->i_q_meas = i_q;
  me->i_d_ref  = i_d_ref;
  me->i_q_ref  = i_q_ref;

  // 故障锁定: 不清零输出, 保持上一次值
  if (me->mode == FclMode_Fault) {
    return;
  }

  // 空闲模式: 输出 0
  if (me->mode == FclMode_Idle) {
    me->v_d_ref = 0.0f;
    me->v_q_ref = 0.0f;
    me->pi_d_integral = 0.0f;
    me->pi_q_integral = 0.0f;
    return;
  }

  // 2. 过流检测 (去抖)
  if (MATH_ABS(i_d) > me->cfg.i_max || MATH_ABS(i_q) > me->cfg.i_max) {
    me->overcurrent_cnt++;
    if (me->overcurrent_cnt >= FCL_OVERCURRENT_DEBOUNCE) {
      me->mode        = FclMode_Fault;
      me->fault_code  = 1;  // bit0 = 过流
      me->v_d_ref     = 0.0f;
      me->v_q_ref     = 0.0f;
      me->pi_d_integral = 0.0f;
      me->pi_q_integral = 0.0f;
      return;
    }
  } else {
    me->overcurrent_cnt = 0;
  }

  // 3. d 轴 PI
  float err_d = i_d_ref - i_d;
  me->pi_d_integral += me->cfg.ki_d * err_d * me->cfg.dt;

  // 积分器钳位到 [-v_limit, +v_limit], 取 v_bus 和 cfg.v_dc_max 较小者
  float v_limit = (v_bus < me->cfg.v_dc_max) ? v_bus : me->cfg.v_dc_max;
  if (me->pi_d_integral >  v_limit) me->pi_d_integral =  v_limit;
  if (me->pi_d_integral < -v_limit) me->pi_d_integral = -v_limit;

  float v_d_pi = me->cfg.kp_d * err_d + me->pi_d_integral;

  // 4. q 轴 PI
  float err_q = i_q_ref - i_q;
  me->pi_q_integral += me->cfg.ki_q * err_q * me->cfg.dt;

  // 积分器钳位到 [-v_limit, +v_limit]
  if (me->pi_q_integral >  v_limit) me->pi_q_integral =  v_limit;
  if (me->pi_q_integral < -v_limit) me->pi_q_integral = -v_limit;

  float v_q_pi = me->cfg.kp_q * err_q + me->pi_q_integral;

  // 5. dq 交叉解耦 + 反电动势前馈
  float v_d_ff = -omega_e * me->cfg.lq * i_q;               // d 轴解耦: -ω·Lq·i_q
  float v_q_ff =  omega_e * me->cfg.ld * i_d                // q 轴解耦: +ω·Ld·i_d
                + omega_e * me->cfg.flux_pm;                // 反电动势: +ω·ψ_pm

  // 6. 有源阻尼 (可选, kp_damp > 0 时生效)
  float v_d_damp = 0.0f;
  float v_q_damp = 0.0f;
  if (me->cfg.kp_damp > 0.0f) {
    v_d_damp = -me->cfg.kp_damp * omega_e * i_q;             // 高频振荡抑制
    v_q_damp =  me->cfg.kp_damp * omega_e * i_d;
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
  me->fault_code      = 0;
  me->overcurrent_cnt = 0;
  me->mode            = FclMode_Idle;
}

uint16_t fcl_get_fault(const FclCtrl *me) {
  return me->fault_code;
}
