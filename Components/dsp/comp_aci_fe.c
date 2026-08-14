// 异步电机磁链估计器实现
//
// 来源: TI controlSUITE ACIFE_MACRO (motor_control/math_blocks/v4.3)
// 翻译为 HardC 纯C float 版本

#include "comp_aci_fe.h"
#include <string.h>

void aci_fe_init(AciFe *me, const AciFeConst *coeff, float kp, float ki) {
  memset(me, 0, sizeof(AciFe));
  me->coeff = coeff;
  me->kp = kp;
  me->ki = ki;
}

float aci_fe_run(AciFe *me) {
  const AciFeConst *c = me->coeff;

  // ---- 阶段 1: sin/cos 预计算 ----
  me->sin_val = sinf(me->theta_flux);
  me->cos_val = cosf(me->theta_flux);

  // ---- 阶段 2: Park 变换 (静止 → 同步) ----
  // 实测电流从静止 αβ → 同步旋转 d 轴
  me->i_ds_e = me->i_qs_s * me->sin_val + me->i_ds_s * me->cos_val;

  // ---- 阶段 3: 电流模型 (转子磁链一阶 LPF) ----
  me->flux_dr_e = c->k1 * me->flux_dr_e + c->k2 * me->i_ds_e;

  // ---- 阶段 4: 反Park (同步 → 静止) — 转子磁链 ----
  me->flux_dr_s = me->flux_dr_e * me->cos_val;
  me->flux_qr_s = me->flux_dr_e * me->sin_val;

  // ---- 阶段 5: 定子磁链 (从转子磁链 + 漏感) ----
  me->flux_ds_s = c->k3 * me->flux_dr_s + c->k4 * me->i_ds_s;
  me->flux_qs_s = c->k3 * me->flux_qr_s + c->k4 * me->i_qs_s;

  // ---- 阶段 6: PI 补偿 (电压模型 ← 电流模型差) — d 轴 ----
  float kp = me->kp;
  float ki = me->ki;

  me->error_d = me->psi_ds_s - me->flux_ds_s;
  // 并行 PI: Ucomp = Kp×err + Ui,  Ui += Kp×Ki×err
  me->u_comp_ds_s = kp * me->error_d + me->ui_ds_s;
  me->ui_ds_s += kp * ki * me->error_d;

  // ---- 阶段 7: PI 补偿 — q 轴 ----
  me->error_q = me->psi_qs_s - me->flux_qs_s;
  me->u_comp_qs_s = kp * me->error_q + me->ui_qs_s;
  me->ui_qs_s += kp * ki * me->error_q;

  // ---- 阶段 8: 反电动势 + 电压模型积分 (d 轴) ----
  // Emf = U - Ucomp - Rs×I
  // Psi_s = ∫Emf (梯形积分)
  me->old_emf_d = me->emf_ds_s;
  me->emf_ds_s = me->u_ds_s - me->u_comp_ds_s - c->k5 * me->i_ds_s;
  me->psi_ds_s += c->k6 * 0.5f * (me->emf_ds_s + me->old_emf_d);

  // ---- 阶段 9: 反电动势 + 电压模型积分 (q 轴) ----
  me->old_emf_q = me->emf_qs_s;
  me->emf_qs_s = me->u_qs_s - me->u_comp_qs_s - c->k5 * me->i_qs_s;
  me->psi_qs_s += c->k6 * 0.5f * (me->emf_qs_s + me->old_emf_q);

  // ---- 阶段 10: 转子磁链 (从电压模型定子磁链) ----
  me->psi_dr_s = c->k7 * me->psi_ds_s - c->k8 * me->i_ds_s;
  me->psi_qr_s = c->k7 * me->psi_qs_s - c->k8 * me->i_qs_s;

  // ---- 阶段 11: 转子磁链角度 ----
  me->theta_flux = atan2f(me->psi_qr_s, me->psi_dr_s);

  return me->theta_flux;
}

void aci_fe_reset(AciFe *me) {
  float kp_saved = me->kp;
  float ki_saved = me->ki;
  const AciFeConst *c = me->coeff;
  aci_fe_init(me, c, kp_saved, ki_saved);
}
