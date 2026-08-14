// 电机控制 — 异步电机磁链估计器 (ACI Flux Estimator)
//
// 来源: TI controlSUITE motor_control/math_blocks/v4.3 (aci_fe.h, aci_fe_const.h)
// 翻译为 HardC 纯C float 版本
//
// 算法 (电流模型 + 电压模型 PI 融合):
//   电流模型: 从 Id 估计转子磁链 → FluxDrE = K1×FluxDrE + K2×IDsE
//   电压模型: 从反电动势积分 → Psi_s = ∫(U_s - Rs×I_s)
//   PI 补偿: 电压模型 → 电流模型差 → PI → 补偿 → 最终磁链
//   θ = atan2(PsiQrS, PsiDrS)
//
// 注: 这是 35 字段的复杂结构体, 不适合 static inline — 用常规函数

#ifndef COMP_ACI_FE_H
#define COMP_ACI_FE_H

#include <math.h>

// ======================= AciFeConst (物理参数 → 算法系数) =======================

typedef struct {
  // 输入物理参数
  float rs;               // 定子电阻 (Ω)
  float rr;               // 转子电阻 (Ω)
  float ls;               // 定子电感 (H)
  float lr;               // 转子电感 (H)
  float lm;               // 互感 (H)
  float ib;               // 电流基值 (A)
  float vb;               // 电压基值 (V)
  float ts;               // 采样周期 (s)

  // 计算得到的系数
  float tr;               // 转子时间常数 = Lr/Rr
  float k1;               // 电流模型一阶 LPF 反馈 = Tr/(Tr+Ts)
  float k2;               // 电流模型一阶 LPF 输入 = Ts/(Tr+Ts)
  float k3;               // 磁链换算 = Lm/Lr
  float k4;               // 漏感系数 = σLs
  float k5;               // 电阻压降定标 = Ib×Rs/Vb
  float k6;               // 反电动势积分定标 = Vb×Ts/(Lm×Ib)
  float k7;               // 转子磁链换算 = Lr/Lm
  float k8;               // 转子磁链换算 2 = σLs/Lm
} AciFeConst;

#define ACI_FE_CONST_DEFAULTS { 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0 }

// 从电机参数和采样周期计算所有系数
static inline void aci_fe_const_calc(AciFeConst *c) {
  c->tr = c->lr / c->rr;
  c->k1 = c->tr / (c->tr + c->ts);
  c->k2 = c->ts / (c->tr + c->ts);
  c->k3 = c->lm / c->lr;
  c->k4 = (c->ls * c->lr - c->lm * c->lm) / (c->lr * c->lm);
  c->k5 = c->ib * c->rs / c->vb;
  c->k6 = c->vb * c->ts / (c->lm * c->ib);
  c->k7 = c->lr / c->lm;
  c->k8 = (c->ls * c->lr - c->lm * c->lm) / (c->lm * c->lm);
}

// ======================= AciFe (异步电机磁链估计器) =======================

typedef struct {
  // 输入
  float i_qs_s;           // 静止 α 轴定子电流 (q, 标幺)
  float i_ds_s;           // 静止 β 轴定子电流 (d, 标幺)
  float u_ds_s;           // 静止 d 轴定子电压 (标幺)
  float u_qs_s;           // 静止 q 轴定子电压 (标幺)

  // 电流模型状态
  float i_ds_e;           // 同步旋转 d 轴电流
  float flux_dr_e;        // 同步旋转 d 轴转子磁链

  // 电流模型输出
  float flux_dr_s;        // 静止 d 轴转子磁链
  float flux_qr_s;        // 静止 q 轴转子磁链
  float flux_ds_s;        // 静止 d 轴定子磁链
  float flux_qs_s;        // 静止 q 轴定子磁链

  // 电压模型状态
  float psi_ds_s;         // 静止 d 轴定子磁链 (电压模型积分)
  float psi_qs_s;         // 静止 q 轴定子磁链 (电压模型积分)
  float emf_ds_s;         // 静止 d 轴反电动势
  float emf_qs_s;         // 静止 q 轴反电动势
  float old_emf_d;        // 旧反电动势 (梯形积分用)
  float old_emf_q;

  // PI 补偿器状态
  float error_d;          // d 轴误差 (电压模型 - 电流模型)
  float error_q;          // q 轴误差
  float ui_ds_s;          // 静止 d 轴积分项
  float ui_qs_s;          // 静止 q 轴积分项
  float u_comp_ds_s;      // 静止 d 轴补偿电压
  float u_comp_qs_s;      // 静止 q 轴补偿电压

  // 输出
  float psi_dr_s;         // 静止 d 轴估计转子磁链
  float psi_qr_s;         // 静止 q 轴估计转子磁链
  float theta_flux;       // 转子磁链角 (rad)

  // 参数
  float kp;               // PI 比例增益
  float ki;               // PI 积分增益
  float sin_val;          // sin(θ)
  float cos_val;          // cos(θ)

  // 系数集 (由 aci_fe_const_calc 计算)
  const AciFeConst *coeff;
} AciFe;

#define ACI_FE_DEFAULTS { 0,0,0,0, 0,0, 0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0, 1,0, 0,0, 0 }

// 初始化磁链估计器
void aci_fe_init(AciFe *me, const AciFeConst *coeff, float kp, float ki);

// 单步运行 (ISR 中每控制周期调用)
//   返回: 转子磁链角度 theta_flux (rad)
float aci_fe_run(AciFe *me);

// 重置
void aci_fe_reset(AciFe *me);

#endif  // COMP_ACI_FE_H
