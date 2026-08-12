// 电机控制 — 异步电机转差法转速估计器 (ACI Speed Estimator)
//
// 来源: TI controlSUITE motor_control/math_blocks/v4.3 (aci_se.h, aci_se_const.h)
// 翻译为 C-OOP 纯C float 版本
//
// 与 comp_aci_fe (磁链估计器) 配对使用: aci_fe 估计转子磁链 → aci_se 从磁链和电流估计转速
//
// 算法 (转差法, 感应电机无传感器转速估计):
//   1. 转差速度: WSlip = K1 × (PsiDr×IQs − PsiQr×IDs) / |Psi|²
//   2. 同步转速: WSyn = K2 × d(ThetaFlux)/dt  (磁链角微分, 带边界保护)
//   3. 低通滤波: WPsi = K3×WPsi + K4×WSyn
//   4. 转子转速: WrHat = WPsi − WSlip (pu), 饱和到 [-1, 1]
//   5. 转速输出: WrHatRpm = BaseRpm × WrHat
//
// 所有角度/速度均为标幺 (pu): 角度 0~1 对应 0~2π, 速度 0~1 对应基频

#ifndef COMP_ACI_SE_H
#define COMP_ACI_SE_H

// ======================= 常数 (物理参数 → 算法系数) =======================

typedef struct {
  // 输入物理参数
  float rr;               // 转子电阻 (Ω)
  float lr;               // 转子电感 (H)
  float fb;               // 基波电频率 (Hz)
  float fc;               // 低通滤波截止频率 (Hz)
  float ts;               // 采样周期 (s)

  // 计算得到的系数
  float tr;               // 转子时间常数 (s) = Lr/Rr
  float wb;               // 基波角速度 (rad/s) = 2π·fb
  float tc;               // 低通时间常数 (s) = 1/(2π·fc)
  float k1;               // 转差速度系数 = 1/(Wb·Tr)
  float k2;               // 同步速度系数 (微分) = 1/(fb·Ts)
  float k3;               // 低通反馈系数 = Tc/(Tc+Ts)
  float k4;               // 低通输入系数 = Ts/(Tc+Ts)
} AciSeConst;

// 从电机参数和采样周期计算所有系数
static inline void aci_se_const_calc(AciSeConst *c) {
  const float two_pi = 6.28318530718f;

  c->tr = c->lr / c->rr;                      // 转子时间常数
  c->tc = 1.0f / (two_pi * c->fc);            // 低通时间常数
  c->wb = two_pi * c->fb;                     // 基波角速度
  c->k1 = 1.0f / (c->wb * c->tr);
  c->k2 = 1.0f / (c->fb * c->ts);
  c->k3 = c->tc / (c->tc + c->ts);
  c->k4 = c->ts / (c->tc + c->ts);
}

// ======================= 转速估计器 =======================

// 磁链角微分有效区间 — 角度标幺到 [0,1] 时, 跨越 0/1 边界处差分无效
#define ACI_SE_DIFF_MAX_LIMIT  0.80f
#define ACI_SE_DIFF_MIN_LIMIT  0.20f

typedef struct {
  // 系数 (由 aci_se_const_calc 计算)
  AciSeConst coeff;

  // 参数
  float base_rpm;         // 额定转速 (rpm, 转速输出标幺基准)

  // 状态
  float squared_psi;      // 转子磁链模平方 |Psi|²
  float old_theta_flux;   // 上一拍转子磁链角 (pu)
  float wpsi;             // 滤波后同步转速 (pu)
  float w_slip;           // 转差速度 (pu)
  float w_syn;            // 同步转速 (pu)

  // 输出
  float wr_hat;           // 估计转速 (pu, -1~1)
  float wr_hat_rpm;       // 估计转速 (rpm)
} AciSe;

// 初始化 — base_rpm 为额定转速 (如 1500rpm)
static inline void aci_se_init(AciSe *me, const AciSeConst *coeff, float base_rpm) {
  me->coeff = *coeff;
  me->base_rpm = base_rpm;
  me->squared_psi = 0.0f;
  me->old_theta_flux = 0.0f;
  me->wpsi = 0.0f;
  me->w_slip = 0.0f;
  me->w_syn = 0.0f;
  me->wr_hat = 0.0f;
  me->wr_hat_rpm = 0.0f;
}

// 单步运行 (ISR 中每控制周期调用, 须在 aci_fe_run 之后)
//   参数 (均为标幺):
//     i_qs_s     — 静止 q 轴定子电流
//     i_ds_s     — 静止 d 轴定子电流
//     psi_dr_s   — 静止 d 轴转子磁链 (aci_fe 输出)
//     psi_qr_s   — 静止 q 轴转子磁链 (aci_fe 输出)
//     theta_flux — 转子磁链角 (pu, 0~1 = 0~2π, aci_fe 输出)
//   返回: 估计转速 (rpm)
static inline float aci_se_run(AciSe *me, float i_qs_s, float i_ds_s,
                               float psi_dr_s, float psi_qr_s, float theta_flux) {
  // 1. 转子磁链模平方
  me->squared_psi = psi_dr_s * psi_dr_s + psi_qr_s * psi_qr_s;

  // 2. 转差速度: WSlip = K1 × (PsiDr×IQs − PsiQr×IDs) / |Psi|²
  //    低磁链时转差计算无意义, 输出 0 (防止除零)
  if (me->squared_psi > 1e-9f) {
    me->w_slip = me->coeff.k1 * (psi_dr_s * i_qs_s - psi_qr_s * i_ds_s)
               / me->squared_psi;
  } else {
    me->w_slip = 0.0f;
  }

  // 3. 同步转速: 磁链角差分, 仅在角度处于线性中段时有效
  //    角度在 0 或 1 附近跨越回绕边界, 差分会跳变 → 保持上一拍 WPsi
  if (theta_flux < ACI_SE_DIFF_MAX_LIMIT && theta_flux > ACI_SE_DIFF_MIN_LIMIT) {
    me->w_syn = me->coeff.k2 * (theta_flux - me->old_theta_flux);
  } else {
    me->w_syn = me->wpsi;
  }

  // 4. 同步转速低通滤波 (抑制微分噪声)
  me->wpsi = me->coeff.k3 * me->wpsi + me->coeff.k4 * me->w_syn;

  // 5. 转子转速 = 同步转速 − 转差速度
  me->old_theta_flux = theta_flux;
  me->wr_hat = me->wpsi - me->w_slip;

  // 6. 饱和到 [-1, 1] pu
  if (me->wr_hat > 1.0f) me->wr_hat = 1.0f;
  else if (me->wr_hat < -1.0f) me->wr_hat = -1.0f;

  // 7. 转速换算到 rpm
  me->wr_hat_rpm = me->base_rpm * me->wr_hat;
  return me->wr_hat_rpm;
}

// 重置状态 (如模式切换后重新收敛)
static inline void aci_se_reset(AciSe *me) {
  me->squared_psi = 0.0f;
  me->old_theta_flux = 0.0f;
  me->wpsi = 0.0f;
  me->w_slip = 0.0f;
  me->w_syn = 0.0f;
  me->wr_hat = 0.0f;
  me->wr_hat_rpm = 0.0f;
}

#endif  // COMP_ACI_SE_H
