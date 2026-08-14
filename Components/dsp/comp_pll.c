// 软件锁相环实现 — SOGI-PLL (单相) / SRF-PLL (三相) / NotchPLL / DDSRF-PLL
//
// 来源: TI controlSUITE solar/v1.2/float
//   SPLL_1ph_SOGI_F  → SOGI + QSG 正交积分锁相环 (高性能单相)
//   SPLL_1ph_F       → 陷波型单相 PLL (轻量单相, 无正交发生器)
//   SPLL_3ph_SRF_F   → 三相同步旋锁相环
//   SPLL_3ph_DDSRF_F → 解耦双同步旋锁相环 (电网不平衡鲁棒)
// 翻译为 HardC 纯C float 版本

#include "comp_pll.h"
#include <string.h>   // memset
#include "comp_math.h"

// ======================= SOGI-PLL =======================

// 预计算 SOGI-QSG 系数 (双线性变换离散化)
// 连续传递函数:
//   同相: H_d(s) = k*ω*s / (s² + k*ω*s + ω²)
//   正交: H_q(s) = k*ω²  / (s² + k*ω*s + ω²)
static void sogi_coeff_update(SogiPll *me) {
  float omega = M_2PI * me->fn;       // 角频率 (rad/s)
  float t = me->delta_t;
  float c = 2.0f / t;                 // 双线性变换常数
  float k = me->osg.k;

  // 预计算公共项
  float c2 = c * c;
  float k_w_c = k * omega * c;
  float w2 = omega * omega;

  float den = c2 + k_w_c + w2;        // 分母公共项

  // 同相通道 (带通): b0*(u[k] - u[k-2]) = b0*u[k] + 0*u[k-1] - b0*u[k-2]
  me->osg.b0 = k_w_c / den;
  me->osg.b2 = -me->osg.b0;
  me->osg.a1 = (2.0f * c2 - 2.0f * w2) / den;
  me->osg.a2 = -(c2 - k_w_c + w2) / den;

  // 正交通道 (低通): qb0*u[k] + qb1*u[k-1] + qb2*u[k-2]
  me->osg.qb0 = k * w2 / den;
  me->osg.qb1 = 2.0f * me->osg.qb0;
  me->osg.qb2 = me->osg.qb0;
}

void sogi_pll_init(SogiPll *me, float grid_freq_hz, float dt,
                   float kp, float ki, float k_damp) {
  memset(me, 0, sizeof(SogiPll));

  me->fn = grid_freq_hz;
  me->delta_t = dt;
  me->osg.k = k_damp;

  // 环路滤波器系数 (Tustin 双线性变换)
  // PI(s) = kp + ki/s  → 离散化: y[k] = y[k-1] + B0*e[k] + B1*e[k-1]
  // B0 = kp + ki*T/2,  B1 = -kp + ki*T/2
  me->lpf.b0 = kp + ki * dt * 0.5f;
  me->lpf.b1 = -kp + ki * dt * 0.5f;
  me->lpf.a1 = 0.0f;  // 积分器: 无遗忘因子

  sogi_coeff_update(me);
}

float sogi_pll_run(SogiPll *me, float v_grid) {
  // ---- 阶段 0: 历史移位 ----
  me->u[2] = me->u[1];
  me->u[1] = me->u[0];
  me->u[0] = v_grid;

  // ---- 阶段 1: SOGI-QSG 同相输出 (带通滤波器) ----
  // osg_u[k] = b0*(u[k] - u[k-2]) + a1*osg_u[k-1] + a2*osg_u[k-2]
  float osg_u_new = me->osg.b0 * (me->u[0] - me->u[2])
                  + me->osg.a1 * me->osg_u[1]
                  + me->osg.a2 * me->osg_u[2];

  // ---- 阶段 2: SOGI-QSG 正交输出 (90°移相) ----
  // osg_qu[k] = qb0*u[k] + qb1*u[k-1] + qb2*u[k-2] + a1*osg_qu[k-1] + a2*osg_qu[k-2]
  float osg_qu_new = me->osg.qb0 * me->u[0]
                   + me->osg.qb1 * me->u[1]
                   + me->osg.qb2 * me->u[2]
                   + me->osg.a1 * me->osg_qu[1]
                   + me->osg.a2 * me->osg_qu[2];

  // SOGI 历史移位
  me->osg_u[2] = me->osg_u[1];
  me->osg_u[1] = me->osg_u[0];
  me->osg_u[0] = osg_u_new;

  me->osg_qu[2] = me->osg_qu[1];
  me->osg_qu[1] = me->osg_qu[0];
  me->osg_qu[0] = osg_qu_new;

  // ---- 阶段 3: Park 变换 (αβ → dq, α=osg_u, β=osg_qu) ----
  // q 轴 = cos(θ)*osg_u + sin(θ)*osg_qu  →  锁相误差
  // d 轴 = cos(θ)*osg_qu - sin(θ)*osg_u  →  幅值估计
  me->u_q[1] = me->u_q[0];
  me->u_q[0] = me->cos_val * me->osg_u[0] + me->sin_val * me->osg_qu[0];
  me->u_d    = me->cos_val * me->osg_qu[0] - me->sin_val * me->osg_u[0];

  // ---- 阶段 4: 环路滤波器 (PI 型一阶 IIR) ----
  // ylf[k] = ylf[k-1] + B0*u_Q[k] + B1*u_Q[k-1]
  float ylf_new = me->ylf[1] + me->lpf.b0 * me->u_q[0] + me->lpf.b1 * me->u_q[1];

  me->ylf[1] = me->ylf[0];
  me->ylf[0] = ylf_new;

  // ---- 阶段 5: 压控振荡器 VCO ----
  me->fo = me->fn + ylf_new;

  // 角度积分: θ[k] = θ[k-1] + fo * ΔT * 2π
  me->theta[1] = me->theta[0];
  me->theta[0] = me->theta[1] + me->fo * me->delta_t * M_2PI;

  // 相位折叠 (重置到 0, 与原始 SPLL_1ph_SOGI_F 一致)
  if (me->theta[0] > M_2PI) {
    me->theta[0] = 0.0f;
  }

  // 预计算 sin/cos 供下一周期 Park 变换使用
  me->sin_val = sinf(me->theta[0]);
  me->cos_val = cosf(me->theta[0]);

  return me->fo;
}

void sogi_pll_reset(SogiPll *me) {
  float fn_saved = me->fn;
  float dt_saved = me->delta_t;
  float k_saved = me->osg.k;
  float kp = me->lpf.b0 + me->lpf.b1;            // 还原 Kp = B0 + B1
  float ki = (me->lpf.b0 - me->lpf.b1) / dt_saved; // 还原 Ki = (B0 - B1)/T

  sogi_pll_init(me, fn_saved, dt_saved, kp, ki, k_saved);
}

// ======================= SRF-PLL =======================

void srf_pll_init(SrfPll *me, float grid_freq_hz, float dt,
                  float kp, float ki) {
  memset(me, 0, sizeof(SrfPll));

  me->fn = grid_freq_hz;
  me->delta_t = dt;
  me->freq_lim = 200.0f;   // 默认 ±200 Hz 频率偏差限幅

  // 环路滤波器系数 (同 SOGI-PLL 的 PI 离散化)
  me->b0_lf = kp + ki * dt * 0.5f;
  me->b1_lf = -kp + ki * dt * 0.5f;
}

float srf_pll_run_ab(SrfPll *me, float v_alpha, float v_beta,
                     float *out_sin, float *out_cos) {
  // 步骤 1: Park 变换 (αβ → q) — 内置
  // v_q = v_beta * cos(θ) - v_alpha * sin(θ)
  float sin_t = sinf(me->theta[0]);
  float cos_t = cosf(me->theta[0]);

  *out_sin = sin_t;
  *out_cos = cos_t;

  float v_q_new = v_beta * cos_t - v_alpha * sin_t;
  return srf_pll_run_q(me, v_q_new);
}

float srf_pll_run_q(SrfPll *me, float v_q) {
  // 阶段 1: 环路滤波器 (PI 型一阶 IIR)
  // ylf[k] = ylf[k-1] + B0*v_q[k] + B1*v_q[k-1]
  float ylf_new = me->ylf[1] + me->b0_lf * v_q + me->b1_lf * me->v_q[1];

  // 频率偏差限幅
  if (ylf_new > me->freq_lim) {
    ylf_new = me->freq_lim;
  } else if (ylf_new < -me->freq_lim) {
    ylf_new = -me->freq_lim;
  }

  me->ylf[1] = me->ylf[0];
  me->ylf[0] = ylf_new;

  // 阶段 2: 压控振荡器 VCO
  me->fo = me->fn + ylf_new;

  // 角度积分
  me->theta[1] = me->theta[0];
  me->theta[0] = me->theta[1] + me->fo * me->delta_t * M_2PI;

  // 相位折叠: 用减法保留分数相位 (与原始 SPLL_3ph_SRF_F 一致)
  while (me->theta[0] > M_2PI) {
    me->theta[0] -= M_2PI;
  }

  // 历史移位
  me->v_q[1] = me->v_q[0];
  me->v_q[0] = v_q;

  return me->fo;
}

void srf_pll_reset(SrfPll *me) {
  float fn_saved = me->fn;
  float dt_saved = me->delta_t;
  float kp = me->b0_lf + me->b1_lf;
  float ki = (me->b0_lf - me->b1_lf) / dt_saved;

  srf_pll_init(me, fn_saved, dt_saved, kp, ki);
}

// ======================= 陷波型单相 PLL =======================
//
// 来源: TI SPLL_1ph_F (solar/v1.2/float)
// 机制: 积型鉴相器 → 陷波(2f₀) → PI环路滤波 → 精确离散VCO
// 优势: 比 SOGI-PLL 省 CPU (无正交发生器)

// 计算陷波滤波器系数 (双线性变换, 2 倍频陷波)
static void notch_pll_calc_coeff(NotchPll *me) {
  float dt = me->delta_t;
  float wn = me->wn;                     // 标称角频率 (rad/s)
  float w2 = 2.0f * wn;                  // 陷波频率 = 2×f₀ (100/120Hz)
  float t = dt;
  float c = 2.0f / t;                    // 双线性常数
  float c2 = c * c;
  float w2_sq = w2 * w2;
  float c1_damp = 0.01f;                 // 陷波深度阻尼
  float c2_damp = 0.1f;                  // 陷波带宽阻尼

  float den = c2 + 2.0f * c1_damp * w2 * c + w2_sq;

  // 递归系数
  me->notch_b0 = (c2 + w2_sq) / den;
  me->notch_b1 = (2.0f * w2_sq - 2.0f * c2) / den;
  me->notch_b2 = (c2 + w2_sq) / den;
  me->notch_a1 = (2.0f * c2 - 2.0f * w2_sq) / den;
  me->notch_a2 = (c2 - 2.0f * c1_damp * w2 * c + w2_sq) / den;
}

// 计算环路滤波器系数 (LPF)
static void notch_pll_calc_lpf_coeff(NotchPll *me, float kp, float ki) {
  float dt = me->delta_t;

  // PI 离散化 (Tustin) →
  // LPF 系数: y[k] = b1*y[k-1] + b0*e[k],  其中 b0 = kp+ki*T/2, b1 隐含在差分中
  // 这里用经典形式: ylf += b0*notch_out
  me->lpf_b0 = kp + ki * dt * 0.5f;
  me->lpf_b1 = -kp + ki * dt * 0.5f;
  me->lpf_a1 = 0.0f;
}

void notch_pll_init(NotchPll *me, float grid_freq_hz, float dt,
                    float kp, float ki) {
  memset(me, 0, sizeof(NotchPll));

  me->delta_t = dt;
  me->wn = M_2PI * grid_freq_hz;          // 标称角频率

  // 初始相位
  me->theta[0] = 0.0f;
  me->theta[1] = 0.0f;

  // VCO 初始: cos(0)=1, sin(0)=0
  me->cos_val[0] = 1.0f;
  me->cos_val[1] = 1.0f;
  me->sin_val[0] = 0.0f;
  me->sin_val[1] = 0.0f;
  me->wo = me->wn;

  notch_pll_calc_coeff(me);
  notch_pll_calc_lpf_coeff(me, kp, ki);
}

float notch_pll_run(NotchPll *me, float v_grid) {
  me->ac_input = v_grid;

  // ---- 阶段 1: 积型鉴相器 (product detector) ----
  // upd = AC_input × cos(θ) — 积型检测器输出含 2f₀ 纹波
  me->upd[2] = me->upd[1];
  me->upd[1] = me->upd[0];
  me->upd[0] = v_grid * me->cos_val[0];

  // ---- 阶段 2: 陷波滤波器 (去除 2f₀ 纹波) ----
  // ynotch[k] = b0*upd[k] + b1*upd[k-1] + b2*upd[k-2] + a1*ynotch[k-1] + a2*ynotch[k-2]
  float ynotch_new = me->notch_b0 * me->upd[0]
                   + me->notch_b1 * me->upd[1]
                   + me->notch_b2 * me->upd[2]
                   + me->notch_a1 * me->ynotch[1]
                   + me->notch_a2 * me->ynotch[2];

  me->ynotch[2] = me->ynotch[1];
  me->ynotch[1] = me->ynotch[0];
  me->ynotch[0] = ynotch_new;

  // ---- 阶段 3: 环路滤波器 (PI) ----
  // ylf[k] = ylf[k-1] + b0*ynotch[k] + b1*ynotch[k-1]
  float ylf_new = me->ylf[1] + me->lpf_b0 * me->ynotch[0] + me->lpf_b1 * me->ynotch[1];

  me->ylf[1] = me->ylf[0];
  me->ylf[0] = ylf_new;

  // ---- 阶段 4: VCO (精确离散时间振荡器) ----
  // 比欧拉积分更精确: 使用三角函数加法公式
  // cos(θ[k]) = cos(θ[k-1])·cos(Δθ) - sin(θ[k-1])·sin(Δθ)
  // sin(θ[k]) = sin(θ[k-1])·cos(Δθ) + cos(θ[k-1])·sin(Δθ)
  // 其中 Δθ = (wn + ylf_new) · ΔT
  me->wo = me->wn + ylf_new;
  float delta_theta = me->wo * me->delta_t;

  float cos_delta = cosf(delta_theta);
  float sin_delta = sinf(delta_theta);

  float cos_new = me->cos_val[0] * cos_delta - me->sin_val[0] * sin_delta;
  float sin_new = me->sin_val[0] * cos_delta + me->cos_val[0] * sin_delta;

  // 正交归一化 (防止幅值漂移)
  float mag = MATH_SQRT(cos_new * cos_new + sin_new * sin_new);
  if (mag > 0.0f) {
    cos_new /= mag;
    sin_new /= mag;
  }

  me->cos_val[1] = me->cos_val[0];
  me->cos_val[0] = cos_new;
  me->sin_val[1] = me->sin_val[0];
  me->sin_val[0] = sin_new;

  // 相位 (标幺 0~1)
  me->theta[1] = me->theta[0];
  me->theta[0] = me->theta[1] + delta_theta / M_2PI;
  while (me->theta[0] > 1.0f) {
    me->theta[0] -= 1.0f;
  }

  // 返回当前频率
  return me->wo / M_2PI;
}

void notch_pll_coeff_update(NotchPll *me, float wn) {
  me->wn = wn;
  notch_pll_calc_coeff(me);
}

void notch_pll_reset(NotchPll *me) {
  float grid_hz = me->wn / M_2PI;
  float dt = me->delta_t;
  float kp = me->lpf_b0 + me->lpf_b1;
  float ki = (me->lpf_b0 - me->lpf_b1) / dt;

  notch_pll_init(me, grid_hz, dt, kp, ki);
}

// ======================= DDSRF-PLL (解耦双同步旋 PLL) =======================
//
// 来源: TI SPLL_3ph_DDSRF_F (solar/v1.2/float)
// 机制: 正序+负序双 dq 解耦 → 交叉耦合补偿 → 4 路 LPF → 正序 q 轴 → PI → VCO
// 核心: 不平衡电网下正负序解耦, 消除负序对锁相的干扰
//
// 解耦网络方程 (复域):
//   dq_p_decoupled = dq_p_raw - dq_n_filtered × (cos2θ + j·sin2θ)
//   dq_n_decoupled = dq_n_raw - dq_p_filtered × (cos2θ - j·sin2θ)

void ddsrf_pll_init(DdsrfPll *me, float grid_freq_hz, float dt,
                    float kp, float ki) {
  memset(me, 0, sizeof(DdsrfPll));

  me->fn = grid_freq_hz;
  me->delta_t = dt;

  // 环路滤波器系数 (PI 离散化)
  me->ylf[0] = 0.0f;
  me->ylf[1] = 0.0f;

  // 初始化 VCO
  me->theta[0] = 0.0f;
  me->theta[1] = 0.0f;
  me->fo = grid_freq_hz;

  // LPF 系数 (一阶低通, 截止频率 ~50Hz)
  float wc = M_2PI * 50.0f;
  float den_lpf = 1.0f / (1.0f + wc * dt);
  me->k2 = den_lpf;                           // 反馈系数 = 1/(1+ωc·T)
  me->k1 = wc * dt * den_lpf;                 // 输入系数 = ωc·T/(1+ωc·T)

  // 环路滤波器 PI 离散化
  me->b0_lf = kp + ki * dt * 0.5f;
  me->b1_lf = -kp + ki * dt * 0.5f;
}

// 一阶 LPF: y[k] = k1*x[k] + k2*y[k-1]
static inline float ddsrf_lpf(float *state, float k1, float k2, float x) {
  float y = k1 * x + k2 * state[0];
  state[1] = state[0];
  state[0] = y;
  return y;
}

float ddsrf_pll_run(DdsrfPll *me, float d_p, float d_n, float q_p, float q_n) {
  // 存取输入
  me->d_p = d_p;
  me->d_n = d_n;
  me->q_p = q_p;
  me->q_n = q_n;

  // ---- 阶段 1: 解耦网络 ----
  // 利用 cos(2θ)≈cos²-sin², sin(2θ)=2sin·cos
  float sin_t = sinf(me->theta[0]);
  float cos_t = cosf(me->theta[0]);

  me->cos_2theta = cos_t * cos_t - sin_t * sin_t;
  me->sin_2theta = 2.0f * sin_t * cos_t;

  // 正序解耦: dq_p_decoupled = dq_p - dq_n_filtered × (cos2θ + j·sin2θ)
  me->d_p_decoupl = me->d_p - (me->d_n_lpf * me->cos_2theta + me->q_n_lpf * me->sin_2theta);
  me->q_p_decoupl = me->q_p - (me->q_n_lpf * me->cos_2theta - me->d_n_lpf * me->sin_2theta);

  // 负序解耦: dq_n_decoupled = dq_n - dq_p_filtered × (cos2θ - j·sin2θ)
  //             = dq_n - dq_p_filtered × (cos2θ - j·sin2θ)
  me->d_n_decoupl = me->d_n - (me->d_p_lpf * me->cos_2theta - me->q_p_lpf * me->sin_2theta);
  me->q_n_decoupl = me->q_n - (me->q_p_lpf * me->cos_2theta + me->d_p_lpf * me->sin_2theta);

  // ---- 阶段 2: 4 路 LPF (对解耦后的 dq 分量分别滤波) ----
  me->d_p_lpf = ddsrf_lpf(me->y, me->k1, me->k2, me->d_p_decoupl);
  me->q_p_lpf = ddsrf_lpf(me->x, me->k1, me->k2, me->q_p_decoupl);
  me->d_n_lpf = ddsrf_lpf(me->w, me->k1, me->k2, me->d_n_decoupl);
  me->q_n_lpf = ddsrf_lpf(me->z, me->k1, me->k2, me->q_n_decoupl);

  // ---- 阶段 3: 环路滤波器 (正序 q 轴 → PI) ----
  me->v_q[1] = me->v_q[0];
  me->v_q[0] = me->q_p_lpf;

  float ylf_new = me->ylf[1] + me->b0_lf * me->v_q[0] + me->b1_lf * me->v_q[1];

  me->ylf[1] = me->ylf[0];
  me->ylf[0] = ylf_new;

  // ---- 阶段 4: 压控振荡器 VCO ----
  me->fo = me->fn + ylf_new;

  // 角度积分
  me->theta[1] = me->theta[0];
  me->theta[0] = me->theta[1] + me->fo * me->delta_t * M_2PI;

  // 相位折叠
  while (me->theta[0] > M_2PI) {
    me->theta[0] -= M_2PI;
  }

  return me->fo;
}

void ddsrf_pll_reset(DdsrfPll *me) {
  float fn_saved = me->fn;
  float dt_saved = me->delta_t;
  float kp = me->b0_lf + me->b1_lf;
  float ki = (me->b0_lf - me->b1_lf) / dt_saved;

  ddsrf_pll_init(me, fn_saved, dt_saved, kp, ki);
}
