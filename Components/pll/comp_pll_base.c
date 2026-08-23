// 锁相环平台层 —— 基类实现
// 所有子类共享的 LF(环路滤波=纯 PI) + VCO(角度积分+相位折叠) 下沉于此。
// 与 PID 的 PidBase 分工对应: PidBase 管 dt/限幅/抗饱和调度, PllBase 管 LF+VCO 反馈调度。

#include "comp_pll_base.h"
#include "bsp_dsp.h"      // bsp_sin_f32/cos_f32/atan2_f32 — 硬件加速 (CLAmath) / math.h 回退
#include <stddef.h>

// 基类默认构造: 只设默认时钟 1ms、标称频率 50Hz、不限频、未绑定 ops
void pll_base_init(PllBase *base) {
  base->ops       = NULL;
  base->fn        = 50.0f;
  base->delta_t   = 1e-3f;
  base->kp        = 0.0f;
  base->ki        = 0.0f;
  base->freq_lim  = 0.0f;      // 默认不限幅 (子类 init 可覆盖, 如 SRF 设 ±200)
  base->vco_mode  = PllVcoEuler;  // 默认欧拉 (子类可切 Precise)
  base->v_q_prev  = 0.0f;
  base->ylf       = 0.0f;
  base->b0        = 0.0f;
  base->b1        = 0.0f;
  base->fo        = 0.0f;
  base->theta     = 0.0f;
  base->sin_val   = 0.0f;
  base->cos_val   = 1.0f;
}

// 统一 LF + VCO: 子类 PD 输出锁相误差 v_q → 纯 PI 推频率 → VCO 积分相位 (重建反馈)
//   LF(s) = kp + ki/s   ⟶  Tustin:  ylf += b0*v_q[k] + b1*v_q[k-1],  b0=kp+ki·T/2, b1=-kp+ki·T/2
//   fo   = fn + ylf
//   VCO 按 vco_mode 分派:
//     PllVcoEuler   — 欧拉积分: θ += fo·T·2π, 相位折叠到 0~2π
//     PllVcoPrecise — 精确离散振荡器: cos(θ+Δ)=cos·cosΔ−sin·sinΔ, 幅值归一化, θ=atan2(sin,cos)
//   sin/cos 缓存, 供下一拍鉴相器/旋转变换复用
void pll_base_lf_vco(PllBase *base, float v_q) {
  // ---- LF: 环路滤波 (纯 PI) ----
  float ylf_new = base->ylf + base->b0 * v_q + base->b1 * base->v_q_prev;

  // 频率偏差限幅 (防失锁发散)
  if (base->freq_lim > 0.0f) {
    if (ylf_new > base->freq_lim) ylf_new = base->freq_lim;
    else if (ylf_new < -base->freq_lim) ylf_new = -base->freq_lim;
  }

  base->v_q_prev = v_q;
  base->ylf      = ylf_new;

  // ---- VCO: 压控振荡器 ----
  base->fo = base->fn + base->ylf;
  float delta_theta = base->fo * base->delta_t * M_2PI;

  if (base->vco_mode == PllVcoPrecise) {
    // 精确离散时间振荡器: 三角加法公式 + 幅值归一化 (防幅值漂移)
    float cos_delta = bsp_cos_f32(delta_theta);
    float sin_delta = bsp_sin_f32(delta_theta);
    float cos_new = base->cos_val * cos_delta - base->sin_val * sin_delta;
    float sin_new = base->sin_val * cos_delta + base->cos_val * sin_delta;

    // 正交归一化 (防止幅值漂移 + 保持单位模长)
    float mag = MATH_SQRT(cos_new * cos_new + sin_new * sin_new);
    if (mag > 0.0f) {
      cos_new /= mag;
      sin_new /= mag;
    }

    base->cos_val = cos_new;
    base->sin_val = sin_new;
    base->theta   = bsp_atan2_f32(sin_new, cos_new);   // [-π, π]
    if (base->theta < 0.0f) {
      base->theta += M_2PI;                     // 折叠到 [0, 2π)
    }
  } else {
    // 欧拉积分 (默认)
    base->theta += delta_theta;
    if (base->theta > M_2PI) {
      base->theta -= M_2PI;
    }
    if (base->theta < 0.0f) {
      base->theta += M_2PI;
    }
    base->sin_val = bsp_sin_f32(base->theta);
    base->cos_val = bsp_cos_f32(base->theta);
  }
}

// 运行时更新 LF PI 参数 (重算 Tustin 系数, 不重置积分器)
void pll_base_set_pi(PllBase *base, float kp, float ki) {
  base->kp = kp;
  base->ki = ki;
  base->b0 = kp + ki * base->delta_t * 0.5f;
  base->b1 = -kp + ki * base->delta_t * 0.5f;
}

// 只清零运行时状态 (LF 积分器 + VCO 相位/输出), 保留配置与 ops (含 vco_mode)
void pll_base_reset_state(PllBase *base) {
  base->v_q_prev = 0.0f;
  base->ylf      = 0.0f;
  base->fo       = 0.0f;
  base->theta    = 0.0f;
  base->sin_val  = 0.0f;
  base->cos_val  = 1.0f;
}
