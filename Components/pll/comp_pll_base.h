// 锁相环平台层 —— 极简基类: 承诺 PD(鉴相) → LF(环路滤波) → VCO(压控振荡器) 三环节
//
// 结构模型 (与 PID 域 PidBase 同构, 继承方式二):
//   PllBase = { ops 虚表 + 统一输入帧 + 统一反馈输出 } + LF(PI) + VCO
//   父类承诺一个输入 (相位电压帧 PllInput) + 一个反馈输出 (锁相角 theta + 频率 fo)。
//   LF(环路滤波=纯 PI, Tustin 离散化) 与 VCO(角度积分+相位折叠) 对 5 个子类全部一致,
//   故收进基类字段 + 统一实现 pll_base_lf_vco() 下沉复用。
//   子类只实现 PD(鉴相): 把输入电压帧投影为锁相误差 v_q, 再调基类 LF+VCO。
//
// 统一入口 pll_run(base, in) 内部完成: 子类 PD → 基类 LF → 基类 VCO(更新反馈)。
//
// 鉴相方式覆盖 (都是乘法/乘积/投影型, 无过零鉴相):
//   PllSogi   — SOGI-QSG 正交 + Park 乘法投影 (单相高性能, 亦为 SSRF-SPLL 变体)
//   PllSrf    — Park 旋转坐标乘法投影 (三相标准 SRF)
//   PllNotch  — 纯乘积累积型检测器 v×cos + 陷波 2f0 (轻量单相)
//   PllDdsrf  — 双 dq 正负序解耦 + Park 乘法投影 (三相不平衡鲁棒)
//   PllSogiFll— SOGI-QSG 正交 + Park 乘法投影 + FLL 频率自适
//
// 保留旧 comp_pll.h/c (SogiPll/SrfPll/NotchPll/DdsrfPll) + comp_sogi_fll.h (SogiFll)
// 作为库存不动; 本域子类为其 OOP 重构版 (PllSogi/PllSrf/PllNotch/PllDdsrf/PllSogiFll)。

#ifndef COMP_PLL_BASE_H
#define COMP_PLL_BASE_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "comp_math.h"

// ======== 前向声明 ========
typedef struct PllBase PllBase;

// ======== 统一输入帧 —— 一个输入 ========
// 各子类只读取自己需要的字段:
//   v        — 单相电网电压采样 (PllSogi / PllNotch / PllSogiFll 用)
//   v_alpha  — Clarke 变换后的 α 轴 (PllSrf 用)
//   v_beta   — Clarke 变换后的 β 轴 (PllSrf 用)
//   d_p/d_n/q_p/q_n — 正负序 dq 分量 (DdsrfPll 用, 外部 Park 变换填入)
typedef struct {
  float v;         // 单相电压
  float v_alpha;   // α 轴
  float v_beta;    // β 轴
  float d_p, d_n;  // 正/负序 d 轴
  float q_p, q_n;  // 正/负序 q 轴
} PllInput;

// ======== 虚函数签名 (子类各自实现) ========

// 单步推进: 子类实现 PD → 调基类 pll_base_lf_vco() 完成 LF+VCO
typedef void  (*pll_run_fn)(PllBase *base, const PllInput *in);

// 清零内部状态 (故障恢复 / 重新启动)
typedef void  (*pll_reset_fn)(PllBase *base);

// ======== 虚函数表 ========
typedef struct {
  pll_run_fn   run;    // 必选
  pll_reset_fn reset;  // 可选
} PllOps;

// ======== 基类结构体 —— LF(PI) + VCO 字段 + 统一反馈输出 ========
struct PllBase {
  const PllOps *ops;

  // ---- 配置 ----
  float fn;           // 标称频率 (Hz), 如 50 / 60
  float delta_t;      // 采样周期 (s), 如 1e-4 = 100µs
  float kp, ki;       // LF 环路滤波 PI 参数 (带宽由 kp/ki 决定)
  float freq_lim;     // 频率偏差限幅 (±Hz), 0 = 不限制 (SRF 默认 ±200)

  // ---- LF (环路滤波 = 纯 PI, Tustin 离散化) 状态 ----
  float v_q_prev;     // 上拍鉴相误差 v_q[k-1]
  float ylf;          // LF 积分器状态 (输出频率偏差 Δω)
  float b0, b1;       // Tustin PI 系数 (b0=kp+ki·T/2, b1=-kp+ki·T/2)

  // ---- VCO (压控振荡器) 状态 + 统一反馈输出 ----
  float fo;           // 输出: 锁定频率 (Hz) = fn + ylf
  float theta;        // 输出: 锁相角 (rad, 0~2π) —— 反馈回鉴相器的关键
  float sin_val;      // 输出: sin(θ) (缓存, 供下一拍 PD / Park 使用)
  float cos_val;      // 输出: cos(θ)
};

// ======== 基类构造 ========

// 初始化基类默认字段; ops 由子类 init 时绑定
void pll_base_init(PllBase *base);

// ======== 基类统一实现: LF + VCO (子类 PD 算出 v_q 后调用) ========
// 封装在 .c, 子类 PD 内联调用 (仅在子类文件中使用)
void pll_base_lf_vco(PllBase *base, float v_q);

// 运行时更新 LF PI 参数 (不重置积分器状态)
void pll_base_set_pi(PllBase *base, float kp, float ki);

// 只清零运行时状态 (LF 积分器 + VCO 相位/输出), 保留配置与 ops
// 供子类 reset 复用: 不触碰 fn/delta_t/kp/ki/freq_lim/ops
void pll_base_reset_state(PllBase *base);

// ======== 统一对外接口 (inline, 零开销) ========

// 单次锁相周期入口: 输入相位电压帧 → 子类 PD → 基类 LF → 基类 VCO (更新反馈 theta/fo)
// 返回: 锁定频率 fo (Hz)
static inline float pll_run(PllBase *base, const PllInput *in) {
  base->ops->run(base, in);
  return base->fo;
}

// 清零内部状态 (委托子类 reset)
static inline void pll_reset(PllBase *base) {
  if (base->ops->reset) {
    base->ops->reset(base);
  }
}

// ---- 反馈输出 getter ----

// 锁定频率 (Hz)
static inline float pll_get_freq(PllBase *base) {
  return base->fo;
}

// 锁相角 (rad, 0~2π) —— 供 Park/InvPark 变换 / 并网同步
static inline float pll_get_theta(PllBase *base) {
  return base->theta;
}

// sin(θ) / cos(θ) —— 供旋转变换复用
static inline float pll_get_sin(PllBase *base) {
  return base->sin_val;
}

static inline float pll_get_cos(PllBase *base) {
  return base->cos_val;
}

#endif  // COMP_PLL_BASE_H
