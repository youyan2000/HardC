// 基波电力分析 — 同步正交相关解调 + 谐波失真 (IEC 62053 电能计量)
//
// 来源: TI C2000Ware Digital Power SDK
//   libraries/energy-metrology_library/energy_metrology_f28p55
//   (metrology_background.c 正交相关累加 + metrology_calculations.c
//    calculateFundamentalRMSVoltage/ActivePower/ReactivePower + THD)
// 翻译为 C-OOP 纯C float 版本
//
// 与 comp_power_meas.h (宽频带能量累加) 的区别: 本组件在测得的电网频率上做
// 窄带同步解调, 只提取基波分量 — 对谐波污染免疫. 两者配合得到:
//   基波: FRMS/FP/FQ/FS (本组件)
//   谐波失真: THD = sqrt(RMS² − FRMS²)/FRMS × 100 (本组件用宽频 RMS 结算)
//
// 算法 (metrology_background.c):
//   1. 相位参考: phase += 2π·f_ref·dt, 每采样步进 (f_ref 由过零周期估计更新)
//   2. 正交参考: v_ref = sin(θ), v_quad = cos(θ)  [= sin(θ+90°)]
//   3. 逐采样相关累加:
//        v_in += v·sinθ          v_cos += v·cosθ
//        i_in += i·sinθ          i_q   += i·(−cosθ)   ← TI 无功符号约定
//   4. 窗口结算 (N 样本):
//        V_mag2 = sqrt((v_in/N)² + (v_cos/N)²)   (基波幅值一半)
//        FRMS_V = V_mag2·√2
//        FRMS_I = sqrt((i_in/N)² + (i_q/N)²)·√2
//        F_P = 2·(v_in·i_in − v_cos·i_q)   (电压框架投影, 旋转不变)
//        F_Q = 2·(v_cos·i_in + v_in·i_q)
//        F_S = FRMS_V·FRMS_I
//        THD  = sqrt(RMS² − FRMS²)/FRMS × 100 (RMS 由宽频测量输入)
//
// 注意1: 结算时累加器清零但相位参考保持连续, 窗间相关不失步
// 注意2: 功率按电压复矢量投影 (旋转不变) — 参考帧无需与电网电压锁相.
//   ADC 启动相位任意, P/Q 始终相对电压测量. 与 TI 计量库的相位对齐
//   (VoltagePure 投影) 数学等价; 单纯 (i_in/N)·A_v 仅在参考与电压同相时正确

#ifndef COMP_POWER_FUND_H
#define COMP_POWER_FUND_H

#include <math.h>
#include <stdint.h>

// ======================= 基波分析器 =======================

typedef struct {
  // 参数
  float f_ref;            // 参数: 参考频率 (Hz), 由电网频率估计每窗更新
  float dt;               // 参数: 采样周期 (s)
  uint16_t window_n;      // 参数: 结算窗口样本数 (典型 1~4 周期)

  // 内部
  float phase;            // 相位参考 (rad, 连续)
  float v_sin, v_cos;     // Σv·sinθ, Σv·cosθ
  float i_sin, i_q;       // Σi·sinθ, Σi·(−cosθ)
  uint32_t count;         // 窗口内样本计数 (高采样率长窗不溢出)

  // 输出 (power_fund_update 后有效)
  float frms_v, frms_i;   // 基波有效值 (V, A)
  float f_p, f_q, f_s;    // 基波有功/无功/视在 (W, VAR, VA)
  float thd_v, thd_i;     // 电压/电流 THD (%, 需宽频 RMS 输入)
} PowerFund;

// 初始化
//   sample_rate   — 采样率 (Hz)
//   f_ref         — 初始参考频率 (Hz, 电网标称, 运行时可用 set_freq 更新)
//   window_n      — 结算窗口样本数 (如 4 周期 × Fs/f_ref)
static inline void power_fund_init(PowerFund *me, float sample_rate,
                                   float f_ref, uint16_t window_n) {
  me->f_ref = f_ref;
  me->dt = 1.0f / sample_rate;
  me->window_n = window_n;

  me->phase = 0.0f;
  me->v_sin = 0.0f;
  me->v_cos = 0.0f;
  me->i_sin = 0.0f;
  me->i_q = 0.0f;
  me->count = 0u;

  me->frms_v = 0.0f;
  me->frms_i = 0.0f;
  me->f_p = 0.0f;
  me->f_q = 0.0f;
  me->f_s = 0.0f;
  me->thd_v = 0.0f;
  me->thd_i = 0.0f;
}

// 更新参考频率 — 由电网频率估计 (过零/锁相环) 每窗调用, 抑制窗泄漏
static inline void power_fund_set_freq(PowerFund *me, float f_hz) {
  me->f_ref = f_hz;
}

// 清零累加器与相位参考
static inline void power_fund_reset(PowerFund *me) {
  me->phase = 0.0f;
  me->v_sin = 0.0f;
  me->v_cos = 0.0f;
  me->i_sin = 0.0f;
  me->i_q = 0.0f;
  me->count = 0u;
}

// 逐采样正交相关累加 — ISR 每采样调用
static inline void power_fund_sample(PowerFund *me, float v, float i) {
  const float two_pi = 6.28318530718f;

  // 用当前相位相关 (与信号 t=k·dt 处相位对齐), 再步进参考
  // 若先步进再相关, 首个样本错相 Δ, 使测得功率角偏移一个采样
  float s = sinf(me->phase);
  float c = cosf(me->phase);

  me->v_sin += v * s;
  me->v_cos += v * c;
  me->i_sin += i * s;
  me->i_q += i * (-c);   // TI 无功符号约定: 滞后电流 → Q>0

  // 相位参考步进 (rad, 连续, 不随结算复位)
  me->phase += two_pi * me->f_ref * me->dt;
  if (me->phase >= two_pi) {
    me->phase -= two_pi;
  }

  me->count++;
}

// 窗口结算 — 每 window_n 样本调用
//   rms_v, rms_i — 宽频带有效值 (来自 comp_power_meas.h), 用于 THD
//   返回 1 = 有新的基波结果, 0 = 窗口未满
static inline int power_fund_update(PowerFund *me, float rms_v, float rms_i) {
  if (me->count == 0u || me->count < me->window_n) {
    return 0;
  }

  float n = (float)me->count;
  float v_in = me->v_sin / n;
  float v_q = me->v_cos / n;
  float i_in = me->i_sin / n;
  float i_q = me->i_q / n;

  // 基波幅值一半 V_mag2 = sqrt(VI² + VQ²) (同相/正交相关合量)
  float v_mag2 = sqrtf(v_in * v_in + v_q * v_q);
  float i_mag2 = sqrtf(i_in * i_in + i_q * i_q);

  // 基波有效值: 峰值 = 2·mag2, RMS = 峰值/√2 → mag2·√2
  me->frms_v = v_mag2 * 1.41421356237f;
  me->frms_i = i_mag2 * 1.41421356237f;

  // 基波功率: 电压复矢量 (v_in + j·v_cos) 上的电流投影 — 旋转不变
  //   P = Re(I·conj(V))·2 = 2·(v_in·i_in − v_cos·i_q)
  //   Q = Im(I·conj(V))·2 = 2·(v_cos·i_in + v_in·i_q)
  // 参考帧与电压间的任意相位偏移 ψ 自动消除, 无需电网锁相
  me->f_p = 2.0f * (v_in * i_in - v_q * i_q);
  me->f_q = 2.0f * (v_q * i_in + v_in * i_q);
  me->f_s = me->frms_v * me->frms_i;

  // THD: 谐波含量 = sqrt(宽频RMS² − 基波RMS²), 除以基波
  float v_diff = rms_v * rms_v - me->frms_v * me->frms_v;
  float i_diff = rms_i * rms_i - me->frms_i * me->frms_i;
  me->thd_v = (v_diff > 0.0f && me->frms_v > 0.0f)
              ? (sqrtf(v_diff) / me->frms_v) * 100.0f : 0.0f;
  me->thd_i = (i_diff > 0.0f && me->frms_i > 0.0f)
              ? (sqrtf(i_diff) / me->frms_i) * 100.0f : 0.0f;

  // 清累加器, 相位参考保持连续 (窗间不失步)
  me->v_sin = 0.0f;
  me->v_cos = 0.0f;
  me->i_sin = 0.0f;
  me->i_q = 0.0f;
  me->count = 0u;

  return 1;
}

#endif  // COMP_POWER_FUND_H
