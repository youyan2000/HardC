// 信号发生器 — 正弦波/余弦波发生器 (纯C, 无硬件依赖)
//
// 来源: TI controlSUITE SGEN (FPUfastRTS sincos) + DCL SGEN
// 翻译为 C-OOP 纯C float static inline 版本
//
// 两种模式:
//   SgenFixed  — 固定频率信号发生器 (给定幅值/频率/相位偏移)
//   SgenSweep  — 扫频信号发生器 (频率从 f_start → f_end 线性/对数扫)
//
// 应用场景:
//   - 逆变器正弦参考波生成
//   - PFC 电流参考生成 (与 PfcICmd 配合)
//   - 软件 DDS (直接数字合成) 替代硬件定时器
//   - 扫频阻抗测量 (与 SineAnalyzer 配合)

#ifndef COMP_SGEN_H
#define COMP_SGEN_H

#include <math.h>

#ifndef M_2PI
#define M_2PI 6.283185f
#endif

// ======================= SgenFixed (固定频率信号发生器) =======================

// 软件 DDS: y[k] = amplitude × sin(phase[k]) + offset
// 相位累加: phase[k] = phase[k-1] + Δphase, Δphase = freq × 2π × dt
//
// 等价于: y[k] = A × sin(2π × f × k × dt + φ₀) + offset

typedef struct {
  float amplitude;        // 输入: 幅值 (如 1.0 = 满幅)
  float freq_hz;          // 输入: 频率 (Hz)
  float phase_offset_rad; // 输入: 初始相位偏移 (rad)
  float offset;           // 输入: DC 偏移

  float phase;            // 内部: 当前相位 (rad)
  float dt;               // 参数: 采样周期 (s)
  float out;              // 输出: 当前信号值
  float sin_val;          // 输出: sin(phase)  (可单独使用)
  float cos_val;          // 输出: cos(phase)  (正交分量)
} SgenFixed;

#define SGEN_FIXED_DEFAULTS { 1.0f, 50.0f, 0, 0, 0, 0.0001f, 0, 0, 0 }

static inline void sgen_fixed_init(SgenFixed *me, float dt,
                                    float amplitude, float freq_hz,
                                    float phase_offset_rad, float offset) {
  me->amplitude = amplitude;
  me->freq_hz = freq_hz;
  me->phase_offset_rad = phase_offset_rad;
  me->offset = offset;
  me->phase = phase_offset_rad;     // 初始相位 = 偏移
  me->dt = dt;
  me->out = 0.0f;
  me->sin_val = 0.0f;
  me->cos_val = 1.0f;
}

// 单步: 相位累加 → sin → 幅值+偏移
//   new_freq_hz: 当前频率 (可动态改变, 相位连续)
//   返回: y[k]
static inline float sgen_fixed_tick(SgenFixed *me, float new_freq_hz) {
  me->freq_hz = new_freq_hz;

  // 相位增量: Δθ = 2π × f × dt
  float dtheta = M_2PI * me->freq_hz * me->dt;

  // 相位累加
  me->phase += dtheta;

  // 相位折叠 [0, 2π)
  while (me->phase > M_2PI) {
    me->phase -= M_2PI;
  }

  // sin/cos (硬件加速: 如果有 CMSIS-DSP 或 C2000 TMU, 编译器自动优化)
  me->sin_val = sinf(me->phase);
  me->cos_val = cosf(me->phase);

  // 输出
  me->out = me->amplitude * me->sin_val + me->offset;

  return me->out;
}

// 设置幅值 (运行时调幅)
static inline void sgen_fixed_set_amplitude(SgenFixed *me, float amplitude) {
  me->amplitude = amplitude;
}

// 强制相位 (用于 PLL 同步时重置相位)
static inline void sgen_fixed_set_phase(SgenFixed *me, float phase_rad) {
  me->phase = phase_rad;
}

// 重置相位到初始偏移
static inline void sgen_fixed_reset_phase(SgenFixed *me) {
  me->phase = me->phase_offset_rad;
}

// ======================= SgenSweep (扫频信号发生器) =======================

// 频率从 f_start → f_end 线性扫描 (chirp 信号)
//   f[k] = f_start + (f_end - f_start) × (k / total_steps)
//   y[k] = A × sin(phase_accumulated) + offset

typedef struct {
  float amplitude;
  float offset;
  float f_start_hz;       // 起始频率
  float f_end_hz;         // 结束频率
  float dt;
  int   total_steps;      // 总步数 (扫描时长 = total_steps × dt)

  float phase;            // 累计相位
  int   step;             // 当前步数
  float out;
  float current_freq;     // 输出: 当前频率 (Hz)
  float sin_val, cos_val;
} SgenSweep;

#define SGEN_SWEEP_DEFAULTS { 1.0f, 0, 10.0f, 1000.0f, 0.0001f, 1000, 0, 0, 0, 10.0f, 0, 0 }

static inline void sgen_sweep_init(SgenSweep *me, float dt,
                                    float amplitude, float offset,
                                    float f_start_hz, float f_end_hz,
                                    int total_steps) {
  me->amplitude = amplitude;
  me->offset = offset;
  me->f_start_hz = f_start_hz;
  me->f_end_hz = f_end_hz;
  me->dt = dt;
  me->total_steps = total_steps;
  me->phase = 0.0f;
  me->step = 0;
  me->out = 0.0f;
  me->current_freq = f_start_hz;
  me->sin_val = 0.0f;
  me->cos_val = 1.0f;
}

// 单步: 计算当前频率 → 相位累加 → sin → 幅值+偏移
//   扫完 total_steps 后保持末端频率
//   返回: y[k], 若扫完返回 0 (静音)
static inline float sgen_sweep_tick(SgenSweep *me) {
  if (me->step >= me->total_steps) {
    me->out = 0.0f;
    return 0.0f;  // 扫完, 静音
  }

  // 线性插值: 当前频率
  float frac = (float)me->step / (float)me->total_steps;
  me->current_freq = me->f_start_hz + (me->f_end_hz - me->f_start_hz) * frac;

  // 相位增量
  float dtheta = M_2PI * me->current_freq * me->dt;

  // 相位累加
  me->phase += dtheta;
  while (me->phase > M_2PI) {
    me->phase -= M_2PI;
  }

  me->sin_val = sinf(me->phase);
  me->cos_val = cosf(me->phase);

  me->out = me->amplitude * me->sin_val + me->offset;
  me->step++;

  return me->out;
}

// 重置扫频 (重新开始)
static inline void sgen_sweep_reset(SgenSweep *me) {
  me->phase = 0.0f;
  me->step = 0;
  me->current_freq = me->f_start_hz;
  me->out = 0.0f;
}

#endif  // COMP_SGEN_H
