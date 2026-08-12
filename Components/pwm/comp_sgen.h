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

// ======== 高级发生器 (v1.1 扩展) ========
// 来源: TI controlSUITE dsp/SGEN/v101/include/sgen.h
//
// 新增 4 种发生器:
//   SgenHp1     — 单音硬件探测 (系统辨识用, 注入固定频率正弦波)
//   SgenHp2     — 双音硬件探测 (互调失真/非线性测试, 同时注入两个频率)
//   SgenT3D     — 三音 DDS 发生器 (独立三路正弦叠加, 多频激励/谐波合成)
//   SgenProfile — 分段线性 Profile 发生器 (任意 (时间, 值) 断点, 线性插值)
//   SgenDeadzone — 死区线测试发生器 (交替输出活动区/零区, 死区补偿校准)

// ======================= SgenHp1 (单音硬件探测) =======================
// 来源: TI controlSUITE SGENHP_1 — 32-bit 高精度单频探测
//
// 注入单个固定频率正弦波到被控对象，测量输出响应，用于系统辨识:
//   y[k] = amplitude × sin(2π × freq_hz × k × dt) + offset
//
// 应用场景:
//   - 单频率植物传递函数测量 (Bode 图逐点扫频)
//   - 电流环扰动注入 (配合 SFRA 软件频响分析仪)
//   - 直流母线电压纹波注入测试

typedef struct {
  float freq_hz;          // 输入: 探测频率 (Hz)
  float amplitude;        // 输入: 探测幅值
  float offset;           // 输入: DC 偏移

  float dt;               // 参数: 采样周期 (s)
  float phase;            // 内部: 当前相位 (rad), 折叠至 [0, 2π)
  float out;              // 输出: 当前信号值
  float sin_val;          // 输出: sin(phase)
  float cos_val;          // 输出: cos(phase) (正交分量, 用于 IQ 解调)
} SgenHp1;

#define SGEN_HP1_DEFAULTS { 1000.0f, 1.0f, 0.0f, 0.0001f, 0.0f, 0.0f, 0.0f, 0.0f }

// 初始化单音探测发生器
//   dt:        采样周期 (s)
//   freq_hz:   探测频率, 通常选在被控对象带宽内
//   amplitude: 注入幅值, 典型值 0.05~0.1 (小信号, 不破坏工作点)
//   offset:    DC 偏移, 通常为工作点 (如 PFC 母线电压参考值)
static inline void sgen_hp1_init(SgenHp1 *me, float dt,
                                  float freq_hz, float amplitude, float offset) {
  me->freq_hz = freq_hz;
  me->amplitude = amplitude;
  me->offset = offset;
  me->dt = dt;
  me->phase = 0.0f;
  me->out = 0.0f;
  me->sin_val = 0.0f;
  me->cos_val = 1.0f;
}

// 单步运行: 相位累加 → sin → 幅值+偏移
//   返回: y[k] = amplitude × sin(phase) + offset
//   同时输出 sin_val (同相分量) 和 cos_val (正交分量), 供 IQ 解调使用
static inline float sgen_hp1_run(SgenHp1 *me) {
  // 相位增量: Δθ = 2π × f × dt
  float dtheta = M_2PI * me->freq_hz * me->dt;

  // 相位累加 + 折叠 [0, 2π)
  me->phase += dtheta;
  while (me->phase > M_2PI) {
    me->phase -= M_2PI;
  }

  // sin/cos 计算 (硬件加速: CMSIS-DSP / C2000 TMU)
  me->sin_val = sinf(me->phase);
  me->cos_val = cosf(me->phase);

  // 输出: 幅值 × sin + DC 偏移
  me->out = me->amplitude * me->sin_val + me->offset;

  return me->out;
}

// 运行时修改频率 (用于扫频系统辨识, 相位连续)
static inline void sgen_hp1_set_freq(SgenHp1 *me, float freq_hz) {
  me->freq_hz = freq_hz;
}

// ======================= SgenHp2 (双音硬件探测) =======================
// 来源: TI controlSUITE SGENHP_2 — 32-bit 高精度双频探测
//
// 同时注入两个不同频率的正弦波，用于互调失真和非线性测试:
//   y[k] = A1 × sin(2π × f1 × k × dt) + A2 × sin(2π × f2 × k × dt)
//
// 应用场景:
//   - 互调失真 (IMD) 测试: f1≠f2, 测量 f1±f2 混频分量
//   - 双频阻抗测量: 同时测量基频+谐波阻抗
//   - 非线性系统辨识: 检测交叉调制效应
//   - 陷波滤波器验证: 双频激励测试选择性

typedef struct {
  float freq1_hz;         // 输入: 第一探测频率 (Hz)
  float freq2_hz;         // 输入: 第二探测频率 (Hz)
  float amplitude1;       // 输入: 第一频率幅值
  float amplitude2;       // 输入: 第二频率幅值
  float offset;           // 输入: DC 偏移

  float dt;               // 参数: 采样周期 (s)
  float phase1;           // 内部: 第一频率相位 (rad)
  float phase2;           // 内部: 第二频率相位 (rad)
  float out;              // 输出: 当前合成信号值
  float sin_val1;         // 输出: sin(phase1) (第一频率分量)
  float sin_val2;         // 输出: sin(phase2) (第二频率分量)
} SgenHp2;

#define SGEN_HP2_DEFAULTS { 1000.0f, 2000.0f, 0.5f, 0.5f, 0.0f, 0.0001f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }

// 初始化双音探测发生器
//   dt:          采样周期 (s)
//   freq1_hz:    第一探测频率
//   freq2_hz:    第二探测频率 (建议与 freq1 不成整数倍, 避免谐波重叠)
//   amplitude1:  第一频率幅值
//   amplitude2:  第二频率幅值
//   offset:      DC 偏移
static inline void sgen_hp2_init(SgenHp2 *me, float dt,
                                  float freq1_hz, float freq2_hz,
                                  float amplitude1, float amplitude2,
                                  float offset) {
  me->freq1_hz = freq1_hz;
  me->freq2_hz = freq2_hz;
  me->amplitude1 = amplitude1;
  me->amplitude2 = amplitude2;
  me->offset = offset;
  me->dt = dt;
  me->phase1 = 0.0f;
  me->phase2 = 0.0f;
  me->out = 0.0f;
  me->sin_val1 = 0.0f;
  me->sin_val2 = 0.0f;
}

// 单步运行: 两路独立相位累加 → sin → 求和 + 偏移
//   返回: y[k] = A1×sin(phase1) + A2×sin(phase2) + offset
static inline float sgen_hp2_run(SgenHp2 *me) {
  // 第一频率: 相位累加
  float dtheta1 = M_2PI * me->freq1_hz * me->dt;
  me->phase1 += dtheta1;
  while (me->phase1 > M_2PI) {
    me->phase1 -= M_2PI;
  }

  // 第二频率: 相位累加
  float dtheta2 = M_2PI * me->freq2_hz * me->dt;
  me->phase2 += dtheta2;
  while (me->phase2 > M_2PI) {
    me->phase2 -= M_2PI;
  }

  // 分别计算正弦值 (保留分量供外部解调)
  me->sin_val1 = sinf(me->phase1);
  me->sin_val2 = sinf(me->phase2);

  // 合成输出: 双频叠加 + DC 偏移
  me->out = me->amplitude1 * me->sin_val1
          + me->amplitude2 * me->sin_val2
          + me->offset;

  return me->out;
}

// ======================= SgenT3D (三音 DDS 发生器) =======================
// 来源: TI controlSUITE SGENT_3D — 三路独立 DDS 合成
//
// 三路独立 DDS 正弦发生器，各有独立的频率/幅值/相位偏移，输出叠加:
//   y[k] = Σ A[i] × sin(phase[i] + φ[i]),  i = 0,1,2
//   phase[i][k] = phase[i][k-1] + 2π × f[i] × dt
//
// 应用场景:
//   - 多频率阻抗测量: 3 频点同时测量, 大幅缩短扫频时间
//   - 谐波合成: 基频+3次+5次谐波生成特定波形 (近似方波/三角波)
//   - 电网模拟: 基频+谐波 (THD 可编程)
//   - 主动噪声抑制: 多频反相注入

typedef struct {
  float freq_hz[3];       // 输入: 三路频率 (Hz)
  float amplitude[3];     // 输入: 三路幅值
  float phase_offset[3];  // 输入: 三路相位偏移 (rad)

  float dt;               // 参数: 采样周期 (s)
  float phase[3];         // 内部: 三路当前相位 (rad)
  float out;              // 输出: 三路合成信号
  float tone_out[3];      // 输出: 各分路单独输出 (tone_out[0..2])
} SgenT3D;

#define SGEN_T3D_DEFAULTS { \
  { 50.0f, 150.0f, 250.0f }, \
  { 1.0f, 0.33f, 0.2f }, \
  { 0.0f, 0.0f, 0.0f }, \
  0.0001f, \
  { 0.0f, 0.0f, 0.0f }, \
  0.0f, \
  { 0.0f, 0.0f, 0.0f } \
}

// 初始化三音发生器, 所有通道默认关闭 (幅值=0)
//   调用 sgen_t3d_set_tone() 逐通道配置频率/幅值/相位
static inline void sgen_t3d_init(SgenT3D *me, float dt) {
  me->dt = dt;
  me->out = 0.0f;
  for (int i = 0; i < 3; i++) {
    me->freq_hz[i] = 0.0f;
    me->amplitude[i] = 0.0f;
    me->phase_offset[i] = 0.0f;
    me->phase[i] = 0.0f;
    me->tone_out[i] = 0.0f;
  }
}

// 设置单个通道参数
//   index:  通道编号 (0/1/2)
//   freq:   频率 (Hz), 0 = 禁用此通道
//   amp:    幅值
//   ph_rad: 相位偏移 (rad)
static inline void sgen_t3d_set_tone(SgenT3D *me, int index,
                                      float freq, float amp, float ph_rad) {
  if (index < 0 || index > 2) return;
  me->freq_hz[index] = freq;
  me->amplitude[index] = amp;
  me->phase_offset[index] = ph_rad;
  me->phase[index] = ph_rad;  // 重置相位到偏移值
}

// 单步运行: 三路独立 DDS → 叠加输出
//   返回: 合成信号 y[k]
static inline float sgen_t3d_run(SgenT3D *me) {
  float sum = 0.0f;

  for (int i = 0; i < 3; i++) {
    if (me->freq_hz[i] <= 0.0f || me->amplitude[i] == 0.0f) {
      me->tone_out[i] = 0.0f;
      continue;
    }

    // 相位累加 + 折叠
    float dtheta = M_2PI * me->freq_hz[i] * me->dt;
    me->phase[i] += dtheta;
    while (me->phase[i] > M_2PI) {
      me->phase[i] -= M_2PI;
    }

    // 正弦输出: sin(phase + phase_offset) 注: 初始时 phase = offset, 此后独立累加
    me->tone_out[i] = me->amplitude[i] * sinf(me->phase[i]);

    sum += me->tone_out[i];
  }

  me->out = sum;
  return me->out;
}

// ======================= SgenProfile (分段线性 Profile 发生器) =======================
// 来源: TI controlSUITE PROFILE — 分段折线发生器
//
// 定义一组 (时间, 值) 断点, 发生器按时间推进, 在相邻断点之间线性插值:
//   t ∈ [t[i], t[i+1]] → y = v[i] + (v[i+1] - v[i]) × (t - t[i]) / (t[i+1] - t[i])
//
// 应用场景:
//   - 软启动斜坡: (0,0) → (0.5, 0.5) → (1.0, 1.0)
//   - 电流限幅曲线: 启动时限制电流上升速率
//   - 速度 Profile: 加速→匀速→减速三段式运动规划
//   - 电压/电流任意波形: 逐段线性逼近任意曲线
//   - PID 参考值平滑过渡: 避免阶跃引起过冲

#define SGEN_PROFILE_MAX_BP 16   // 最大断点数

typedef struct {
  float t;    // 时间 (s)
  float v;    // 值
} SgenProfileBP;

typedef struct {
  const SgenProfileBP *bps; // 输入: 断点数组 (时间必须递增)
  int   bp_count;           // 输入: 断点数量 (≤ SGEN_PROFILE_MAX_BP)
  int   current_segment;    // 内部: 当前段索引 (0 = bp[0]→bp[1])
  float elapsed;            // 内部: 当前段已用时间 (s)
  float out;                // 输出: 当前插值结果
  int   done;               // 输出: 全部完成 (1=已完成)
} SgenProfile;

#define SGEN_PROFILE_DEFAULTS { NULL, 0, 0, 0.0f, 0.0f, 0 }

// 初始化 Profile 发生器 (不加载数据, 需后续调用 sgen_profile_load)
static inline void sgen_profile_init(SgenProfile *me) {
  me->bps = NULL;
  me->bp_count = 0;
  me->current_segment = 0;
  me->elapsed = 0.0f;
  me->out = 0.0f;
  me->done = 0;
}

// 加载断点数组
//   bps:  断点数组指针 (必须是静态/全局数组, Profile 不拷贝数据)
//   count: 断点数量 (至少 2 个, ≤ SGEN_PROFILE_MAX_BP)
static inline void sgen_profile_load(SgenProfile *me,
                                      const SgenProfileBP *bps, int count) {
  me->bps = bps;
  me->bp_count = count;
  me->current_segment = 0;
  me->elapsed = 0.0f;
  me->done = (count < 2) ? 1 : 0;
  me->out = (bps && count > 0) ? bps[0].v : 0.0f;
}

// 单步推进: 按 dt 推进时间 → 线性插值当前段的值
//   返回: 当前插值结果, 完成后保持末端值不变
static inline float sgen_profile_run(SgenProfile *me, float dt) {
  if (me->done || me->bp_count < 2 || me->bps == NULL) {
    return me->out;
  }

  // 推进时间
  me->elapsed += dt;

  // 检查是否需要切换到下一段
  while (me->current_segment < me->bp_count - 1) {
    const SgenProfileBP *bp_start = &me->bps[me->current_segment];
    const SgenProfileBP *bp_end   = &me->bps[me->current_segment + 1];

    float seg_duration = bp_end->t - bp_start->t;

    if (me->elapsed < seg_duration) {
      // 仍在当前段内: 线性插值
      float frac = me->elapsed / seg_duration;
      me->out = bp_start->v + (bp_end->v - bp_start->v) * frac;
      return me->out;
    } else {
      // 进入下一段: 减去当前段耗时
      me->elapsed -= seg_duration;
      me->current_segment++;
    }
  }

  // 全部段完成: 钳位在末端值
  me->done = 1;
  me->out = me->bps[me->bp_count - 1].v;
  return me->out;
}

// 查询是否已完成 (达末端断点)
static inline int sgen_profile_is_done(SgenProfile *me) {
  return me->done;
}

// 重置到起始位置 (从第一段重新开始)
static inline void sgen_profile_reset(SgenProfile *me) {
  me->current_segment = 0;
  me->elapsed = 0.0f;
  me->done = (me->bp_count < 2) ? 1 : 0;
  me->out = (me->bps && me->bp_count > 0) ? me->bps[0].v : 0.0f;
}

// ======================= SgenDeadzone (死区线测试发生器) =======================
// 来源: TI controlSUITE TZDLGEN (Transition Zone Dead-time Line Generator)
//
// 交替输出活动区和零区:
//   ┌─ 活动区 (active_width): y = amplitude × sign (正半周输出 +amplitude)
//   └─ 零区 (zero_width):     y = 0
//   重复此模式
//
// 应用场景:
//   - 死区补偿校准: 注入死区测试信号观察交越失真
//   - PWM 线性度测试: 小占空比区间 PWM 输出线性度评估
//   - 逆变器死区效应测量: 观察输出电压在过零点的畸变
//   - 继电器/接触器寿命测试: 周期性通断

typedef struct {
  float active_width;     // 输入: 活动区宽度 (s)
  float zero_width;       // 输入: 零区宽度 (s)
  float amplitude;        // 输入: 活动区幅值
  int   polarity;         // 输入: 极性模式 (0=双极性±交替, 1=仅正, -1=仅负)

  float dt;               // 参数: 采样周期 (s)
  float timer;            // 内部: 当前相位计时器 (s)
  int   is_active;        // 内部: 当前是否在活动区 (1=活动, 0=零区)
  int   sign;             // 内部: 当前极性 (+1 / -1)
  float out;              // 输出: 当前信号值
} SgenDeadzone;

#define SGEN_DEADZONE_DEFAULTS { 0.1f, 0.05f, 1.0f, 0, 0.0001f, 0.0f, 1, 1, 0.0f }

// 初始化死区线测试发生器
//   dt:           采样周期 (s)
//   active_width: 活动区持续时间 (s)
//   zero_width:   零区持续时间 (s)
//   amplitude:    活动区输出幅值
//   polarity:     0=双极性 (正负交替), 1=仅正半周, -1=仅负半周
static inline void sgen_deadzone_init(SgenDeadzone *me, float dt,
                                       float active_width, float zero_width,
                                       float amplitude, int polarity) {
  me->active_width = active_width;
  me->zero_width = zero_width;
  me->amplitude = amplitude;
  me->polarity = polarity;
  me->dt = dt;
  me->timer = 0.0f;
  me->is_active = 1;       // 从活动区开始
  me->sign = 1;
  me->out = 0.0f;
}

// 单步运行: 推进计时器 → 判断活动/零区 → 输出
//   活动区: y = sign × amplitude
//   零区:   y = 0
//   双极性模式: 每次进入活动区时翻转符号
static inline float sgen_deadzone_run(SgenDeadzone *me) {
  // 推进计时器
  me->timer += me->dt;

  if (me->is_active) {
    // 活动区: 检查是否到期
    if (me->timer >= me->active_width) {
      me->timer -= me->active_width;
      me->is_active = 0;

      // 双极性模式: 活动区结束时翻转符号, 供下一次活动区使用
      if (me->polarity == 0) {
        me->sign = -me->sign;
      } else {
        me->sign = (me->polarity > 0) ? 1 : -1;
      }
    }
  } else {
    // 零区: 检查是否到期
    if (me->timer >= me->zero_width) {
      me->timer -= me->zero_width;
      me->is_active = 1;
    }
  }

  // 输出
  if (me->is_active) {
    me->out = me->sign * me->amplitude;
  } else {
    me->out = 0.0f;
  }

  return me->out;
}

// 重置计时器，从活动区重新开始
static inline void sgen_deadzone_reset(SgenDeadzone *me) {
  me->timer = 0.0f;
  me->is_active = 1;
  me->sign = 1;
  me->out = 0.0f;
}

#endif  // COMP_SGEN_H
