// 电机控制 — 旋转变压器接口 (Resolver)
//
// 来源: TI controlSUITE motor_control/math_blocks/v4.3 (resolver.h)
// 翻译为 C-OOP 纯C float inline 版本
//
// 旋变: 绝对位置传感器, 通过解码 sin/cos 励磁信号获取机械角度
//   电角度 = 极对数 × 机械角度
//   机械角度 = (RawPosition - Offset) × Scaler
//
// 定标: 输入 RawTheta 为 Q0 整数 (0~StepsPerTurn), 输出为弧度

#ifndef COMP_RESOLVER_H
#define COMP_RESOLVER_H

#include <math.h>
#include <stdint.h>
#include "comp_iqmath.h"

#ifndef M_2PI
#define M_2PI 6.283185f
#endif

// ======================= Resolver (旋转变压器解码) =======================

typedef struct {
  // 输入
  float raw_theta;        // 输入: 原始位置 (整数计数, 0~StepsPerTurn)

  // 输出
  float elec_theta;       // 输出: 电角度 (rad)
  float mech_theta;       // 输出: 机械角度 (rad)
  float speed;            // 输出: 转速 (标幺)

  // 参数
  float init_theta;       // 参数: 初始角度偏移 (Q0)
  float mech_scaler;      // 参数: 机械角度缩放 = 2π/总计数
  float steps_per_turn;   // 参数: 每圈离散位置数
  uint16_t pole_pairs;    // 参数: 极对数
} Resolver;

#define RESOLVER_DEFAULTS { 0, 0, 0, 0, 0, 0.0001f, 4096, 2 }

// 初始化
//   steps_per_turn: 每圈编码器位置数 (如 4096)
//   pole_pairs:     电机极对数 (如 2=4极, 4=8极)
static inline void resolver_init(Resolver *me, float steps_per_turn,
                                  uint16_t pole_pairs) {
  me->raw_theta = 0.0f;
  me->elec_theta = 0.0f;
  me->mech_theta = 0.0f;
  me->speed = 0.0f;
  me->init_theta = 0.0f;
  me->mech_scaler = M_2PI / steps_per_turn;
  me->steps_per_turn = steps_per_turn;
  me->pole_pairs = pole_pairs;
}

// 旋变解码单步运行
//   raw_position: 当前原始编码器读数 (0 ~ StepsPerTurn-1)
//   返回: 电角度 (rad)
static inline float resolver_decode(Resolver *me, float raw_position) {
  me->raw_theta = raw_position;

  // 机械角度 = (原始值 - 偏移) × 缩放因子
  me->mech_theta = (raw_position - me->init_theta) * me->mech_scaler;

  // 电角度 = 极对数 × 机械角度
  me->elec_theta = (float)me->pole_pairs * me->mech_theta;

  // 角度折叠 — 保留分数部分
  while (me->elec_theta > M_2PI) {
    me->elec_theta -= M_2PI;
  }
  while (me->elec_theta < 0.0f) {
    me->elec_theta += M_2PI;
  }

  return me->elec_theta;
}

// 设置机械角度偏移 (校准用)
static inline void resolver_set_offset(Resolver *me, float offset) {
  me->init_theta = offset;
}

// ======== IQmath 定点解调路径 (v1.1 扩展) ========
//
// 来源: TI controlSUITE motor_control/libs/resolver/v101/Resolver_Fixed.h
// 移植为 C-OOP 纯C _iq 定点版本 (Q24)
//
// 完整旋变信号链:
//   1. DDS 生成励磁载波 (sin 参考信号) → 驱动 DAC/PWM
//   2. ADC 同步采样 sin(θ)·sin(ωt) 和 cos(θ)·sin(ωt) 调制信号
//   3. 同步解调: ADC 信号 × 励磁参考 → 提取包络
//   4. 一阶 IIR 低通滤波 → 滤除 2ω 分量, 得到 DC sin(θ)/cos(θ)
//   5. atan2 查表 → 瞬时角度 (per-unit)
//   6. PLL 锁相环跟踪 → 滤波角度 + 速度估计
//
// 所有内部运算使用 _iq (Q24) 定点格式, 无浮点依赖, 适合无 FPU 平台

// ---- 配置结构体 ----

typedef struct {
  float    excitation_freq_hz;  // 励磁载波频率 (Hz), 典型 10k~20k
  float    sample_freq_hz;      // 控制/采样频率 (Hz)
  float    lpf_bw_hz;           // 解调 LPF 带宽 (Hz), 典型 500~2000
  uint16_t pole_pairs;          // 电机极对数 (1=2极, 2=4极)
  _iq      lpf_k;               // 预计算: 一阶 IIR 系数 k = 2π·fc/fs (Q24)
  _iq      phase_step;          // 预计算: DDS 相位步进 = f_exc/f_sample (Q24)
} ResolverFixedCfg;

// ---- 运行时状态 ----

typedef struct {
  // DDS 励磁发生器
  _iq exc_phase_acc;            // 相位累加器 (Q24, [0, 1) → 0~2π)
  _iq exc_ref;                  // 当前励磁参考输出 (Q24, sin 值)

  // 同步解调
  _iq sin_dc;                   // 瞬时解调 sin 分量 (Q24, 未滤波)
  _iq cos_dc;                   // 瞬时解调 cos 分量 (Q24, 未滤波)
  _iq sin_dc_filt;              // LPF 滤波后 sin 分量 (Q24)
  _iq cos_dc_filt;              // LPF 滤波后 cos 分量 (Q24)

  // PLL 锁相环
  _iq pll_angle;                // PLL 跟踪角度 (Q24, per-unit [0, 1))
  _iq pll_speed;                // PLL 速度估计 (Q24, per-unit/控制周期)
  _iq pll_integral;             // PLL 积分累加器 (Q24)
  _iq pll_kp;                   // PLL 比例增益 (Q24, 用户可调)
  _iq pll_ki;                   // PLL 积分增益 (Q24, 用户可调)

  // 输出
  _iq elec_angle;               // 电角度 (Q24, per-unit [0, 1))
  _iq mech_angle;               // 机械角度 (Q24, per-unit, 单电周期内)
} ResolverFixedState;

// ---- API ----

// 初始化状态 — 所有字段归零, PLL 增益设默认值
//   st: 状态指针
static inline void resolver_fixed_init(ResolverFixedState *st) {
  st->exc_phase_acc = IQ_ZERO;
  st->exc_ref       = IQ_ZERO;
  st->sin_dc        = IQ_ZERO;
  st->cos_dc        = IQ_ZERO;
  st->sin_dc_filt   = IQ_ZERO;
  st->cos_dc_filt   = IQ_ZERO;
  st->pll_angle     = IQ_ZERO;
  st->pll_speed     = IQ_ZERO;
  st->pll_integral  = IQ_ZERO;
  st->pll_kp        = _IQ(0.1);     // 默认 PLL 比例增益
  st->pll_ki        = _IQ(0.001);   // 默认 PLL 积分增益
  st->elec_angle    = IQ_ZERO;
  st->mech_angle    = IQ_ZERO;
}

// 返回默认配置: 励磁 10kHz, LPF 500Hz, 2 对极, Q24 格式
//   sample_freq_hz: 控制/采样频率 (Hz), 用于预计算系数
static inline ResolverFixedCfg resolver_fixed_cfg_default(float sample_freq_hz) {
  ResolverFixedCfg cfg;
  cfg.excitation_freq_hz = 10000.0f;   // 励磁 10kHz
  cfg.sample_freq_hz     = sample_freq_hz;
  cfg.lpf_bw_hz          = 500.0f;     // LPF 带宽 500Hz
  cfg.pole_pairs         = 2;          // 默认 2 对极 (4 极电机)
  // 预计算系数 (Q24)
  //   lpf_k = 2π·fc/fs — 一阶 IIR 系数
  //   phase_step = f_exc/fs — DDS 每步相位增量
  cfg.lpf_k      = _IQ(6.2831853f * cfg.lpf_bw_hz / sample_freq_hz);
  cfg.phase_step = _IQ(cfg.excitation_freq_hz / sample_freq_hz);
  return cfg;
}

// DDS 励磁参考输出 — 每控制周期调用, 更新相位累加器并返回 sin 参考值
//   cfg: 配置 (含 phase_step)
//   st:  状态 (读写 exc_phase_acc, 写入 exc_ref)
//   返回: 当前励磁 sin 值 (Q24), 可用于 DAC 输出或 PWM 占空比计算
static inline _iq resolver_fixed_excite(ResolverFixedCfg *cfg,
                                        ResolverFixedState *st) {
  // 相位累加 — DDS 核心
  st->exc_phase_acc += cfg->phase_step;
  // 折叠到 [0, 1) per-unit 范围
  if (st->exc_phase_acc >= IQ_ONE) {
    st->exc_phase_acc -= IQ_ONE;
  }
  // 查表生成 sin 励磁参考信号
  st->exc_ref = _IQsinPU(st->exc_phase_acc);
  return st->exc_ref;
}

// 同步解调 + atan2 + PLL 跟踪 — 每控制周期调用 (在 excite 之后)
//   cfg:     配置 (含 lpf_k / pole_pairs)
//   st:      状态 (读写解调/PLL 字段, 使用 exc_ref)
//   sin_adc: ADC 采样的 sin 绕组信号 (Q24, 载波调制后的原始值)
//   cos_adc: ADC 采样的 cos 绕组信号 (Q24)
//   返回: 电角度 (Q24, per-unit [0, 1))
//
//   注意: sin_adc/cos_adc 应是与励磁峰值同步采样的值,
//   或在载波周期内任意点采样后通过本函数的乘参考+LPF方式解调
static inline _iq resolver_fixed_demodulate(ResolverFixedCfg *cfg,
                                            ResolverFixedState *st,
                                            _iq sin_adc, _iq cos_adc) {
  // 第1步: 同步解调 — ADC 信号 × 励磁参考
  //   物理: sin_adc·sin(ωt) → sin(θ)·sin²(ωt) = sin(θ)·(1-cos(2ωt))/2
  //   同理 cos_adc·sin(ωt) → cos(θ)·(1-cos(2ωt))/2
  //   DC 项 = sin(θ)/2 或 cos(θ)/2, 2ω 项将被 LPF 滤除
  _iq sin_demod = _IQmpy(sin_adc, st->exc_ref);
  _iq cos_demod = _IQmpy(cos_adc, st->exc_ref);

  // 存储瞬时解调值 (调试用)
  st->sin_dc = sin_demod;
  st->cos_dc = cos_demod;

  // 第2步: 一阶 IIR 低通滤波 — 滤除 2ω 载波残余
  //   y[n] = y[n-1] + k·(x[n] - y[n-1])
  //   截止频率 fc = k·fs/(2π), 即 k = 2π·fc/fs
  st->sin_dc_filt += _IQmpy(cfg->lpf_k, sin_demod - st->sin_dc_filt);
  st->cos_dc_filt += _IQmpy(cfg->lpf_k, cos_demod - st->cos_dc_filt);

  // 第3步: atan2 查表计算瞬时角度 (per-unit [0, 1))
  //   来源: comp_iqmath.h — 512 点查表 + 线性插值, 精度 ~1e-4
  _iq angle_meas = _IQatan2PU(st->sin_dc_filt, st->cos_dc_filt);

  // 第4步: PLL 锁相环 — 抑制测量噪声, 同时估计速度
  //   误差 = 测量角 - 跟踪角, 折叠到 [-0.5, 0.5) 选择最短路径
  _iq error = angle_meas - st->pll_angle;
  if (error > IQ_HALF) {
    error -= IQ_ONE;
  } else if (error < -IQ_HALF) {
    error += IQ_ONE;
  }
  // PI 控制器: 速度 = Kp·error + ∫Ki·error
  st->pll_integral += _IQmpy(st->pll_ki, error);
  st->pll_speed = _IQmpy(st->pll_kp, error) + st->pll_integral;
  // 更新 PLL 角度
  st->pll_angle += st->pll_speed;
  // 折叠 PLL 角度到 [0, 1)
  if (st->pll_angle >= IQ_ONE) {
    st->pll_angle -= IQ_ONE;
  } else if (st->pll_angle < IQ_ZERO) {
    st->pll_angle += IQ_ONE;
  }

  // 第5步: 输出
  //   电角度 = PLL 跟踪角度 (比瞬时 atan2 更平滑)
  st->elec_angle = st->pll_angle;
  //   机械角度 = 电角度 / 极对数 (单电周期内, [0, 1/pole_pairs))
  //   注: 多圈绝对位置需外部累加电周期计数
  st->mech_angle = _IQdiv(st->elec_angle, _IQ24fromI(cfg->pole_pairs));

  return st->elec_angle;
}

// 获取电角度, _iq → float 弧度
//   st: 状态 (只读 elec_angle)
//   返回: 电角度 (rad, [0, 2π))
static inline float resolver_fixed_get_angle_rad(const ResolverFixedState *st) {
  return _IQtoF(st->elec_angle) * M_2PI;
}

// 获取电角速度, _iq → float rad/s
//   cfg: 配置 (含 sample_freq_hz 用于标度转换)
//   st:  状态 (只读 pll_speed)
//   返回: 电角速度 (rad/s)
static inline float resolver_fixed_get_speed(const ResolverFixedCfg *cfg,
                                             const ResolverFixedState *st) {
  // pll_speed 单位: per-unit/控制周期
  // rad/s = pu/周期 × 2π rad/pu × fs 周期/s
  return _IQtoF(st->pll_speed) * M_2PI * cfg->sample_freq_hz;
}

#endif  // COMP_RESOLVER_H
