#ifndef PWM_BUCKBOOST_H
#define PWM_BUCKBOOST_H

// 四开关 Buck/Boost PWM —— PwmBase 子类, 相位参数化 (phases: 1..3)
//
// 从第一天支持单相与三相: 一路相位 = 两个互补腿 (A 腿 + B 腿) = 两个 HRTIM 定时器.
// BUCK 时 B 腿恒通 (dutyB = duty_base), BOOST 时 A 腿恒通, BUCKBOOST 两腿交替
// (四开关). 3 相并联 = 6 定时器 → 需 G474 (6 定时器); F334 仅 5 定时器, 只支持 1 相.
// 单相 (N=1) 时相序/均流/切换同步全退化为无操作.
//
// 占空比律 (Device 内部, host 可测), ratio = vb/va, 钳位 [ratio_lo, ratio_hi]:
//   BUCK      : dutyA = duty_base × ratio,   dutyB = duty_base
//   BOOST     : dutyA = duty_base,            dutyB = duty_base / ratio
//   BUCKBOOST : dutyA = bb_gain × (ratio+1),  dutyB = bb_gain × (1/ratio+1)
// 模块每周期写 alpha (相位调制指数): 实际占空比 = 律值 × alpha, 上限 duty_max (0.97,
// 为死区/非理想留 3% 裕量). 律常量可用 pwm_bb_set_law_constants 覆盖.
//
// 相位: N 相均匀错相 360°/N, init 自动计算, start 时经 bsp_set_phase_shift 写入
//   两腿定时器. 真交错 (HRTIM CMP2 主复位连线) 是 BSP/工具链 AI 缺口 (AGENT-SYNC),
//   本层存相位并调 API, 对齐即正确.
//
// 接线契约: init 阶段不写硬件 (bsp_cfg.handle 为 NULL). App 在 board_init 注入
//   handle + clk_hz 后调用 pwm_bb_set_freq, 再 set_mode/ratio/alpha, 最后 start.

#include "comp_pwm.h"
#include "bsp_pwm.h"

// ======== 相位常量 ========
#define PWM_BB_MAX_PHASES 3  // 最大并联相数 (1 相=2 定时器, 3 相=6)

// ======== 相位配置 (init 时传入) ========
typedef struct {
  BspPwmTimer timer_a;  // A 腿互补对定时器
  uint32_t mask_a;      // A 腿输出掩码 (如 BSP_OUT_TA1 | BSP_OUT_TA2)
  BspPwmTimer timer_b;  // B 腿互补对定时器 (leg_b_used=true 时有效)
  uint32_t mask_b;      // B 腿输出掩码
  bool leg_b_used;      // true=四开关双腿; false=单腿 (Buck/Boost 简化)
} PwmBbPhaseCfg;

// ======== 运行时相位状态 (public, host 测试可读) ========
typedef struct {
  BspPwmTimer timer_a;  // A 腿定时器
  BspPwmTimer timer_b;  // B 腿定时器
  uint32_t mask_a;      // A 腿输出掩码
  uint32_t mask_b;      // B 腿输出掩码
  bool leg_b_used;      // B 腿是否启用
  float duty_a;         // 当前 A 腿占空比 (占空比域, 律值 × alpha)
  float duty_b;         // 当前 B 腿占空比
  float alpha;          // 调制指数 (模块写入, 1.0 名义)
  float phase_deg;      // 相位偏移 (度, 0 = 参考相)
} PwmBbPhase;

// ======== 子类结构体 —— base 必须是第一个成员 ========
typedef struct {
  PwmBase base;  // 基类 (必须为第一个成员)

  // BSP 硬件绑定 (App 注入 handle + clk_hz)
  BspPwmConfig bsp_cfg;

  uint8_t num_phases;  // 并联相数 (1..PWM_BB_MAX_PHASES)
  float ratio;         // 电压比 vb/va (占空比律输入, 钳位)
  PwmMode mode;        // 工作模式 (Buck/Boost/BuckBoost)
  bool sync_rect;      // true=同步整流 (互补输出 + 死区)

  // 电力电子参数
  uint32_t deadtime_ns;  // 死区 (仅同步整流有效)
  uint32_t period;       // PWM 周期 (BSP 计数值)
  bool center_aligned;   // true=中心对齐

  // 占空比律常量 (默认值, 可用 pwm_bb_set_law_constants 覆盖)
  float duty_base;       // 恒通腿占空比 (0.96)
  float buckboost_gain;  // 四开关模式增益 (0.33)
  float ratio_lo;        // 电压比下限 (0.73)
  float ratio_hi;        // 电压比上限 (1.37)
  float duty_max;        // 占空比上限 (0.97, 留死区裕量)

  PwmBbPhase phases[PWM_BB_MAX_PHASES];
} PwmBuckBoost;

// ======== 构造 ========

// 相位参数化初始化 (支持 N=1..3):
//   freq_hz:   开关频率 (在 App 注入 clk_hz 后经 pwm_bb_set_freq 生效)
//   num_phases: 并联相数
//   cfg:       每相两腿配置数组 (长度 num_phases)
//   mode:      初始工作模式 (Buck/Boost/BuckBoost)
//   sync_rect: 是否启用互补输出 + 死区
void pwm_bb_init_phases(PwmBuckBoost *me, uint32_t freq_hz, uint8_t num_phases, const PwmBbPhaseCfg *cfg, PwmMode mode,
                        bool sync_rect);

// N=1 兼容入口 (旧签名, 单相单腿): 内部转 init_phases
void pwm_bb_init(PwmBuckBoost *me, uint32_t freq_hz, BspPwmTimer timer, uint32_t output_mask, PwmMode mode,
                 bool sync_rect);

// 析构
void pwm_bb_deinit(PwmBuckBoost *me);

// ======== 占空比律 (运行时调参) ========

// 设置电压比 ratio = vb/va — 钳位 [ratio_lo, ratio_hi] 后重算全部相
void pwm_bb_set_ratio(PwmBuckBoost *me, float ratio);

// 切换工作模式 (Buck/Boost/BuckBoost) — 重算全部相
void pwm_bb_set_mode(PwmBuckBoost *me, PwmMode mode);

// 写某相调制指数 alpha — 重算该相两腿并写硬件 (28kHz 热路径)
void pwm_bb_set_alpha(PwmBuckBoost *me, uint8_t phase, float alpha);

// 覆盖占空比律常量 (默认 0.96/0.33/0.73/1.37/0.97) — 重算全部相
void pwm_bb_set_law_constants(PwmBuckBoost *me, float duty_base, float bb_gain, float ratio_lo, float ratio_hi,
                              float duty_max);

// ======== 运行时调参 (沿用 PwmBase 统一接口) ========

// 兼容入口: 写 phase[0] A 腿绝对占空比 (mod_buck 单相路径)
void pwm_bb_set_duty(PwmBuckBoost *me, float duty);
void pwm_bb_set_freq(PwmBuckBoost *me, uint32_t freq_hz);
void pwm_bb_set_deadtime(PwmBuckBoost *me, uint32_t deadtime_ns);

uint8_t pwm_bb_get_num_phases(const PwmBuckBoost *me);

#endif
