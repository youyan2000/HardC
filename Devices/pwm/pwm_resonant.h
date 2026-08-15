// 谐振变换器变频 PWM —— PwmBase 子类
//
// 拓扑: 通过调节开关频率改变谐振腔阻抗, 从而控制输出电压/功率
// 占空比固定 50% (上下管各半周期), 死区防止直通
//
// 应用:
//   LLC 谐振变换器
//   双有源桥 (DAB) — 变频 + 移相组合控制
//   LCC / CLLC 谐振
//
// 控制原理:
//   频率 ↑ → 谐振腔增益 ↓ → 输出电压 ↓
//   频率 ↓ → 谐振腔增益 ↑ → 输出电压 ↑ (接近谐振点时增益最大)
//
// 可调参数:
//   - 开关频率 (Hz, 核心控制量, 在 [freq_min, freq_max] 范围内调节)
//   - 死区时间 (ns, 固定值, 保证 ZVS 软开关)
//   - 占空比 (固定 50%, 如需微调则走基类 set_duty 接口)
//
// 变频约束:
//   频率范围受限于: 谐振腔参数 (Lr, Cr, Lm) + 磁性元件饱和 + 开关损耗
//   典型范围: 50kHz ~ 300kHz (LLC), 100kHz ~ 500kHz (高频 GaN)

#ifndef PWM_RESONANT_H
#define PWM_RESONANT_H

#include "comp_pwm.h"
#include "bsp_pwm.h"

// 来源: TI controlSUITE PWMDRV_LLC
// LLC burst mode 状态
typedef enum {
  LlcBurstMode_Off,          // 正常连续运行
  LlcBurstMode_Burst,        // burst 模式 (跳周期)
  LlcBurstMode_Auto,         // 自动: 轻载→burst, 重载→连续
} LlcBurstMode;

// ZVS 检测状态 (来源: TI controlSUITE PWMDRV_LLC ZVS sense)
typedef enum {
  LlcZvsState_Unknown,       // 未检测 / 初始状态
  LlcZvsState_Achieved,      // ZVS 成功 (Vds=0 在开通前)
  LlcZvsState_HardSwitch,    // 硬开关 (Vds>0, 有开关损耗)
  LlcZvsState_Marginal,      // 临界 (Vds 接近 0, 裕量不足)
} LlcZvsState;

// ======== 子类结构体 —— base 必须是第一个成员 ========
typedef struct {
  PwmBase  base;               // 基类 (必须为第一个成员)

  // BSP 硬件绑定
  BspPwmConfig bsp_cfg;
  BspPwmTimer  timer;          // 硬件定时器编号
  uint32_t     output_mask;    // 输出通道掩码 (互补对)

  // 电力电子参数 (变频是核心)
  uint32_t freq_hz;            // 当前开关频率
  uint32_t freq_min;           // 最低频率 (Hz, 物理约束: 变压器饱和 / 听觉噪声)
  uint32_t freq_max;           // 最高频率 (Hz, 物理约束: 开关损耗 / 死区占比)
  uint32_t deadtime_ns;        // 死区 (ns, 保证 ZVS 软开关的最小死区)
  uint32_t period;             // PWM 周期 (BSP 计数值)

  // 谐振参数 (可选 —— 用于自适应频率计算)
  float    resonant_freq;      // 谐振频率 (Hz, fr = 1 / (2π√LrCr))
  bool     below_resonant;     // true=低于谐振点工作 (ZCS区), false=高于谐振点 (ZVS区)

  // === LLC burst mode (控制层访问) ===
  LlcBurstMode burst_mode;       // burst 模式选择
  float        burst_threshold;  // 进入 burst 的功率阈值 (pu, 0~1)
  int          burst_on_cycles;  // burst ON 持续周期数
  int          burst_off_cycles; // burst OFF 跳过周期数
  int          burst_counter;    // 内部计数器
  bool         burst_active;     // 当前是否在 burst ON 阶段

  // === ZVS 检测 (来源: TI controlSUITE PWMDRV_LLC ZVS sense) ===
  LlcZvsState zvs_state;         // 当前 ZVS 状态
  int         zvs_fail_count;    // ZVS 失败累计 (连续硬开关次数)
  int         zvs_fail_limit;    // ZVS 失败上限 (超过则紧急停机)
  bool        zvs_sense_enable;  // 使能 ZVS 检测
  float       zvs_vds_threshold; // Vds 硬开关判断阈值 (V, 典型 5~20V)
} PwmResonant;

// ======== 构造 ========

// 初始化谐振变频 PWM:
//   freq_start:  起始开关频率 (通常略高于谐振频率)
//   freq_min:    最低频率限制
//   freq_max:    最高频率限制
//   deadtime_ns: 死区 (ns, 保证 ZVS 的最小值)
//   timer:       硬件定时器
//   output_mask: 输出通道
void pwm_res_init(PwmResonant *me,
                  uint32_t freq_start, uint32_t freq_min, uint32_t freq_max,
                  uint32_t deadtime_ns,
                  BspPwmTimer timer, uint32_t output_mask);

void pwm_res_deinit(PwmResonant *me);

// ======== 运行时调参 (变频为主) ========

// 设置开关频率 (核心控制接口)
void pwm_res_set_freq(PwmResonant *me, uint32_t freq_hz);

// 设置死区
void pwm_res_set_deadtime(PwmResonant *me, uint32_t deadtime_ns);

// 获取当前频率
uint32_t pwm_res_get_freq(PwmResonant *me);

// 获取频率范围
void pwm_res_get_freq_range(PwmResonant *me, uint32_t *freq_min, uint32_t *freq_max);

// 设置谐振参数 (用于自适应控制)
void pwm_res_set_resonant_params(PwmResonant *me, float resonant_freq, bool below_resonant);

// 配置 burst mode
void pwm_resonant_set_burst(PwmResonant *me, LlcBurstMode mode,
                             float threshold_pu, int on_cycles, int off_cycles);

// 每 PWM 周期调用 — ISR 热路径
// 返回 true = 本周期应输出 PWM, false = 跳过 (burst OFF)
bool pwm_resonant_burst_tick(PwmResonant *me);

// 自动 burst 判断 — ISR 中每周期调用
// power_pu: 当前输出功率标幺值 (0~1)
// 返回: true = burst 状态发生变化
bool pwm_resonant_burst_auto_update(PwmResonant *me, float power_pu);

// ZVS 检测 — ISR 中在开关管开通前调用
// vds_sample: 当前 Vds 采样值 (V, 通过 ADC/比较器读取)
// 返回: 当前 ZVS 状态
LlcZvsState pwm_resonant_zvs_detect(PwmResonant *me, float vds_sample);

// 配置 ZVS 检测参数
void pwm_resonant_zvs_config(PwmResonant *me, bool enable,
                              float vds_threshold_v, int fail_limit);

// 自适应死区 — 根据 ZVS 状态调整死区
// 返回: 推荐死区 (ns), 如果 ZVS 裕量不足则增加死区
uint32_t pwm_resonant_adaptive_deadtime(PwmResonant *me);

#endif
