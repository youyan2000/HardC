// 单开关变频 PWM —— PwmBase 子类, SEPIC / 反激 / 单管正激
//
// 来源: TI controlSUITE DPLib PWMDRV_1ch / PWMDRV_1ch_UpDnCnt (digital_power)
// 翻译为 HardC 风格
//
// 拓扑: 单开关管 + 二极管 + LC 耦合电容
// 应用: SEPIC DC/DC (升降压, 输入输出同极性), 反激, 单管正激
//
// 与普通 Buck PWM 的本质区别:
//   1. 变频运行: 轻载降频提效, 重载升频控纹波
//   2. CCM/DCM 边界检测: 电感电流过零判断, 决定同步整流/断续模式
//   3. 单通道: 只有一个开关管 (非互补, 非多相)
//
// 接口:
//   sepic_set_point(&sepic, duty, freq_hz);  // 同时设置占空比+频率 (变频模式)
//   sepic_set_duty(&sepic, duty);             // 仅设占空比 (定频模式)

#ifndef PWM_SEPIC_H
#define PWM_SEPIC_H

#include "comp_pwm.h"
#include "bsp_pwm.h"

// SEPIC 工作模式
typedef enum {
  SepicMode_CCM,          // 连续导通模式 (电感电流不归零)
  SepicMode_DCM,          // 断续导通模式 (电感电流归零, 低功率时自动进入)
  SepicMode_BCM,          // 临界导通模式 (谷值开关, 变频控制)
} SepicMode;

// SEPIC 单开关变频 PWM 子类
typedef struct {
  PwmBase base;                     // 基类 (必须为第一个成员)

  // BSP 硬件绑定
  BspPwmConfig bsp_cfg;             // BSP 配置
  BspPwmTimer  timer;               // 单定时器 (1 个通道)
  uint32_t     output_mask;         // 输出通道掩码

  // 变频相关 — SEPIC 特有
  uint32_t freq_min_hz;             // 最低开关频率 (Hz, 如 20000)
  uint32_t freq_max_hz;             // 最高开关频率 (Hz, 如 200000)
  uint32_t freq_curr_hz;            // 当前运行频率 (Hz)
  float    freq_step_hz;            // 变频步长 (Hz, 如 1000)

  // 占空比
  float    duty;                    // 当前占空比 [0, 1]
  uint32_t period;                  // PWM 周期 (BSP 计数值)

  // 模式
  SepicMode mode;                   // CCM / DCM / BCM
  bool      freq_variable;          // 是否启用变频 (false = 定频模式)

  // CCM/DCM 边界检测 (需外部 ADC 采样提供电感电流)
  float    i_inductor;              // 电感电流 (A, 外部写入)
  float    i_threshold_dcm;         // DCM 判定阈值 (A, 低于此值进入 DCM)
} PwmSepic;

// 构造
//   freq_hz: 初始开关频率
//   duty:    初始占空比 (启动时通常为 0)
//   timer:   硬件定时器编号
//   output_mask: 输出通道掩码
void sepic_init(PwmSepic *me, uint32_t freq_hz, float duty,
                BspPwmTimer timer, uint32_t output_mask);

// 反初始化
void sepic_deinit(PwmSepic *me);

// 设置占空比 (定频模式, 只改占空比)
void sepic_set_duty(PwmSepic *me, float duty);

// 设置占空比 + 频率 (变频模式, SEPIC 核心接口)
// 频率不超 [freq_min_hz, freq_max_hz] 范围
void sepic_set_point(PwmSepic *me, float duty, uint32_t freq_hz);

// 自动变频: 根据电感电流纹波比决定最优频率
//   i_inductor: 当前电感电流 (A)
//   返回: 选择的频率 (Hz)
uint32_t sepic_auto_freq(PwmSepic *me, float i_inductor);

// CCM/DCM 模式检测 (由外部在 ISR 中调用, 基于电感电流采样)
void sepic_detect_mode(PwmSepic *me, float i_inductor);

#endif  // PWM_SEPIC_H
