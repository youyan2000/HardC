// 电机控制 — V/Hz 特性曲线生成器
//
// 来源: TI controlSUITE motor_control/math_blocks/v4.3 (vhzprof.h)
// 翻译为 C-OOP 纯C float inline 版本
//
// 三段式 V/f 曲线:
//   区 1: |f| ≤ LowFreq  → 恒定升压电压 VoltMin (补偿定子电阻压降)
//   区 2: LowFreq < |f| ≤ HighFreq → 线性 V/f 斜坡
//   区 3: HighFreq < |f| ≤ FreqMax → 恒定额定电压 VoltMax (弱磁区)
//
// 调用方式:
//   float v_out = vhz_profile_run(&vhz, freq_cmd_hz);

#ifndef COMP_VHZ_H
#define COMP_VHZ_H

#include <math.h>

// ======================= VhzProfile (V/Hz 曲线) =======================

typedef struct {
  float freq;             // 输入: 频率指令 (Hz, 可正可负 — 符号决定旋转方向)
  float volt_out;         // 输出: 电压幅值 (标幺)
  float low_freq;         // 参数: 低频拐点 (Hz), 低于此频率恒定升压
  float high_freq;        // 参数: 高频拐点 (Hz), 达到额定电压的频率
  float freq_max;         // 参数: 最大频率 (Hz)
  float volt_max;         // 参数: 额定电压 (标幺)
  float volt_min;         // 参数: 低频升压电压 (标幺)

  // 内部计算值
  float vf_slope;         // V/f 斜率 = (Vmax - Vmin) / (Fhigh - Flow)
  float abs_freq;         // 频率绝对值
} VhzProfile;

#define VHZ_PROFILE_DEFAULTS { 0, 0, 5.0f, 50.0f, 100.0f, 1.0f, 0.1f, 0, 0 }

// 初始化 V/Hz 曲线
//   low_freq:   低频拐点 (Hz), 如 5Hz
//   high_freq:  高频拐点 (Hz), 如 50Hz
//   freq_max:   最大频率 (Hz), 如 100Hz
//   volt_max:   额定电压 (标幺, 0~1), 如 1.0
//   volt_min:   低频升压 (标幺), 如 0.1 (补偿定子电阻)
static inline void vhz_profile_init(VhzProfile *me, float low_freq,
                                     float high_freq, float freq_max,
                                     float volt_max, float volt_min) {
  me->low_freq = low_freq;
  me->high_freq = high_freq;
  me->freq_max = freq_max;
  me->volt_max = volt_max;
  me->volt_min = volt_min;
  me->vf_slope = 0.0f;
  me->abs_freq = 0.0f;
  me->volt_out = 0.0f;

  // 预计算斜率
  if (high_freq > low_freq) {
    me->vf_slope = (volt_max - volt_min) / (high_freq - low_freq);
  }
}

// V/Hz 单步运行 — 输入频率指令, 输出电压幅值
static inline float vhz_profile_run(VhzProfile *me, float freq_hz) {
  me->freq = freq_hz;
  me->abs_freq = fabsf(freq_hz);

  if (me->abs_freq <= me->low_freq) {
    // 区 1: 恒定升压 (补偿定子电阻压降, 保证低频转矩)
    me->volt_out = me->volt_min;
  } else if (me->abs_freq <= me->high_freq) {
    // 区 2: 线性 V/f (恒磁通控制)
    me->volt_out = me->volt_min + me->vf_slope * (me->abs_freq - me->low_freq);
  } else {
    // 区 3 (+超频): 恒定额定电压 (弱磁区, 电压已达上限)
    me->volt_out = me->volt_max;
  }

  return me->volt_out;
}

#endif  // COMP_VHZ_H
