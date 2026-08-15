// 电机控制 — 相电压计算 (从调制函数反推实际电压)
//
// 来源: TI controlSUITE motor_control/math_blocks/v4.3 (volt_calc.h)
// 翻译为 HardC 纯C float inline 版本
//
// 从三相调制函数 Mfunc 和直流母线电压 DcBus 反推实际相电压:
//   VphaseA = (Vdc/3) × (2×MfuncA - MfuncB - MfuncC)
//   VphaseB = (Vdc/3) × (2×MfuncB - MfuncA - MfuncC)
//   Clarke: Vα = VphaseA,  Vβ = (VphaseA + 2×VphaseB) / √3
//
// 用途: 在无电压传感器的矢量控制中, 从 PWM 占空比重构电压反馈

#ifndef COMP_VOLT_H
#define COMP_VOLT_H

// ======================= PhaseVoltage (三相相电压计算) =======================

typedef struct {
  float dc_bus;           // 输入: 直流母线电压 (标幺)
  float mfunc_a;          // 输入: A 相调制函数 (标幺, -1~+1)
  float mfunc_b;          // 输入: B 相调制函数 (标幺)
  float mfunc_c;          // 输入: C 相调制函数 (标幺)

  float v_phase_a;        // 输出: A 相电压 (标幺)
  float v_phase_b;        // 输出: B 相电压 (标幺)
  float v_phase_c;        // 输出: C 相电压 (标幺)

  float v_alpha;          // 输出: α 轴电压 (标幺)
  float v_beta;           // 输出: β 轴电压 (标幺)

  // 内部
  float one_third;        // 1/3
  float inv_sqrt3;        // 1/√3
} PhaseVoltage;

#define PHASE_VOLTAGE_DEFAULTS { 0,0,0,0, 0,0,0, 0,0, 0.33333333f, 0.577350269f }

// 计算相电压 + Clarke → αβ
//   返回: Vβ (可用于后续 Park 变换)
static inline float phase_voltage_calc(PhaseVoltage *me, float dc_bus,
                                        float mf_a, float mf_b, float mf_c) {
  me->dc_bus = dc_bus;
  me->mfunc_a = mf_a;
  me->mfunc_b = mf_b;
  me->mfunc_c = mf_c;

  float temp = dc_bus * (1.0f / 3.0f);

  // 相电压 (去零序, 保留线电压)
  me->v_phase_a = temp * (2.0f * mf_a - mf_b - mf_c);
  me->v_phase_b = temp * (2.0f * mf_b - mf_a - mf_c);
  me->v_phase_c = temp * (2.0f * mf_c - mf_a - mf_b);

  // Clarke 变换: A,B → αβ
  me->v_alpha = me->v_phase_a;
  me->v_beta  = (me->v_phase_a + 2.0f * me->v_phase_b) * 0.577350269f;

  return me->v_beta;
}

#endif  // COMP_VOLT_H
