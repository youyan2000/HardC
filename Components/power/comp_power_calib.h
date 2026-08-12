// 结果级校准 — PowerCalib 每相校准 POD + 死区减法应用
//
// 来源: TI C2000Ware Digital Power SDK
//   libraries/energy-metrology_library/energy_metrology_f28p55
//   (metrology_nv_structs.h calibrationData + metrology_calibration.h
//    applyCalibrationPhase: deadband = mean·scale, <offset → 0, 否则 −offset)
// 翻译为 C-OOP 纯C float 版本 (TI 的 q11/q31 定点分数缩放简化)
//
// 校准在"结果"上做死区减法, 不是逐采样:
//   result = mean·scale;  |result| < offset → 0;  否则朝零方向减 offset
// 死区消除小信号偏置 (噪声/串扰残余), 保证空载读数归零
//
// 死区保符号: 有功/无功/基波功率是有符号量 (馈出/容性为负). TI 的字面
// 死区 (r < offset → 0) 会把负读数误归零, 本组件改为对称死区 |r| < offset
// → 0, 真实负值按原符号直通 (仅当 |r| ≥ offset 才减 offset, 方向朝零)
//
// 简化说明: TI 的相位偏移用 256 项 FIR 系数表 (须按采样率重生成) 做亚采样
// 相位修正 — 本组件不处理相位, 相位偏移由测量层 (comp_power_meas.h 的
// quad_delay) 折算, 头注释记录该简化
//
// NV 持久化: 本 POD 即 TI 的可持久化结构体 — TI 只持久化校准数据 (结构体
// 魔数 0x59 + 校准初始化标志 0xABCD, 无 CRC), 能量计数是 RAM-only 不掉电
// 保持. flash 读写 + 完整性校验 (可选复用 comp_crc.h) 是应用层职责

#ifndef COMP_POWER_CALIB_H
#define COMP_POWER_CALIB_H

#include <math.h>

// ======================= PowerCalibPhase (每相校准数据) =======================

typedef struct {
  // 电压
  float v_scale;          // 电压比例因子 (raw → V)
  float vac_offset;       // VRMS 交流偏置 (宽频, 死区)
  float v_fac_offset;     // FRMS 基波偏置 (死区)

  // 电流
  float i_scale;          // 电流比例因子 (raw → A)
  float iac_offset;       // IRMS 交流偏置 (宽频, 死区)
  float i_fac_offset;     // FRMS 基波偏置 (死区)

  // 功率
  float p_scale;          // 功率比例因子 (raw → W/VAR)
  float active_offset;    // 宽频有功偏置 (死区)
  float reactive_offset;  // 宽频无功偏置 (死区)
  float f_active_offset;  // 基波有功偏置 (死区)
  float f_reactive_offset;// 基波无功偏置 (死区)

  // DC 滤波器种子
  float v_dc_init;        // 电压 DC 滤波器初始估计 (启动时喂给测量层)
  float i_dc_init;        // 电流 DC 滤波器初始估计
} PowerCalibPhase;

// 恒等默认 (scale=1, offsets=0) — 未校准时原值直通
static inline void power_calib_init(PowerCalibPhase *me) {
  me->v_scale = 1.0f;
  me->vac_offset = 0.0f;
  me->v_fac_offset = 0.0f;

  me->i_scale = 1.0f;
  me->iac_offset = 0.0f;
  me->i_fac_offset = 0.0f;

  me->p_scale = 1.0f;
  me->active_offset = 0.0f;
  me->reactive_offset = 0.0f;
  me->f_active_offset = 0.0f;
  me->f_reactive_offset = 0.0f;

  me->v_dc_init = 0.0f;
  me->i_dc_init = 0.0f;
}

// ---- 死区减法 (所有应用函数共用) ----

// 对称死区: |raw·scale| < offset → 0; 否则减 offset, 方向朝零 (保符号)
static inline float power_calib_deadband(float raw, float scale, float offset) {
  float r = raw * scale;
  if (fabsf(r) < offset) {
    return 0.0f;
  }
  return (r >= 0.0f) ? r - offset : r + offset;
}

// 宽频电压有效值校准: VRMS = deadband(raw, v_scale, vac_offset)
static inline float power_calib_apply_vrms(const PowerCalibPhase *me, float raw) {
  return power_calib_deadband(raw, me->v_scale, me->vac_offset);
}

// 基波电压有效值校准: FRMS_V = deadband(raw, v_scale, v_fac_offset)
static inline float power_calib_apply_frms_v(const PowerCalibPhase *me, float raw) {
  return power_calib_deadband(raw, me->v_scale, me->v_fac_offset);
}

// 宽频电流有效值校准: IRMS = deadband(raw, i_scale, iac_offset)
static inline float power_calib_apply_irms(const PowerCalibPhase *me, float raw) {
  return power_calib_deadband(raw, me->i_scale, me->iac_offset);
}

// 基波电流有效值校准: FRMS_I = deadband(raw, i_scale, i_fac_offset)
static inline float power_calib_apply_frms_i(const PowerCalibPhase *me, float raw) {
  return power_calib_deadband(raw, me->i_scale, me->i_fac_offset);
}

// 宽频有功校准: P = deadband(raw, p_scale, active_offset)
static inline float power_calib_apply_p(const PowerCalibPhase *me, float raw) {
  return power_calib_deadband(raw, me->p_scale, me->active_offset);
}

// 宽频无功校准: Q = deadband(raw, p_scale, reactive_offset)
static inline float power_calib_apply_q(const PowerCalibPhase *me, float raw) {
  return power_calib_deadband(raw, me->p_scale, me->reactive_offset);
}

// 基波有功校准: FP = deadband(raw, p_scale, f_active_offset)
static inline float power_calib_apply_fp(const PowerCalibPhase *me, float raw) {
  return power_calib_deadband(raw, me->p_scale, me->f_active_offset);
}

// 基波无功校准: FQ = deadband(raw, p_scale, f_reactive_offset)
static inline float power_calib_apply_fq(const PowerCalibPhase *me, float raw) {
  return power_calib_deadband(raw, me->p_scale, me->f_reactive_offset);
}

#endif  // COMP_POWER_CALIB_H
