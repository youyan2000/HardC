// PFC 功率因数校正算法块
//
// 来源: TI controlSUITE digital_power (PFC_ICMD_F, PFC_BL_ICMD_F,
//   PFC_InvRmsSqr, PFC_INVSQR) (f28x7x_v1.0/C_macros)
// 翻译为 HardC 纯C float static inline 版本
//
// 块说明:
//   PfcICmd     — PFC 电流指令生成 (Vcmd × VinvSqr × VacRect × VmaxOverVmin)
//   PfcBLICmd   — 升压开关电流指令 (含占空比前馈)
//   PfcInvRmsSqr — 输入 RMS² 倒数 (包含最小值限制)
//   PfcInvSqr   — 带 HALF_PI 缩放的平方倒数

#ifndef COMP_PFC_H
#define COMP_PFC_H

#include <math.h>
#include "comp_math.h"

// ======================= PfcICmd (电流指令) =======================

// 标准 PFC 电流参考计算:
//   Iref = Vcmd × VinvSqr × VacRect × (Vmax/Vmin)
// 其中:
//   Vcmd:     电压环输出 (控制信号)
//   VinvSqr:  1/Vrms² (输入电压 RMS 倒数之方, 前馈解耦)
//   VacRect:  整流后的瞬时电压波形 (|sin(ωt)|)
//   VmaxOverVmin: 最大/最小输入电压比 (防止轻载过调制)

typedef struct {
  float v_cmd;            // 输入: 电压环输出
  float v_inv_sqr;        // 输入: 1/Vrms² (输入 RMS 倒数方)
  float v_ac_rect;        // 输入: 整流后瞬时电压
  float vmax_over_vmin;   // 参数: 最大/最小输入电压比
  float out;              // 输出: 电流参考
} PfcICmd;

#define PFC_ICMD_DEFAULTS { 0, 0, 0, 1.0f, 0 }

// Iref = Vcmd × (1/Vrms²) × |Vac| × (Vmax/Vmin)
static inline float pfc_icmd_run(PfcICmd *me, float v_cmd, float v_inv_sqr,
                                  float v_ac_rect) {
  me->v_cmd = v_cmd;
  me->v_inv_sqr = v_inv_sqr;
  me->v_ac_rect = v_ac_rect;

  me->out = v_cmd * v_inv_sqr * v_ac_rect * me->vmax_over_vmin;
  return me->out;
}

// ======================= PfcBLICmd (升压开关电流指令) =======================

// 含占空比前馈的 PFC 电流指令 (用于升压拓扑)
// 公式:
//   Vpfc_term = Vpfc × (VoutMax/VinMax) × Duty
//   Iref = (VacRect × VinvSqr × Vcmd × VmaxOverVmin)
//        × 1/Vpfc_term
//        × (Vpfc × VoutMax/VinMax - VacRect)

typedef struct {
  float v_cmd;
  float v_inv_sqr;
  float v_ac_rect;
  float duty;             // 输入: 当前占空比
  float v_pfc;            // 输入: PFC 输出电压
  float vmax_over_vmin;
  float vout_max_over_vin_max;  // 参数: VoutMax / VinMax
  float out;              // 输出: 电流参考
} PfcBLICmd;

#define PFC_BL_ICMD_DEFAULTS { 0, 0, 0, 0, 0, 1.0f, 1.0f, 0 }

static inline float pfc_bl_icmd_run(PfcBLICmd *me, float v_cmd, float v_inv_sqr,
                                     float v_ac_rect, float duty, float v_pfc) {
  me->v_cmd = v_cmd;
  me->v_inv_sqr = v_inv_sqr;
  me->v_ac_rect = v_ac_rect;
  me->duty = duty;
  me->v_pfc = v_pfc;

  // Vpfc_term = Vpfc × (VoutMax/VinMax) × Duty, 下限 0.005
  float vpfc_term = v_pfc * me->vout_max_over_vin_max * duty;
  if (vpfc_term < 0.005f) vpfc_term = 0.005f;

  // 前馈项
  float ff = v_ac_rect * v_inv_sqr * v_cmd * me->vmax_over_vmin;

  // 电压差
  float v_diff = v_pfc * me->vout_max_over_vin_max - v_ac_rect;

  me->out = ff * (1.0f / vpfc_term) * v_diff;
  return me->out;
}

// ======================= PfcInvRmsSqr (RMS² 倒数) =======================

// 输入 RMS 平方的倒数, 带最小限制
// 公式: Vtmp = max(Vin, Vmin)
//       Vout = (1/Vtmp × Vmin/Vmax)², 上限 1.0

typedef struct {
  float in;               // 输入: RMS 电压
  float vmin;             // 参数: 最小电压限制
  float vmin_over_vmax;   // 参数: Vmin / Vmax
  float out;              // 输出: 1/Vrms² (标幺)
} PfcInvRmsSqr;

#define PFC_INV_RMS_SQR_DEFAULTS { 0, 0.1f, 0.1f, 0 }

static inline float pfc_inv_rms_sqr_run(PfcInvRmsSqr *me, float vrms) {
  me->in = vrms;

  // 限下
  float vtmp = (me->vmin > vrms) ? me->vmin : vrms;

  // 归一化倒数
  float vinv = (1.0f / vtmp) * me->vmin_over_vmax;

  // 平方 → 上限 1.0
  me->out = vinv * vinv;
  if (me->out > 1.0f) me->out = 1.0f;

  return me->out;
}

// ======================= PfcInvSqr (带 HALF_PI 缩放的平方倒数) =======================

// 公式: Vtmp = max(Vin, Vmin)
//       Vout = (1/(Vtmp × π/2) × Vmin/Vmax)², 上限 1.0

typedef struct {
  float in;
  float vmin;
  float vmin_over_vmax;
  float out;
} PfcInvSqr;

#define PFC_INV_SQR_DEFAULTS { 0, 0.1f, 0.1f, 0 }
#define HALF_PI_PFC  1.570796327f

static inline float pfc_inv_sqr_run(PfcInvSqr *me, float vrms) {
  me->in = vrms;

  float vtmp = (me->vmin > vrms) ? me->vmin : vrms;

  float vinv = (1.0f / (vtmp * HALF_PI_PFC)) * me->vmin_over_vmax;

  me->out = vinv * vinv;
  if (me->out > 1.0f) me->out = 1.0f;

  return me->out;
}

// ======================= PfcBlIcmd (无桥 PFC 电流指令) =======================

// 无桥 PFC 电流指令 (Bridgeless)
//
// 来源: TI controlSUITE digital_power PFC_BL_ICMD (汇编→C翻译)
//
// 无桥 PFC 无整流桥, 两开关分别处理正/负半周:
//   正半周 (Vin>0): 开关 A 工作, icmd = Vloop × Vin / Vrms²
//   负半周 (Vin<0): 开关 B 工作, icmd = |Vloop × Vin / Vrms²|
//   两路电流指令互斥, 每半周只有一个开关有电流
//
// 注意: 此 Component 只计算电流指令, 不直接控制 PWM 占空比
//   PWM 占空比由外部电流闭环 PID 计算

typedef struct {
  float icmd_pos;         // 正半周电流指令 (A), 负半周时为 0
  float icmd_neg;         // 负半周电流指令 (A), 正半周时为 0
  bool  pos_active;       // 当前正半周激活 (Vin > 0)
} PfcBlIcmd;

#define PFC_BL_ICMD_BRIDGELESS_DEFAULTS { 0, 0, false }

// 计算无桥 PFC 两路电流指令
//   me:       状态指针
//   vin:      交流输入电压瞬时值 (V)
//   vin_rms:  输入电压有效值 (V)
//   vloop:    电压环输出 (标幺 0~1, 或实际电流幅值指令)
//
// 内部自动计算 1/Vrms² 前馈 (需要 vin_rms > 0.1V 防止除零)
static inline void pfc_bl_icmd_bridgeless_run(PfcBlIcmd *me, float vin,
                                               float vin_rms, float vloop) {
  // 半周判断
  me->pos_active = (vin >= 0.0f);

  // 1/Vrms² 前馈 (防除零)
  float inv_vrms_sq;
  if (vin_rms > 0.1f) {
    inv_vrms_sq = 1.0f / (vin_rms * vin_rms);
  } else {
    inv_vrms_sq = 1.0f;   // 启动/低压时不做前馈
  }

  // 电流指令 = Vloop × |Vin| / Vrms²
  float icmd = vloop * MATH_ABS(vin) * inv_vrms_sq;

  // 分配至正/负半周
  if (me->pos_active) {
    me->icmd_pos = icmd;
    me->icmd_neg = 0.0f;
  } else {
    me->icmd_pos = 0.0f;
    me->icmd_neg = icmd;
  }
}

#endif  // COMP_PFC_H
