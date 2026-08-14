// 电机控制 — 电流模型 (转子磁链角度估计)
//
// 来源: TI controlSUITE motor_control/math_blocks/v4.3 (cur_mod.h, cur_const.h)
// 翻译为 HardC 纯C float inline 版本
//
// 算法 (异步电机矢量控制核心):
//   Imd(k+1) = Imd(k) + Kr × (Ids - Imd(k))     ← 励磁电流一阶 LPF
//   Wslip = Kt × Iqs / Imd                         ← 转差计算
//   We = Wr + Wslip                                ← 定子同步频率
//   Theta(k+1) = Theta(k) + K × We                ← 角度积分
//   wrap Theta to [0, 1)
//
// 调用方式:
//   cur_mod_run(&cm, ids, iqs, wr);  // ids=d轴电流,iqs=q轴电流,wr=转子电角速(标幺)
//   float theta = cm.theta;          // 转子磁链角度 (标幺 0~1)

#ifndef COMP_CURMOD_H
#define COMP_CURMOD_H

#include "comp_angle.h"
#include "comp_math.h"

// ======================= CurMod (电流模型磁链观测器) =======================

typedef struct {
  // 输入
  float ids;              // 输入: 同步旋转 d 轴电流 (标幺)
  float iqs;              // 输入: 同步旋转 q 轴电流 (标幺)
  float wr;               // 输入: 转子电角速度 (标幺)

  // 内部状态
  float imd;              // 励磁电流 (d 轴磁化电流估计值)

  // 输出
  float theta;            // 输出: 转子磁链角度 (标幺, 0~1)
  float w_slip;           // 输出: 转差 (标幺)
  float we;               // 输出: 定子同步频率 (标幺)

  // 参数 (从电机参数计算)
  float kr;               // 励磁电流 LPF 系数 = Ts / Tr
  float kt;               // 转差系数 = 1 / (Tr × 2π × fb)
  float k;                // 角度积分系数 = Ts × fb
} CurMod;

#define CUR_MOD_DEFAULTS { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }

// 初始化 — 从电机参数计算系数
//   rr:  转子电阻 (Ω)
//   lr:  转子电感 (H)
//   fb:  基频 (Hz), = 额定频率
//   ts:  采样周期 (s)
static inline void cur_mod_init(CurMod *me, float rr, float lr,
                                 float fb, float ts) {
  float tr = lr / rr;                                   // 转子时间常数
  me->kr = ts / tr;                                      // 励磁电流 LPF 系数
  me->kt = 1.0f / (tr * M_2PI * fb);                    // 转差系数
  me->k  = ts * fb;                                      // 角度积分系数

  me->ids = 0.0f;
  me->iqs = 0.0f;
  me->wr = 0.0f;
  me->imd = 0.0f;
  me->theta = 0.0f;
  me->w_slip = 0.0f;
  me->we = 0.0f;
}

// 电流模型单步运行
//   ids: d 轴电流 (励磁分量, 标幺)
//   iqs: q 轴电流 (转矩分量, 标幺)
//   wr:  转子电角速度 (标幺)
//   返回: 转子磁链角度 (标幺 0~1)
static inline float cur_mod_run(CurMod *me, float ids, float iqs, float wr) {
  me->ids = ids;
  me->iqs = iqs;
  me->wr = wr;

  // 阶段 1: 励磁电流估计 (一阶 LPF 模拟转子磁链惯性)
  // Imd(k+1) = Imd(k) + Kr × (Ids - Imd(k))
  me->imd += me->kr * (me->ids - me->imd);

  // 阶段 2: 转差计算
  // Wslip = Kt × Iqs / Imd
  if (me->imd != 0.0f) {
    me->w_slip = me->kt * me->iqs / me->imd;
  } else {
    me->w_slip = 0.0f;
  }

  // 阶段 3: 定子同步频率 = 转子频率 + 转差
  me->we = me->wr + me->w_slip;

  // 阶段 4: 角度积分
  me->theta += me->k * me->we;

  // 阶段 5: 角度归算 [0, 1)
  me->theta = angle_wrap(me->theta);

  return me->theta;
}

// 重置
static inline void cur_mod_reset(CurMod *me) {
  me->imd = 0.0f;
  me->theta = 0.0f;
  me->w_slip = 0.0f;
  me->we = 0.0f;
}

#endif  // COMP_CURMOD_H
