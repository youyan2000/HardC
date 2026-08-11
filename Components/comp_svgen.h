// 电机控制 — 空间矢量发生器 SVG (三种算法)
//
// 来源: TI controlSUITE motor_control/math_blocks/v4.3
//   (svgen_comm.h — 連續SVM, svgen_dpwm.h — 不连续PWM, svgen_mf.h — 幅频输入SVM)
// 翻译为 C-OOP 纯C float inline 版本
//
// 三种方案:
//   SvgenComm — 连续 SVM (中点注入), 开关次数多但谐波小
//   SvgenDpwm — 底钳位不连续 PWM (DPWMMIN), 开关次数少但谐波大
//   SvgenMf   — 幅值+频率直接输入, 自跟踪角度和扇区

#ifndef COMP_SVGEN_H
#define COMP_SVGEN_H

#include <math.h>

#ifndef SQRT3_OVER_2
#define SQRT3_OVER_2  0.86602540378444f
#endif

// ======================= SvgenComm (连续中点注入 SVM) =======================

// 连续 SVM — 通过注入共模电压 (Vmax+Vmin)/2, 实现等效空间矢量调制
// 优势: 波形质量好, 谐波小
// 劣势: 每个周期所有三相都在开关

typedef struct {
  float u_alpha;          // 输入: α 轴参考电压 (标幺)
  float u_beta;           // 输入: β 轴参考电压 (标幺)
  float ta;               // 输出: A 相开关函数 (标幺, -1~+1)
  float tb;               // 输出: B 相开关函数
  float tc;               // 输出: C 相开关函数

  // 内部
  float va, vb, vc;       // 三相相电压 (逆Clarke)
  float v_max, v_min;     // 最大/最小相电压
  float v_comm;           // 共模注入电压
} SvgenComm;

#define SVG_COMM_DEFAULTS { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }

// 连续 SVM 单步运行
//   输入 αβ 参考电压, 输出 Ta/Tb/Tc 开关函数
static inline void svgen_comm_run(SvgenComm *me, float u_alpha, float u_beta) {
  me->u_alpha = u_alpha;
  me->u_beta  = u_beta;

  // 步骤 1: 逆 Clarke (αβ → abc)
  float half_ua = u_alpha * 0.5f;
  float sqrt3_2_ub = SQRT3_OVER_2 * u_beta;

  me->va = u_alpha;
  me->vb = -half_ua + sqrt3_2_ub;    // -α/2 + √3/2·β
  me->vc = -half_ua - sqrt3_2_ub;    // -α/2 - √3/2·β

  // 步骤 2: 找最大/最小相
  me->v_max = me->va;
  me->v_min = me->va;
  if (me->vb > me->v_max) me->v_max = me->vb;
  if (me->vc > me->v_max) me->v_max = me->vc;
  if (me->vb < me->v_min) me->v_min = me->vb;
  if (me->vc < me->v_min) me->v_min = me->vc;

  // 步骤 3: 共模电压 = (Vmax+Vmin)/2 (中点注入)
  me->v_comm = (me->v_max + me->v_min) * 0.5f;

  // 步骤 4: 注入共模 → 开关函数
  me->ta = me->va - me->v_comm;
  me->tb = me->vb - me->v_comm;
  me->tc = me->vc - me->v_comm;
}

// ======================= SvgenDpwm (底钳位不连续 PWM) =======================

// DPWMMIN — 每 60° 扇区有一相钳位到负母线, 减少 1/3 开关损耗
// 算法: 扇区判断 → 选择钳位相 → 重新分配占空比

typedef struct {
  float u_alpha;
  float u_beta;
  float ta, tb, tc;

  // 内部
  float tmp1;             // = Ubeta
  float tmp2;             // = Ubeta/2 + √3/2*Ualpha
  float tmp3;             // = tmp2 - tmp1 = -Ubeta/2 + √3/2*Ualpha
  int   vec_sector;       // 扇区 1~6
} SvgenDpwm;

#define SVG_DPWM_DEFAULTS { 0, 0, 0, 0, 0, 0, 0, 0, 0 }

// DPWM 单步运行 (底钳位)
static inline void svgen_dpwm_run(SvgenDpwm *me, float u_alpha, float u_beta) {
  me->u_alpha = u_alpha;
  me->u_beta  = u_beta;

  // 步骤 1: 三相投影 (Clarke 参数形式)
  me->tmp1 = u_beta;
  me->tmp2 = u_beta * 0.5f + SQRT3_OVER_2 * u_alpha;     // b' = Va projection
  me->tmp3 = me->tmp2 - me->tmp1;                          // c' = Vc - Vb

  // 步骤 2: 扇区判断 (通过三相符号判定)
  // vec_sector: 1=(-++), 2=(+-+), 3=(++-), 4=(+--), 5=(-+-), 6=(--+)
  me->vec_sector = 3;
  if (me->tmp2 > 0.0f) me->vec_sector--;          // tmp2 > 0 → sector--
  if (me->tmp3 > 0.0f) me->vec_sector--;          // tmp3 > 0 → sector--
  if (me->tmp1 < 0.0f) me->vec_sector = 7 - me->vec_sector;  // tmp1 < 0 → flip

  // 步骤 3: 扇区映射 → 占空比 (0~1), 每个扇区有一相 = 0 (钳位到负母线)
  if (me->vec_sector == 1 || me->vec_sector == 6) {
    me->ta = 0.0f;           // A 相钳位
    me->tb = me->tmp3;
    me->tc = me->tmp2;
  } else if (me->vec_sector == 2 || me->vec_sector == 3) {
    me->ta = -me->tmp3;
    me->tb = 0.0f;           // B 相钳位
    me->tc = me->tmp1;
  } else {
    me->ta = -me->tmp2;
    me->tb = -me->tmp1;
    me->tc = 0.0f;           // C 相钳位
  }

  // 步骤 4: 缩放 [0,1] → [-1, +1] (PWM 期望输入)
  me->ta = 2.0f * me->ta - 1.0f;
  me->tb = 2.0f * me->tb - 1.0f;
  me->tc = 2.0f * me->tc - 1.0f;
}

// ======================= SvgenMf (幅值+频率输入 SVM) =======================

// 直接输入幅值和频率, 自跟踪角度和扇区
// 不需要外部提供 αβ 参考电压 — 内部自动生成三相正弦 + 共模注入

typedef struct {
  float gain;             // 输入: 电压幅值 (标幺)
  float freq;             // 输入: 频率 (标幺)
  float freq_max;         // 参数: 最大步长 = 6 × f_base × Ts
  float alpha;            // 内部: 当前扇区内角度 (标幺, 0~1)
  int   sector;           // 内部: 扇区号 (0~5)

  float ta, tb, tc;       // 输出: 三相开关函数

  // 内部计算
  float dx, dy;           // 扇区内矢量作用时间分量
  float step_angle;       // 步长 = freq × freq_max
} SvgenMf;

#define SVG_MF_DEFAULTS { 0, 0, 0.005f, 0, 0, 0, 0, 0, 0, 0, 0 }
#define PI_THIRD  1.047197551f

static inline void svgen_mf_init(SvgenMf *me, float freq_max) {
  me->gain = 0.0f;
  me->freq = 0.0f;
  me->freq_max = freq_max;
  me->alpha = 0.0f;
  me->sector = 0;
  me->ta = me->tb = me->tc = 0.0f;
  me->dx = me->dy = 0.0f;
  me->step_angle = 0.0f;
}

// MF SVM 单步: 输入 gain(幅值) 和 freq(频率), 输出 Ta/Tb/Tc
static inline void svgen_mf_run(SvgenMf *me, float gain, float freq) {
  me->gain = gain;
  me->freq = freq;

  // 步骤 1: 步进角度
  me->step_angle = me->freq * me->freq_max;

  // 步骤 2: 角度积分 + 扇区检测
  float old_alpha = me->alpha;
  me->alpha += me->step_angle;
  if (me->alpha >= 1.0f) {
    me->alpha -= 1.0f;
  }

  // 角度减小 → 进入下一扇区
  if (me->alpha < old_alpha) {
    me->sector++;
    if (me->sector > 5) me->sector = 0;
  }

  // 步骤 3: 正弦分量 (60° 扇区内)
  float alpha_rad = me->alpha * PI_THIRD;
  me->dy = sinf(alpha_rad);              // sin(alpha * 60deg)
  me->dx = sinf(PI_THIRD - alpha_rad);   // sin(60deg - alpha * 60deg)

  // 步骤 4: 扇区映射 → 占空比 (7 段对称, 零矢量平分)
  float t_short, t_long, t_zero;
  switch (me->sector) {
    case 0:  // 扇区1: a(*), b(+), c(-)
      t_zero = (1.0f - me->dx - me->dy) * 0.5f;
      me->ta = t_zero;
      me->tb = t_zero + me->dx;
      me->tc = 1.0f - t_zero;
      break;
    case 1:  // 扇区2: a(+), b(*), c(-)
      t_zero = (1.0f - me->dx - me->dy) * 0.5f;
      me->tb = t_zero;
      me->ta = t_zero + me->dy;
      me->tc = 1.0f - t_zero;
      break;
    case 2:  // 扇区3: a(-), b(*), c(+)
      t_zero = (1.0f - me->dx - me->dy) * 0.5f;
      me->tb = t_zero;
      me->tc = t_zero + me->dx;
      me->ta = 1.0f - t_zero;
      break;
    case 3:  // 扇区4: a(-), b(+), c(*)
      t_zero = (1.0f - me->dx - me->dy) * 0.5f;
      me->tc = t_zero;
      me->tb = t_zero + me->dy;
      me->ta = 1.0f - t_zero;
      break;
    case 4:  // 扇区5: a(+), b(-), c(*)
      t_zero = (1.0f - me->dx - me->dy) * 0.5f;
      me->tc = t_zero;
      me->ta = t_zero + me->dx;
      me->tb = 1.0f - t_zero;
      break;
    case 5:  // 扇区6: a(*), b(-), c(+)
    default:
      t_zero = (1.0f - me->dx - me->dy) * 0.5f;
      me->ta = t_zero;
      me->tc = t_zero + me->dy;
      me->tb = 1.0f - t_zero;
      break;
  }

  // 步骤 5: 缩放 [0,1] → [-1,+1] + gain
  me->ta = (2.0f * me->ta - 1.0f) * me->gain;
  me->tb = (2.0f * me->tb - 1.0f) * me->gain;
  me->tc = (2.0f * me->tc - 1.0f) * me->gain;
}

#endif  // COMP_SVGEN_H
