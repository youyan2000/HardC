// MPPT 最大功率点跟踪库 — P&O / INC-V / INC-I
//
// 来源: TI controlSUITE solar/v1.2/float (MPPT_PNO_F, MPPT_INCC_F, MPPT_INCC_I_F)
// 翻译为 C-OOP 纯C float 版本
//
// 三种算法:
//   MpptPno   — 扰动观察法 (Perturb & Observe), 最简单, 适合稳定光照
//   MpptIncV  — 增量电导法电压型 (INC-voltage), 精度高, 适合快速变化
//   MpptIncI  — 增量电导法电流型 (INC-current), 输出电流参考而非电压参考
//
// 调用方式 (ISR 或慢循环, 每 100ms~1s 调用一次):
//   mppt_pno_run(&mppt, v_pv, i_pv);
//   v_ref = mppt.v_out;  // 将 MPPT 输出送给电压环 PID 做目标值

#ifndef COMP_MPPT_H
#define COMP_MPPT_H

#include <stdint.h>

// ======================= MPPT P&O (扰动观察法, 电压型) =======================

typedef struct {
  float i_pv;             // 输入: PV 电流 (A)
  float v_pv;             // 输入: PV 电压 (V)
  float delta_p_min;      // 参数: 最小功率变化阈值 (W, 噪声死区)
  float v_max;            // 参数: 输出电压上限 (V)
  float v_min;            // 参数: 输出电压下限 (V)
  float step_size;        // 参数/状态: 扰动步长 (V, 符号表示搜索方向)
  float v_out;            // 输出: MPP 电压参考 (V)
  float delta_p;          // 内部: 功率变化 ΔP = P(k) - P(k-1)
  float power;            // 内部: 当前功率 P = V * I
  float power_prev;       // 内部: 上一拍功率
  int16_t enable;         // 控制: 1=MPPT使能
  int16_t first;          // 控制: 1=首次调用
} MpptPno;

#define MPPT_PNO_DEFAULTS { 0, 0, 0.5f, 400, 0, 0.5f, 0, 0, 0, 0, 1, 1 }

static inline void mppt_pno_init(MpptPno *me, float v_min, float v_max,
                                 float step_size, float delta_p_min) {
  me->v_min = v_min;
  me->v_max = v_max;
  me->step_size = step_size;
  me->delta_p_min = delta_p_min;
  me->v_out = 0;
  me->power = 0;
  me->power_prev = 0;
  me->delta_p = 0;
  me->enable = 1;
  me->first = 1;
}

// P&O 单步运行 — 返回 MPP 电压参考
static inline float mppt_pno_run(MpptPno *me, float v_pv, float i_pv) {
  me->i_pv = i_pv;
  me->v_pv = v_pv;

  if (!me->enable) return me->v_out;

  if (me->first) {
    // 首次: 从当前电压出发, 走一小步
    me->v_out = v_pv - 0.02f;
    me->first = 0;
    me->power_prev = v_pv * i_pv;
  } else {
    me->power = v_pv * i_pv;
    me->delta_p = me->power - me->power_prev;

    if (me->delta_p > me->delta_p_min) {
      // 功率上升 → 方向正确, 继续同方向
      me->v_out = v_pv + me->step_size;
    } else if (me->delta_p < -me->delta_p_min) {
      // 功率下降 → 方向错误, 反转
      me->step_size = -me->step_size;
      me->v_out = v_pv + me->step_size;
    }
    // |ΔP| ≤ delta_p_min: 不动 (死区, 避免噪声振荡)

    me->power_prev = me->power;
  }

  // 限幅
  if (me->v_out < me->v_min) me->v_out = me->v_min;
  if (me->v_out > me->v_max) me->v_out = me->v_max;

  return me->v_out;
}

static inline void mppt_pno_reset(MpptPno *me) {
  me->first = 1;
  me->power = 0;
  me->power_prev = 0;
}

// ======================= MPPT INC-V (增量电导法, 电压型) =======================

typedef struct {
  float i_pv;             // 输入: PV 电流
  float v_pv;             // 输入: PV 电压
  float v_max;            // 参数: 输出电压上限
  float v_min;            // 参数: 输出电压下限
  float step_size;        // 参数: 扰动步长 (V, 恒正, 方向由算法决定)
  float v_out;            // 输出: MPP 电压参考
  float cond;             // 内部: 瞬时电导 I/V
  float inc_cond;         // 内部: 增量电导 dI/dV
  float delta_v;          // 内部: 电压变化 ΔV
  float delta_i;          // 内部: 电流变化 ΔI
  float v_old;            // 内部: 上一拍电压
  float i_old;            // 内部: 上一拍电流
  float step_first;       // 参数: 首次步长 (V)
  int16_t enable;         // 控制: 1=使能
  int16_t first;          // 控制: 1=首次
} MpptIncV;

#define MPPT_INCV_DEFAULTS { 0, 0, 400, 0, 0.5f, 0, 0, 0, 0, 0, 0, 0, 0.5f, 1, 1 }

static inline void mppt_incv_init(MpptIncV *me, float v_min, float v_max,
                                  float step_size, float step_first) {
  me->v_min = v_min;
  me->v_max = v_max;
  me->step_size = step_size;
  me->step_first = step_first;
  me->v_out = 0;
  me->v_old = 0;
  me->i_old = 0;
  me->enable = 1;
  me->first = 1;
}

// INC-V 单步运行
static inline float mppt_incv_run(MpptIncV *me, float v_pv, float i_pv) {
  me->i_pv = i_pv;
  me->v_pv = v_pv;

  if (!me->enable) return me->v_out;

  if (me->first) {
    me->v_out = v_pv - me->step_first;
    me->v_old = v_pv;
    me->i_old = i_pv;
    me->first = 0;
  } else {
    me->delta_v = v_pv - me->v_old;
    me->delta_i = i_pv - me->i_old;

    if (me->delta_v == 0.0f) {
      // 电压未变 (稳态附近)
      if (me->delta_i != 0.0f) {
        if (me->delta_i > 0.0f) {
          me->v_out = v_pv + me->step_size;   // 功率上升 → 继续
        } else {
          me->v_out = v_pv - me->step_size;   // 功率下降 → 反向
        }
      }
    } else {
      // dP/dV = I + V * dI/dV
      // MPP 条件: dI/dV = -I/V (增量电导 = 负瞬时电导)
      float v_inv = 1.0f / v_pv;
      me->cond = i_pv * v_inv;             // I/V
      me->inc_cond = me->delta_i / me->delta_v;  // dI/dV

      // inc_cond != -cond  → 不在 MPP
      // inc_cond > -cond  → 在 MPP 左侧, 需升压
      // inc_cond < -cond  → 在 MPP 右侧, 需降压
      // 注意: 用乘法避免浮点比较: inc_cond + cond > 0 → 左侧
      if ((me->inc_cond + me->cond) > 0.0f) {
        me->v_out = v_pv + me->step_size;   // 左侧, 升压
      } else if ((me->inc_cond + me->cond) < 0.0f) {
        me->v_out = v_pv - me->step_size;   // 右侧, 降压
      }
      // 相等 → 已在 MPP, 不动
    }

    // 限幅
    if (me->v_out < me->v_min) me->v_out = me->v_min;
    if (me->v_out > me->v_max) me->v_out = me->v_max;

    me->v_old = v_pv;
    me->i_old = i_pv;
  }

  return me->v_out;
}

static inline void mppt_incv_reset(MpptIncV *me) {
  me->first = 1;
  me->v_old = 0;
  me->i_old = 0;
}

// ======================= MPPT INC-I (增量电导法, 电流型) =======================

// 与 INC-V 的算法对称, 但输出电流参考而非电压参考
// 关键差异: DeltaV >= 0 分支的条件和方向符号与 INC-V 相反
typedef struct {
  float i_pv;
  float v_pv;
  float i_max;            // 参数: 输出电流上限
  float i_min;            // 参数: 输出电流下限
  float step_size;        // 参数: 扰动步长 (A)
  float i_out;            // 输出: MPP 电流参考
  float cond;
  float inc_cond;
  float delta_v;
  float delta_i;
  float v_old;
  float i_old;
  float step_first;
  int16_t enable;
  int16_t first;
} MpptIncI;

#define MPPT_INCI_DEFAULTS { 0, 0, 20, 0, 0.1f, 0, 0, 0, 0, 0, 0, 0, 0.1f, 1, 1 }

static inline void mppt_inci_init(MpptIncI *me, float i_min, float i_max,
                                  float step_size, float step_first) {
  me->i_min = i_min;
  me->i_max = i_max;
  me->step_size = step_size;
  me->step_first = step_first;
  me->i_out = 0;
  me->v_old = 0;
  me->i_old = 0;
  me->enable = 1;
  me->first = 1;
}

// INC-I 单步运行 — 返回 MPP 电流参考
static inline float mppt_inci_run(MpptIncI *me, float v_pv, float i_pv) {
  me->i_pv = i_pv;
  me->v_pv = v_pv;

  if (!me->enable) return me->i_out;

  if (me->first) {
    me->i_out = i_pv + me->step_first;
    me->v_old = v_pv;
    me->i_old = i_pv;
    me->first = 0;
  } else {
    me->delta_v = v_pv - me->v_old;
    me->delta_i = i_pv - me->i_old;

    if (me->delta_v >= 0.0f) {
      if (me->delta_i != 0.0f) {
        // 电流型: 方向与电压型相反
        if (me->delta_i > 0.0f) {
          me->i_out = i_pv - me->step_size;
        } else {
          me->i_out = i_pv + me->step_size;
        }
      }
    } else {
      float v_inv = 1.0f / v_pv;
      me->cond = i_pv * v_inv;
      me->inc_cond = me->delta_i / me->delta_v;

      if ((me->inc_cond + me->cond) > 0.0f) {
        me->i_out = i_pv - me->step_size;  // 左侧, 降流 (← 与电压型反向)
      } else if ((me->inc_cond + me->cond) < 0.0f) {
        me->i_out = i_pv + me->step_size;  // 右侧, 升流
      }
    }

    if (me->i_out < me->i_min) me->i_out = me->i_min;
    if (me->i_out > me->i_max) me->i_out = me->i_max;

    me->v_old = v_pv;
    me->i_old = i_pv;
  }

  return me->i_out;
}

static inline void mppt_inci_reset(MpptIncI *me) {
  me->first = 1;
  me->v_old = 0;
  me->i_old = 0;
}

#endif  // COMP_MPPT_H
