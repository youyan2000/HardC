// 电压事件检测 — 暂降 Sag / 暂升 Swell / 中断 Interruption 状态机
//
// 来源: TI C2000Ware Digital Power SDK
//   libraries/energy-metrology_library/energy_metrology_f28p55
//   (metrology_calculations.c checkSagSwellEvents + cyclePhaseDP 逐周期 RMS)
// 翻译为 C-OOP 纯C float 版本
//
// 逐周期 RMS 判据 (周期 = 电压两个正向过零之间):
//   sag_start   = Vnom·sag_pct   (默认 0.80)   低于 → 暂降
//   sag_stop    = sag_start + hysteresis        高于 → 暂降复位 (滞回)
//   swell_start = Vnom·swell_pct (默认 1.20)   高于 → 暂升
//   swell_stop  = swell_start − hysteresis      低于 → 暂升复位 (滞回)
//   min_sag_v   (默认 100V)     低于 → 中断     (须 < sag_start)
// 滞回防止阈值边界抖振: 起效后须越过对侧停止线才复位
//
// 状态机 (每带一个 tick 助手, ONSET/CONTINUING 共用; 交叉检测先于滞回恢复):
//   NORMAL ──<min_sag_v── INT_ONSET        NORMAL ──<sag_start── SAG_ONSET
//   NORMAL ──>swell_start── SWELL_ONSET
//   *_ONSET ──> *_CONTINUING               (仅在事件继续时; 恢复/交叉立即转移)
//   SAG/INT ──≥sag_stop── NORMAL           (滞回复位; INT 恢复用 sag_stop)
//   SWELL   ──≤swell_stop── NORMAL
//   SAG ──<min_sag_v── INT_ONSET           (深化为中断)
//   SAG ──>swell_start── SWELL_ONSET       (交叉: 先于滞回检测, 捕捉跃入对侧带)
//   SWELL ──<sag_start── SAG_ONSET
//
// 计数语义: 每个事件首周期在 NORMAL 检测时计入 (active_duration=1), 之后的
// 周期数只在事件继续时递增 (恢复/交叉的边界周期不计入时长, 但计入极值).
// 深化为中断时, 过渡周期计入中断时长 (成为中断首周期), 暂降时长定格于深化前
//
// 使用: power_event_sample() ISR 逐采样 (内建零过 + 周期 RMS); 或应用自己
// 结算周期 RMS 后直接调 power_event_step(). 输入须为双极性 AC 采样, 单位与
// 阈值一致 (V 或 pu 均可)

#ifndef COMP_POWER_EVENT_H
#define COMP_POWER_EVENT_H

#include <math.h>
#include <stdint.h>

// ======================= PowerEventState (状态枚举) =======================

typedef enum {
  POWER_EVENT_NORMAL = 0,         // 正常
  POWER_EVENT_SAG_ONSET,          // 暂降检测到 (首个越限周期)
  POWER_EVENT_SAG_CONTINUING,     // 暂降持续中
  POWER_EVENT_SWELL_ONSET,        // 暂升检测到
  POWER_EVENT_SWELL_CONTINUING,   // 暂升持续中
  POWER_EVENT_INT_ONSET,          // 中断检测到 (深化或直接跌落)
  POWER_EVENT_INT_CONTINUING      // 中断持续中
} PowerEventState;

// ======================= PowerEvent (电压事件检测器) =======================

typedef struct {
  // 参数 (power_event_init 计算, 可直接覆盖)
  float v_nominal;        // 标称电压 (V, RMS 标定后)
  float sag_start;        // 暂降起始阈值 = Vnom·sag_pct
  float sag_stop;         // 暂降复位阈值 = sag_start + hysteresis
  float swell_start;      // 暂升起始阈值 = Vnom·swell_pct
  float swell_stop;       // 暂升复位阈值 = swell_start − hysteresis
  float min_sag_v;        // 中断阈值 (须 < sag_start)

  // 参数 (零过检测, 默认值可覆盖)
  float threshold;        // 零过死区 (默认 0)
  float slew_limit;       // 突变毛刺过滤 |Δv| 上限 (默认 0 = 关闭)

  // 内部 — 逐周期 RMS
  float v2_sum;           // Σv² (当前周期)
  uint32_t sample_count;  // 当前周期采样数
  uint32_t last_cycle_count;  // 上一完整周期采样数 (启动基线, 防毛刺短周期)
  float last_v;           // 上一采样值 (slew 检查)
  int prev_sign;          // 上一采样符号

  // 内部 — 状态机
  PowerEventState state;  // 当前状态
  uint32_t active_duration;  // 当前事件周期数 (进行中)
  float cur_min_v;        // 当前事件电压极值 (跟踪中)
  float cur_max_v;

  // 输出 (事件完成后 latch)
  uint32_t sag_events;    // 暂降事件计数
  uint32_t swell_events;  // 暂升事件计数
  uint32_t int_events;    // 中断事件计数
  uint32_t sag_duration;  // 最近一次暂降时长 (周期)
  uint32_t swell_duration;  // 最近一次暂升时长 (周期)
  uint32_t int_duration;  // 最近一次中断时长 (周期)
  float event_min_v;      // 最近一次事件电压最小值
  float event_max_v;      // 最近一次事件电压最大值
} PowerEvent;

// 初始化
//   v_nominal   — 标称电压 (RMS)
//   sag_pct     — 暂降起始比例 (默认 0.80)
//   swell_pct   — 暂升起始比例 (默认 1.20)
//   hysteresis_v— 滞回电压 (默认 1.0V)
//   min_sag_v   — 中断阈值, 须 < sag_start (默认 100V)
static inline void power_event_init(PowerEvent *me, float v_nominal,
                                    float sag_pct, float swell_pct,
                                    float hysteresis_v, float min_sag_v) {
  me->v_nominal = v_nominal;
  me->sag_start = v_nominal * sag_pct;
  me->sag_stop = me->sag_start + hysteresis_v;
  me->swell_start = v_nominal * swell_pct;
  me->swell_stop = me->swell_start - hysteresis_v;
  me->min_sag_v = min_sag_v;

  me->threshold = 0.0f;
  me->slew_limit = 0.0f;

  me->v2_sum = 0.0f;
  me->sample_count = 0u;
  me->last_cycle_count = 0u;
  me->last_v = 0.0f;
  me->prev_sign = 1;

  me->state = POWER_EVENT_NORMAL;
  me->active_duration = 0u;
  me->cur_min_v = 0.0f;
  me->cur_max_v = 0.0f;

  me->sag_events = 0u;
  me->swell_events = 0u;
  me->int_events = 0u;
  me->sag_duration = 0u;
  me->swell_duration = 0u;
  me->int_duration = 0u;
  me->event_min_v = 0.0f;
  me->event_max_v = 0.0f;
}

// 重置状态与计数 (保留参数)
static inline void power_event_reset(PowerEvent *me) {
  float vn = me->v_nominal;
  float sag_start = me->sag_start;
  float sag_stop = me->sag_stop;
  float swell_start = me->swell_start;
  float min_sag = me->min_sag_v;
  power_event_init(me, vn, sag_start / vn, swell_start / vn,
                   sag_stop - sag_start, min_sag);
}

// 事件起始: 计数首周期并重置极值
static inline void power_event_begin(PowerEvent *me, float cycle_rms) {
  me->active_duration = 1u;
  me->cur_min_v = cycle_rms;
  me->cur_max_v = cycle_rms;
}

// ---- 每带推进助手 (ONSET/CONTINUING 共用) ----
// 交叉检测先于滞回恢复: 跃入对侧带立即转移, 避免单周期尖峰被恢复吞掉

// 暂降带推进: 深化 <min_sag → 中断; 跃入 >swell_start → 暂升; 恢复 ≥sag_stop → 正常
static inline void power_event_sag_tick(PowerEvent *me, float rms) {
  if (rms < me->cur_min_v) me->cur_min_v = rms;
  if (rms > me->cur_max_v) me->cur_max_v = rms;

  if (rms < me->min_sag_v) {
    // 深化为中断: 暂降时长定格, 过渡周期计入中断首周期
    me->sag_duration = me->active_duration;
    me->int_events++;
    me->state = POWER_EVENT_INT_ONSET;
    me->active_duration = 1u;
  } else if (rms > me->swell_start) {
    me->sag_duration = me->active_duration;
    me->swell_events++;
    me->state = POWER_EVENT_SWELL_ONSET;
    power_event_begin(me, rms);
  } else if (rms >= me->sag_stop) {
    me->sag_duration = me->active_duration;
    me->event_min_v = me->cur_min_v;
    me->event_max_v = me->cur_max_v;
    me->state = POWER_EVENT_NORMAL;
  } else {
    me->active_duration++;
  }
}

// 中断带推进: 恢复 ≥sag_stop → 正常 (电压须回升越过暂降恢复线)
static inline void power_event_int_tick(PowerEvent *me, float rms) {
  if (rms < me->cur_min_v) me->cur_min_v = rms;
  if (rms > me->cur_max_v) me->cur_max_v = rms;

  if (rms >= me->sag_stop) {
    me->int_duration = me->active_duration;
    me->event_min_v = me->cur_min_v;
    me->event_max_v = me->cur_max_v;
    me->state = POWER_EVENT_NORMAL;
  } else {
    me->active_duration++;
  }
}

// 暂升带推进: 跃入 <sag_start → 暂降; 恢复 ≤swell_stop → 正常
static inline void power_event_swell_tick(PowerEvent *me, float rms) {
  if (rms < me->cur_min_v) me->cur_min_v = rms;
  if (rms > me->cur_max_v) me->cur_max_v = rms;

  if (rms < me->sag_start) {
    me->swell_duration = me->active_duration;
    me->sag_events++;
    me->state = POWER_EVENT_SAG_ONSET;
    power_event_begin(me, rms);
  } else if (rms <= me->swell_stop) {
    me->swell_duration = me->active_duration;
    me->event_min_v = me->cur_min_v;
    me->event_max_v = me->cur_max_v;
    me->state = POWER_EVENT_NORMAL;
  } else {
    me->active_duration++;
  }
}

// 状态机推进 — 每周期调用一次
//   cycle_rms — 本周期电压有效值 (由调用者结算, 或 power_event_sample 内部结算)
static inline void power_event_step(PowerEvent *me, float cycle_rms) {
  switch (me->state) {
    case POWER_EVENT_NORMAL:
      if (cycle_rms < me->min_sag_v) {
        me->int_events++;
        me->state = POWER_EVENT_INT_ONSET;
        power_event_begin(me, cycle_rms);
      } else if (cycle_rms < me->sag_start) {
        me->sag_events++;
        me->state = POWER_EVENT_SAG_ONSET;
        power_event_begin(me, cycle_rms);
      } else if (cycle_rms > me->swell_start) {
        me->swell_events++;
        me->state = POWER_EVENT_SWELL_ONSET;
        power_event_begin(me, cycle_rms);
      }
      break;

    case POWER_EVENT_SAG_ONSET:
      power_event_sag_tick(me, cycle_rms);
      if (me->state == POWER_EVENT_SAG_ONSET) {
        me->state = POWER_EVENT_SAG_CONTINUING;  // 事件继续
      }
      break;

    case POWER_EVENT_SAG_CONTINUING:
      power_event_sag_tick(me, cycle_rms);
      break;

    case POWER_EVENT_INT_ONSET:
      power_event_int_tick(me, cycle_rms);
      if (me->state == POWER_EVENT_INT_ONSET) {
        me->state = POWER_EVENT_INT_CONTINUING;
      }
      break;

    case POWER_EVENT_INT_CONTINUING:
      power_event_int_tick(me, cycle_rms);
      break;

    case POWER_EVENT_SWELL_ONSET:
      power_event_swell_tick(me, cycle_rms);
      if (me->state == POWER_EVENT_SWELL_ONSET) {
        me->state = POWER_EVENT_SWELL_CONTINUING;
      }
      break;

    case POWER_EVENT_SWELL_CONTINUING:
      power_event_swell_tick(me, cycle_rms);
      break;
  }
}

// 单采样累加 + 周期边界推进 — ISR 热路径, 每采样调用一次
//   正向零过 (含死区 + 可选 slew 毛刺过滤) = 一个整周期完成
static inline void power_event_sample(PowerEvent *me, float v) {
  me->v2_sum += v * v;
  me->sample_count++;

  // 突变毛刺过滤: |Δv| 超限本采样不判零过 (slew_limit = 0 关闭)
  float dv = fabsf(v - me->last_v);
  me->last_v = v;

  int sign = (v > me->threshold) ? 1 : 0;
  if (me->slew_limit > 0.0f && dv > me->slew_limit) {
    return;
  }

  if (me->prev_sign != sign && sign == 1) {
    if (me->sample_count > 0u) {
      uint32_t last = me->last_cycle_count;
      if (last == 0u) {
        // 首个周期建立基线, 不处理 (避免启动期不完整周期误判事件)
        me->last_cycle_count = me->sample_count;
      } else if (me->sample_count >= last / 2u) {
        power_event_step(me, sqrtf(me->v2_sum / (float)me->sample_count));
        me->last_cycle_count = me->sample_count;
      }
      // 短于基线一半的周期视为毛刺, 直接丢弃 (slew 之外的兜底)
    }
    me->v2_sum = 0.0f;
    me->sample_count = 0u;
  }
  me->prev_sign = sign;
}

// 是否有事件进行中
static inline int power_event_active(const PowerEvent *me) {
  return (me->state != POWER_EVENT_NORMAL) ? 1 : 0;
}

#endif  // COMP_POWER_EVENT_H
