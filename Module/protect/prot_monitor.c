// 慢保护监控模块 — App_OnSlowTick 每周期驱动 (CTX_SLOW)
//
// tick 流程:
//   1. 心跳: Heartbeat_Check(heartbeat_threshold) → 死锁 → 不喂狗 → IWDG 复位兜底
//      (健康才 bsp_watchdog_feed; 死锁/停摆 → 不喂 → 硬件复位)
//   2. 慢保护去抖: 每通道读 latch → 阈值比较 → Debounce_Update → 首确认记故障
//   3. 软关断: 任一 FAULT 级确认 → output_enabled=false + 通知 PWM 封锁 (防重复)
//   4. 状态聚合: error_mask / fault_mask / SYS state
//   5. 事件: 新故障/复位 → SPSC 环推事件 (SLOW→MAIN)

#include "prot_monitor.h"
#include "comp_math.h"  // math_fabsf (阈值比较用, 若需要)
#include <string.h>

// 默认心跳死锁阈值 (tick) — 100 tick @1kHz = 100ms 无心跳即判死锁
#ifndef PROT_HEARTBEAT_DEFAULT_THRESHOLD
#define PROT_HEARTBEAT_DEFAULT_THRESHOLD 100u
#endif

// 事件码 (值直接采用 comp_error.h 位掩码, MAIN 侧可 OR 进错误寄存器)
// 高 4 bit = 保护通道故障位, 0xFE = 心跳死锁, 0xFF = 软关断确认事件
#define PROT_EVT_HEARTBEAT_STALL 0xFEu
#define PROT_EVT_SOFT_SHUTDOWN 0xFFu

void prot_monitor_init(ProtMonitor *me, ProtChannel *channels, uint8_t ch_count, Heartbeat *heartbeat) {
  memset(me, 0, sizeof(*me));
  me->channels = channels;
  me->ch_count = ch_count;
  me->heartbeat = heartbeat;
  me->heartbeat_threshold = PROT_HEARTBEAT_DEFAULT_THRESHOLD;
  me->output_enabled = true;
  me->state = SYS_INIT;
  ring_init(&me->evt, me->evt_buf, sizeof(me->evt_buf));

  for (uint8_t i = 0; i < ch_count; i++) {
    ProtChannel *ch = &channels[i];
    Debounce_Init(&ch->dbn, ch->debounce_n > 0 ? ch->debounce_n : 8u);
  }
}

// 单通道去抖 + 确认 → 记入 error/fault mask; 返回 首次确认 true
static bool prot_channel_poll(ProtMonitor *me, ProtChannel *ch) {
  if (!ch->latch) {
    return false;
  }
  float v = latch_peek(ch->latch);

  // 阈值触发 (高/低侧任一)
  bool tripped = false;
  if (ch->thresh_hi != 0.0f && v > ch->thresh_hi) {
    tripped = true;
  }
  if (ch->thresh_lo != 0.0f && v < ch->thresh_lo) {
    tripped = true;
  }

  // 去抖确认 (Debounce_Update 返回 true 仅首确认)
  if (!Debounce_Update(&ch->dbn, tripped)) {
    return false;
  }

  // 首确认: 聚合错误 / FAULT 级触发软关断
  if (ch->error_bit != 0) {
    ERROR_SET(me->error_mask, ch->error_bit);
    if (!ch->is_warning) {
      ERROR_SET(me->fault_mask, ch->error_bit);
    }
  }
  if (!ch->is_warning) {
    me->output_enabled = false;
    me->state = SYS_FAULT;
    return true;
  }
  return false;  // WARNING 级不上报软关断
}

// FAULT 级软关断 → 通知 PWM 封锁 (App 在回调里调 pwm_emergency_stop / 封波)
// 回调 = soft_shutdown 由 App 注册? 这里保持硬接线: 置标志输出, App 每 SLOW tick 轮询
typedef void (*prot_soft_shutdown_fn)(ProtMonitor *me);
// 注: 回调注册保留在 App 侧 (见 app_main.c.tmpl modules_slow_tick 锚点), 本文件不持有.

void prot_monitor_tick(ProtMonitor *me) {
  // ---- 1. 心跳监控: 死锁 → 停止喂狗 → 硬件复位兜底 ----
  me->watchdog_fed = false;
  if (me->heartbeat) {
    bool stall = Heartbeat_Check(me->heartbeat, me->heartbeat_threshold);
    if (stall) {
      // 死锁/停摆: 不喂狗 → IWDG 到期硬件复位. 软件侧同步置关断.
      me->output_enabled = false;
      me->state = SYS_FAULT;
      if (!ring_push(&me->evt, PROT_EVT_HEARTBEAT_STALL)) {
        me->evt_overflow_cnt++;
      }
      return;  // 已停机, 不再处理其它通道 (等硬件复位)
    }
    // 健康 → 喂狗
    bsp_watchdog_feed();
    me->watchdog_fed = true;
  }

  // ---- 2. 慢保护去抖 (读 FAST Latest latch) ----
  bool any_fault_confirmed = false;
  for (uint8_t i = 0; i < me->ch_count; i++) {
    if (prot_channel_poll(me, &me->channels[i])) {
      any_fault_confirmed = true;
      // 软关断确认事件 (SLOW→MAIN, 不阻塞)
      if (!ring_push(&me->evt, PROT_EVT_SOFT_SHUTDOWN)) {
        me->evt_overflow_cnt++;
      }
    }
  }

  // ---- 3. 软关断判决: FAULT 级 → 锁存关断 (App 每 SLOW tick 读 output_enabled) ----
  if (any_fault_confirmed) {
    me->output_enabled = false;
    me->state = SYS_FAULT;
  }

  // ---- 4. 状态聚合 (已由上面维护 error/fault/state) ----
  // SYS_INIT → SYS_OK: 首个慢周期完成无故障即转 OK
  if (me->state == SYS_INIT && me->fault_mask == 0U) {
    me->state = SYS_OK;
  }
}

void prot_monitor_reset(ProtMonitor *me) {
  me->error_mask = 0U;
  me->fault_mask = 0U;
  me->output_enabled = true;
  me->state = SYS_OK;
  me->pwmed_this_tick = false;
  for (uint8_t i = 0; i < me->ch_count; i++) {
    Debounce_Reset(&me->channels[i].dbn);
  }
  if (me->heartbeat) {
    Heartbeat_Init(me->heartbeat);
  }
}
