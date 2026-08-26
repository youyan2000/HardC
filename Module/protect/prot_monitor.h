// 慢保护监控模块 — 状态聚合 / 心跳看门狗 / 慢保护去抖 / 软关断判决 (Module 层, ctx slow)
//
// 参考: WEILAI Module_ErrChecker (去抖分级) + Module_Status (状态聚合/LED) +
//       HKUST 事件树 (TIM2 SLOW 做状态更新/错误检测), 及 HardC 已有 comp_protection.h
//       (Heartbeat/Debounce) + comp_error.h (WARNING/FAULT bitmask).
//
// 职责 (App_OnSlowTick 每周期调用 prot_monitor_tick):
//   1. 心跳监控: 读 FAST 的 Heartbeat_Tick 计数器 → Heartbeat_Check(N)
//      → 死锁 (连续 N tick 心跳不变) → 停止喂狗 → IWDG 硬件复位 (bsp_watchdog)
//   2. 慢保护去抖: 读 FAST 各 Latch 的 Latest latch (telemetry / 故障标志)
//      对每个保护输入做 Debounce_Update → 确认故障 (FAULT 分级)
//   3. 软关断判决: 任一确认 FAULT → 置 output_enabled=false + 通知 PWM emergency 封锁
//      (硬件封波由 BSP/Device 层负责即时动作; 本模块做软件级软关断/恢复判决)
//   4. 状态聚合: 错误码 bitmask (WARNING/FAULT 分级) + 系统状态机
//      (SYS_OK → SYS_FAULT 锁存; 供 HMI/日志/CAN 上报)
//
// 跨上下文交接 (五原语):
//   FAST→SLOW: Latch (Latest 锁存) 只读 — prot_monitor 只调 latch_peek/latch_is_dirty
//   FAST→MAIN: 本模块 (SLOW) 通过 SPSC 环推事件 (环缓冲由 App 提供)
//   MAIN→FAST: Command 邮箱 (软关断解锁 / 复位, 周期边界生效)
//
// 边界: SLOW 允许轻微抖动, 但确认故障即置关断; 立即性由硬件封波 (PWM emergency_stop /
//   HW FAULT 引脚) 负责. 本模块不碰寄存器/HAL, 只看 latch + 调 bsp_watchdog_feed.

#ifndef PROT_MONITOR_H
#define PROT_MONITOR_H

#include <stdint.h>
#include <stdbool.h>
#include "comp_protection.h"  // Heartbeat / Debounce
#include "comp_error.h"       // ERROR_* bitmask (WARNING/FAULT)
#include "bsp_watchdog.h"     // bsp_watchdog_feed
#include "comp_ring.h"        // SPSC 事件流 (SLOW→MAIN, 环缓冲由 App 提供)
#include "comp_latch.h"       // Latch 只读 (Latest 锁存读)

// ======== 系统状态机 ========
typedef enum {
  SYS_INIT,   // 上电自检
  SYS_OK,     // 正常 (所有保护通过)
  SYS_FAULT,  // 已确认故障 (软关断, 锁存; prot_monitor_reset 复位)
} ProtSysState;

// ======== 慢保护输入源 (App 在 init 绑定 → 每 tick 读 latch) ========
// 每个保护点: FAST 侧每次 latch_write 最新值; SLOW 侧读阈值比较 + 去抖确认.
typedef struct {
  const Latch *latch;   // 指向 FAST 侧遥测 Latch (如 g_root.buck.telemetry 的 vout)
  float thresh_hi;      // 高侧跳闸阈值 (> = 故障). 0 = 该侧禁用
  float thresh_lo;      // 低侧跳闸阈值 (< = 故障). 0 = 该侧禁用
  uint32_t debounce_n;  // 去抖确认次数
  uint8_t error_bit;    // 故障确认后置入 error_mask 的位 (ERROR_* 位掩码)
  bool is_warning;      // true = WARNING 级 (不软关断, 只上报); false = FAULT 级 (软关断)
  Debounce dbn;         // 内部去抖状态 (init 填充)
} ProtChannel;

// ======== 慢保护监控运行时实例 ========
typedef struct {
  // --- 绑定的保护通道 (数组, App init 一次性注入) ---
  ProtChannel *channels;  // 通道表 (由 App 提供静态存储)
  uint8_t ch_count;       // 通道数

  // --- 心跳 (FAST 侧心跳计数器, SLOW 侧检查) ---
  Heartbeat *heartbeat;          // 指向 g_root.heartbeat (FAST ISR 每周期 Heartbeat_Tick)
  uint32_t heartbeat_threshold;  // 心跳不变 N tick → 判死锁 (缺省 100)
  bool watchdog_fed;             // 本周期是否喂狗 (诊断)

  // --- 输出 (软关断判决) ---
  bool output_enabled;   // 软关断标志: false = 已封锁 (App 读此决定是否允许 PWM)
  bool pwmed_this_tick;  // 本 tick 是否已通知 PWM 封锁 (防重复)

  // --- 状态聚合 ---
  uint32_t error_mask;  // 错误码 bitmask (ERROR_* 位聚合, WARNING+FAULT)
  uint32_t fault_mask;  // 仅 FAULT 级位 (软关断依据)
  ProtSysState state;   // 系统状态机

  // --- 跨上下文交接 (五原语) ---
  Ring evt;             // SPSC 环 (SLOW→MAIN): 保护事件, 缓冲内嵌
  uint8_t evt_buf[16];  // 事件环缓冲 (容量 15 事件)
  uint32_t evt_overflow_cnt;
  uint32_t latch_stale_ticks;  // latch 未刷新计数 (诊断: 遥测停摆检测)
} ProtMonitor;

// ======== API ========

// 构造: 清状态 + 初始化各通道去抖 + 绑定心跳
//   channels/ch_count: 保护通道表 (可用 ProtChannel_FromArray 快速建表)
void prot_monitor_init(ProtMonitor *me, ProtChannel *channels, uint8_t ch_count, Heartbeat *heartbeat);

// 每 SLOW 周期调用 (App_OnSlowTick): 心跳 → 去抖 → 软关断 → 状态聚合 → 事件
void prot_monitor_tick(ProtMonitor *me);

// 手动清错/复位 (HMI/命令): 清 error/fault/关断, 回 SYS_OK
void prot_monitor_reset(ProtMonitor *me);

// 读聚合错误码 (供 HMI/CAN 上报)
static inline uint32_t prot_monitor_error_mask(const ProtMonitor *me) {
  return me->error_mask;
}

// 读软关断状态 (App 决定是否允许模块 RUN)
static inline bool prot_monitor_output_enabled(const ProtMonitor *me) {
  return me->output_enabled;
}

// 系统状态
static inline ProtSysState prot_monitor_state(const ProtMonitor *me) {
  return me->state;
}

// 事件环满丢弃计数 (诊断; 或由 App 在排空时用 prot_monitor_evt_pop)
static inline uint32_t prot_monitor_evt_overflow(const ProtMonitor *me) {
  return me->evt_overflow_cnt;
}

// MAIN 侧消费事件流 (SPSC 环排空); 返回 false = 无事件
static inline bool prot_monitor_evt_pop(ProtMonitor *me, uint8_t *ev) {
  return ring_pop(&me->evt, ev);
}

#endif  // PROT_MONITOR_H
