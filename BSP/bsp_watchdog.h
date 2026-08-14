// BSP 看门狗硬件抽象接口 — IWDG (STM32) / WDT (C2000)
//
// 来源: LibXR (bsp-dev-c/Jiu-xiao) src/driver/watchdog.hpp + driver/st/stm32_watchdog.cpp
// 翻译为 HardC 纯C BSP 层接口 (不透明句柄 + 配置 POD)
//
// 上层 Module/App 只调 bsp_watchdog_* 函数, 不直接操作看门狗寄存器.
//
// 喂狗调用点 = CTX_SLOW (App_OnSlowTick), 与 comp_protection.h Heartbeat 配合:
//   FAST: Heartbeat_Tick(&g_root.heartbeat);      // 快通道心跳
//   SLOW: Heartbeat_Check(&g_root.heartbeat, N)   // N tick 心跳不变 → 判死锁
//         → 健康才 bsp_watchdog_feed();            // 死锁/停摆 → 不喂 → 硬件复位
// 对应 LibXR Watchdog::TaskFun 语义 (每次调度喂一次).

#ifndef BSP_WATCHDOG_H
#define BSP_WATCHDOG_H

#include <stdint.h>

// 看门狗配置 (对应 LibXR Watchdog::Configuration)
typedef struct {
  uint32_t timeout_ms;  // 溢出时间 (ms) — 喂狗间隔必须 < 此值
  uint32_t feed_ms;     // 预期喂狗周期 (ms) — 供上层调度参考, 硬件只用 timeout_ms
  uint32_t clock_hz;    // 看门狗时钟 (STM32 LSI ≈ 32kHz; C2000 按设备 WDCLK)
} BspWatchdogCfg;

// 初始化 + 启动 (LibXR SetConfig + Start): 自动计算分频/重载并启动硬件
// hiwdg: 平台句柄 (STM32: IWDG_HandleTypeDef*, 在 MX_IWDG_Init 之后传入; C2000: 忽略)
void bsp_watchdog_init(void *hiwdg, const BspWatchdogCfg *cfg);

// 喂狗 (LibXR Feed) — 仅 CTX_SLOW 健康确认后调用
void bsp_watchdog_feed(void);

#endif  // BSP_WATCHDOG_H
