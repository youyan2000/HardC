// BSP 看门狗 C2000 后端 — bsp_watchdog.h 的 C2000 实现 (WDT WDCR/WDKEY)
//
// C2000 看门狗机制:
//   WDKEY: 喂狗 = 写 0x55 再写 0xAA (两次之间 ≤ 64 WDCLK); 写错立即复位
//   WDCR:  WDDIS(bit6) = 1 关狗; WDPS(2:0) 预分频
// 由 cmake/C-OOP.CMake 的 c2000 分支编译 (Phase 3 落地); 设备头由用户提供.
//
// 设备头约定 (与 bsp_dsp.h Tier 3 一致): 定义 C_OOP_C2000_DEVICE_H 为提供 WdtRegs 的头
// 例: -DC_OOP_C2000_DEVICE_H="F2837xD_device.h"

#include "bsp_watchdog.h"

#if defined(__TMS320C2000__)

#if !defined(C_OOP_C2000_DEVICE_H)
#error "bsp_watchdog_c2000.c: 需定义 C_OOP_C2000_DEVICE_H (提供 WdtRegs 的 C2000 设备头)"
#endif
#include C_OOP_C2000_DEVICE_H

void bsp_watchdog_init(void *dev, const BspWatchdogCfg *cfg) {
  (void) dev;
  if (cfg == NULL || cfg->timeout_ms == 0u) {
    return;
  }

  // WDPS 预分频 (WDCLK 分频): 0=/1 1=/64 2=/512 3=/8192 4=/65536
  // 由 timeout_ms 粗选; WDCLK = LSPCLK/512 因设备/时钟配置而异, Phase 3 按数据手册核对
  uint16_t wdps;
  if (cfg->timeout_ms >= 60000u) {
    wdps = 4u;
  } else if (cfg->timeout_ms >= 8000u) {
    wdps = 3u;
  } else if (cfg->timeout_ms >= 1000u) {
    wdps = 2u;
  } else if (cfg->timeout_ms >= 100u) {
    wdps = 1u;
  } else {
    wdps = 0u;
  }

  // WDCHK(bits 5:3) 必须 = 101, 否则本次写 WDCR 自身立即触发看门狗复位 (C2000 规范)
  EALLOW;
  WdtRegs.WDCR.all = (5u << 3) | wdps;  // WDDIS=0 使能 + WDCHK=101 + WDPS
  EDIS;

  // 清计数器 (与 bsp_watchdog_feed 同序列), 从使能起重新计时
  WdtRegs.WDKEY.all = 0x0055u;
  WdtRegs.WDKEY.all = 0x00AAu;
}

void bsp_watchdog_feed(void) {
  WdtRegs.WDKEY.all = 0x0055u;
  WdtRegs.WDKEY.all = 0x00AAu;
}

#endif  // __TMS320C2000__
