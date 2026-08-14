// BSP 看门狗 C2000 后端 — bsp_watchdog.h 的 F28004x 实现 (driverlib, WDT WDCR/WDKEY)
//
// 来源: LibXR Watchdog 移植 (同 bsp_watchdog_stm32.c). C2000 侧直接走 driverlib —
// 与 bsp_c2000_adc.c / bsp_c2000_epwm.c 同约定: F28004x 设备头无 WdtRegs 全局,
// driverlib 用 HWREGH(WD_BASE + SYSCTL_O_WDCR) 访问寄存器 (f28004x driverlib sysctl.h).
//
// C2000 看门狗机制 (F28004x TRM SPRUI33 看门狗章 + driverlib 枚举核对):
//   WDCR: WDDIS(bit6)=1 关狗; WDCHK(5:3)=101 必填; WDPS(2:0) 后分频; WDPRECLKDIV(11:8) 预分频
//   WDKEY: 喂狗 = 写 0x55 再写 0xAA (两次之间 ≤ 64 WDCLK); 写错立即复位
// 时钟链: INTOSC1(10MHz) → WDPRECLKDIV → PREDIVCLK → WDPS → WDCLK
//   WDCNTR 8 位计满 256 溢出 → 复位脉冲;  timeout = 256 × WDPRECLKDIV × WDPS / 10MHz
//   WDPRECLKDIV 编码 (折断式, 非线序): /2../256 = 位值 8..15; /512../4096 = 位值 0..3 (复位默认 /512)
//   WDPS 编码: /1../64 = 1..7  (旧 C2000 的 0=/1 1=/64… 表是 F2833x, 不适用本系列)
// 超时范围 ≈ 51.2µs (/2×/1) ~ 6.71s (/4096×/64); 请求超限自动钳位.
// 注: F28004x WDCLK 固定由 INTOSC1 (10MHz) 驱动, 与 SYSCLK 无关 → clock_hz 忽略.
// 由 cmake/C-OOP.CMake 的 c2000 分支编译 (Phase 3 落地).

#include "bsp_watchdog.h"
#include <stdint.h>
#include "driverlib.h"

void bsp_watchdog_init(void *dev, const BspWatchdogCfg *cfg) {
  (void) dev;
  if (cfg == NULL || cfg->timeout_ms == 0u) {
    return;
  }

  // 硬件上限 6.71s (/4096×/64), 超限钳位 (同时避免下方 ×5000 溢出 uint32)
  uint32_t timeout_ms = cfg->timeout_ms;
  if (timeout_ms > 6710u) {
    timeout_ms = 6710u;
  }

  // 目标最小积: timeout_ms × 10MHz / 256000 = ×39.0625 (向上取整)
  uint32_t need = (timeout_ms * 5000u + 127u) / 128u;
  // 预分频表: 与 driverlib SysCtl_WDPredivider 枚举一一对应 (折断编码, 枚举值即 WDCR bit11:8)
  static const SysCtl_WDPredivider kPrediv[] = {
    SYSCTL_WD_PREDIV_2,    SYSCTL_WD_PREDIV_4,    SYSCTL_WD_PREDIV_8,
    SYSCTL_WD_PREDIV_16,   SYSCTL_WD_PREDIV_32,   SYSCTL_WD_PREDIV_64,
    SYSCTL_WD_PREDIV_128,  SYSCTL_WD_PREDIV_256,  SYSCTL_WD_PREDIV_512,
    SYSCTL_WD_PREDIV_1024, SYSCTL_WD_PREDIV_2048, SYSCTL_WD_PREDIV_4096
  };
  static const uint16_t kPredivDiv[] = {2u, 4u, 8u, 16u, 32u, 64u, 128u, 256u, 512u, 1024u, 2048u, 4096u};
  // 后分频表: 与 driverlib SysCtl_WDPrescaler 枚举一一对应 (/1../64 = 1..7)
  static const SysCtl_WDPrescaler kPrescale[] = {
    SYSCTL_WD_PRESCALE_1, SYSCTL_WD_PRESCALE_2, SYSCTL_WD_PRESCALE_4,
    SYSCTL_WD_PRESCALE_8, SYSCTL_WD_PRESCALE_16, SYSCTL_WD_PRESCALE_32,
    SYSCTL_WD_PRESCALE_64
  };
  static const uint16_t kPrescaleDiv[] = {1u, 2u, 4u, 8u, 16u, 32u, 64u};

  SysCtl_WDPredivider prediv = SYSCTL_WD_PREDIV_512;  // 命中时覆盖
  SysCtl_WDPrescaler presc = SYSCTL_WD_PRESCALE_1;
  uint32_t best_prod = 0u;  // 0 = 未命中 (需求超最大组合 → 钳位)
  for (int i = 0; i < (int)(sizeof(kPredivDiv) / sizeof(kPredivDiv[0])); i++) {
    for (int j = 0; j < (int)(sizeof(kPrescaleDiv) / sizeof(kPrescaleDiv[0])); j++) {
      uint32_t prod = (uint32_t) kPredivDiv[i] * kPrescaleDiv[j];
      if (prod >= need && (best_prod == 0u || prod < best_prod)) {
        best_prod = prod;
        prediv = kPrediv[i];
        presc = kPrescale[j];
      }
    }
  }
  if (best_prod == 0u) {  // 钳位后实际不可达, 防御性兜底 (硬上限组合)
    prediv = SYSCTL_WD_PREDIV_4096;
    presc = SYSCTL_WD_PRESCALE_64;
  }

  // 配置窗口: 先关狗 → 显式写两级分频 (不依赖复位默认 /512) → 使能 → 清计数器.
  // driverlib 每函数内部已 EALLOW/EDIS; 显式写保证 WDPS/WDPRECLKDIV 与 timeout 确定性映射.
  SysCtl_disableWatchdog();
  SysCtl_setWatchdogPredivider(prediv);
  SysCtl_setWatchdogPrescaler(presc);
  SysCtl_setWatchdogMode(SYSCTL_WD_MODE_RESET);  // 死锁必须复位 — 防 SysConfig 曾置中断模式导致静默失效
  SysCtl_enableWatchdog();
  SysCtl_serviceWatchdog();  // 清计数器, 从使能起重新计时 (与 bsp_watchdog_feed 同序列)
}

void bsp_watchdog_feed(void) {
  SysCtl_serviceWatchdog();
}
