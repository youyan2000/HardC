// BSP 看门狗 STM32 后端 — bsp_watchdog.h 的 STM32 实现 (HAL IWDG)
//
// 移植自: LibXR driver/st/stm32_watchdog.cpp — 分频/重载自动计算表 + HAL_IWDG_Refresh
// 由 cmake/HardC.CMake 的 st 分支编译; 系列由 bsp_stm32_hal.h 选择.
// 仅当 HAL_IWDG_MODULE_ENABLED 时编译 (LibXR 同款门控, 未启用 IWDG 的工程不链接).

#include "bsp_watchdog.h"
#include "bsp_stm32_hal.h"
#include <stdbool.h>

#if defined(HAL_IWDG_MODULE_ENABLED)

static IWDG_HandleTypeDef *s_hiwdg = NULL;

void bsp_watchdog_init(void *hiwdg, const BspWatchdogCfg *cfg) {
  s_hiwdg = (IWDG_HandleTypeDef *) hiwdg;
  if (s_hiwdg == NULL || cfg == NULL || cfg->timeout_ms == 0u || cfg->clock_hz == 0u) {
    return;
  }

  // LibXR stm32_watchdog.cpp 同款: 分频 PR 0..6 → 除 4..256, 选最小的能满足 timeout 的档
  static const struct {
    uint8_t pr;
    uint16_t div;
  } IWDG_TABLE[] = {{0, 4}, {1, 8}, {2, 16}, {3, 32}, {4, 64}, {5, 128}, {6, 256}};

  uint32_t lsi = cfg->clock_hz;
  uint8_t best_pr = 6u;
  uint16_t best_rlr = 0xFFFu;
  bool found = false;

  for (uint32_t i = 0u; i < sizeof(IWDG_TABLE) / sizeof(IWDG_TABLE[0]); i++) {
    uint32_t prescaler = IWDG_TABLE[i].div;
    uint32_t reload = (cfg->timeout_ms * lsi) / (1000u * prescaler);
    if (reload == 0u) {
      reload = 1u;
    }
    if (reload > 1u) {
      reload--;
    }
    if (reload <= 0xFFFu) {
      best_pr = IWDG_TABLE[i].pr;
      best_rlr = (uint16_t) reload;
      found = true;
      break;
    }
  }
  if (!found) {
    return;  // timeout 超出 IWDG 最大溢出 (~32s @ LSI 32kHz), 保持调用前状态
  }

  s_hiwdg->Init.Prescaler = best_pr;
  s_hiwdg->Init.Reload = best_rlr;
#if defined(IWDG)
  s_hiwdg->Instance = IWDG;
#elif defined(IWDG1)
  s_hiwdg->Instance = IWDG1;
#endif
  HAL_IWDG_Init(s_hiwdg);
}

void bsp_watchdog_feed(void) {
  if (s_hiwdg != NULL) {
    HAL_IWDG_Refresh(s_hiwdg);
  }
}

#endif  // HAL_IWDG_MODULE_ENABLED
