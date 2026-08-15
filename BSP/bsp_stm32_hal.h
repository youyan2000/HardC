// STM32 HAL 系列选择头 —— BSP 后端的统一 HAL 入口
//
// HardC 库是系列无关的; 具体 STM32 系列由外部工程/工具链宏决定:
//   -DHARDC_STM32_F1 / F3 / F4 / G4 / H7
// (cmake/HardC.CMake 的 st 分支根据 HARDC_STM32_SERIES 自动注入).
//
// 仅 BSP 后端 (bsp_hrtim.c / bsp_delay.c / bsp_adc_stm32.c / bsp_gpio_stm32.c) +
// Devices/comm 传输层 (com_uart/com_spi/com_i2c/com_can, 保留 HAL 句柄类型) include 本文件,
// 其余层 (Components/Module/App) 保持 HAL-free.

#ifndef BSP_STM32_HAL_H
#define BSP_STM32_HAL_H

#if defined(HARDC_STM32_F1)
#include "stm32f1xx_hal.h"
#elif defined(HARDC_STM32_F3)
#include "stm32f3xx_hal.h"
#elif defined(HARDC_STM32_F4)
#include "stm32f4xx_hal.h"
#elif defined(HARDC_STM32_G4)
#include "stm32g4xx_hal.h"
#elif defined(HARDC_STM32_H7)
#include "stm32h7xx_hal.h"
#else
#error "bsp_stm32_hal.h: 需定义系列宏 HARDC_STM32_F1/F3/F4/G4/H7 (见 cmake/HardC.CMake)"
#endif

#endif  // BSP_STM32_HAL_H
