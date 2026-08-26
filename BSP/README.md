# BSP —— 平台硬件抽象层

HardC 五层架构 L1 的基座。这一层用**不透明句柄**把 STM32 与 TI C2000 的差异抹平：上层拿到的都是一个稳定句柄 + 一套稳定 API，永远不需要知道下面是什么芯片、什么寄存器。

## 这层干什么

- **不透明句柄**：`bsp_pwm.h` / `bsp_adc.h` / `bsp_irq.h` / `bsp_gpio.h` / `bsp_uart.h` / `bsp_spi.h` / `bsp_i2c.h` / `bsp_can.h` / `bsp_flash.h` / `bsp_watchdog.h` / `bsp_delay.h`… —— 头文件只暴露不透明类型与 API，内部实现对外不可见。
- **双平台后端**：每个外设一套两实现 `bsp_*_stm32.c` ↔ `bsp_*_c2000.c`，由构建时选择；调用方代码完全一样。
- **硬件加速抽象**：`bsp_dsp.h`（sqrt/biquad，CMSIS-DSP / C2000Ware / 纯 C 三后端自动选择）、`bsp_dsp_fir.h`、`bsp_dsp_fft.h`——上层调一个 API，平台各自加速。
- **中断三档强制**：`bsp_irq_apply()` 按 FAST>SLOW>HMI 三档对每个中断写 0/1/2 并逐个读回校验，配错即停机（`bsp_irq.h`）。
- **OOP 工具**：`container_of.h` 提供向下转型。

## 边界（别把什么放这里）

- ❌ **禁止把平台类型泄出**：不透明句柄，任何组件/模块不许 include HAL / Driverlib / 平台头
- ❌ 不放业务逻辑 / 算法（那属于上层）
- 平台差异归 BSP —— 上层不感知芯片

## 双平台清单

| 外设 | STM32 后端 | C2000 后端 |
|------|-----------|-----------|
| ADC | `bsp_adc_stm32.c` | `bsp_c2000_adc.c` |
| PWM / HRTIM | `bsp_hrtim.c` | `bsp_c2000_epwm.c` |
| UART / SPI / CAN | `bsp_uart_stm32.c` / `bsp_spi_stm32.c` / `bsp_can_stm32.c` | `bsp_uart_c2000.c` / `bsp_spi_c2000.c` / `bsp_can_c2000.c` |
| GPIO | `bsp_gpio_stm32.c` | `bsp_gpio_c2000.c` |
| IRQ 三档 | `bsp_irq_stm32.c` | `bsp_irq_c2000.c` |
| Flash / Jump | `bsp_flash_stm32.c` / `bsp_jump_stm32.c` | `bsp_flash_c2000*.c` / `bsp_jump_c2000.c` |
| Watchdog | `bsp_watchdog_stm32.c` | `bsp_watchdog_c2000.c` |

---

> 层级总览见 `../README.md`；如何在工程里选平台后端见 `../README.md` 快速上手。
