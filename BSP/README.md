# BSP — 本层是干什么的

> 层职责：**平台硬件抽象**。用不透明句柄把 STM32 / TI C2000 差异抹平。
> 层级模型见 [concept](docs/concept.md)。

## 本层职责
- 不透明句柄（bsp_pwm.h / bsp_adc.h / bsp_dsp.h / bsp_watchdog.h / bsp_gpio.h / bsp_delay.h）。
- 平台差异封到各后端（bsp_*_stm32.c / bsp_*_c2000.c），BSP 只暴露稳定 API。
- 硬件加速抽象：bsp_dsp.h（多后端）、bsp_dsp_fir.h、bsp_dsp_fft.h。

## 边界（别把什么放这里）
- **禁止把平台类型泄出**（不透明句柄；组件/模块不许 include HAL）。
- 不放业务逻辑 / 算法（那属于上层）。
- 平台差异归 BSP，上层不感知芯片。

## 相关
- container_of.h（OOP 下溯工具）。
- MANIFEST.yaml 自描述。
