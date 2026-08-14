// BSP ADC 硬件抽象接口 — 模数转换校准与启动
//
// 不同平台的 ADC 校准流程不同:
//   STM32: HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED)
//   C2000: ADC_setOffsetTrim(hadc) (F28004x 仅有偏移校准, 无增益校准寄存器)
//
// 上层 Components/Devices 只调用 bsp_adc_* 函数, 不直接操作 ADC 寄存器.
// BSP 内部根据编译目标选择正确的校准实现.

#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>

// ======== 不透明句柄 ========
typedef void BspAdcHandle;

// ======== ADC 校准模式 ========
typedef enum {
  BSP_ADC_SINGLE_ENDED,  // 单端模式 (最常用)
  BSP_ADC_DIFFERENTIAL,  // 差分模式
} BspAdcMode;

// ======== BSP 接口 ========

// 启动 ADC 校准 — 阻塞执行, 在 board_init 中调用一次
// hadc: ADC HAL 句柄 (如 &hadc1), 类型为 void* 避免泄漏
void bsp_adc_calibrate(void *hadc, BspAdcMode mode);

// 启动 DMA + ADC 连续转换
// hadc:    ADC HAL 句柄
// hdma:    DMA HAL 句柄 (如 &hdma_adc1)
// buf:     DMA 循环缓冲区的首地址
// num_ch:  扫描通道数
void bsp_adc_start_dma(void *hadc, void *hdma, uint16_t *buf, int num_ch);

// 停止 DMA + ADC 转换 (反初始化)
void bsp_adc_stop_dma(void *hadc, void *hdma);

// 重启 DMA + ADC — DMA 完成 ISR 中把转换重装到新缓冲 (PingPong 备份块)
// 平台差异归 BSP:
//   STM32: HAL_ADC_Stop_DMA + HAL_ADC_Start_DMA(buf) 重装到 buf
//   C2000: ePWM 触发源重注册 (非循环 DMA, stop/start 语义不同)
// hadc/hdma: 同 start_dma; buf: 备份块首地址; num_ch: 扫描通道数
void bsp_adc_restart_dma(void *hadc, void *hdma, uint16_t *buf, int num_ch);

#endif  // BSP_ADC_H
