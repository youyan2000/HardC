// BSP ADC STM32 后端 — bsp_adc.h 的 STM32 实现 (HAL)
//
// 上层 Components/Devices 只调 bsp_adc_* (不透明 void* 句柄),
// 本文件把句柄还原为 HAL 句柄并调用 HAL_ADC_*.
// 由 cmake/C-OOP.CMake 的 st 分支编译; 系列由 bsp_stm32_hal.h 选择.

#include "bsp_adc.h"
#include "bsp_stm32_hal.h"

void bsp_adc_calibrate(void *hadc, BspAdcMode mode) {
  if (!hadc)
    return;
  (void) mode;  // F334/F3 无校准; F1/G4 校准流程见 HAL 版本差异
  // TODO: 系列相关校准 (G4: HAL_ADCEx_Calibration_Start; F1: ADC_CAL 位)
}

void bsp_adc_start_dma(void *hadc, void *hdma, uint16_t *buf, int num_ch) {
  if (!hadc)
    return;
  (void) hdma;  // HAL_ADC_Start_DMA 内部已取 hdma_adc1 (经 ADC handle)
  HAL_ADC_Start_DMA((ADC_HandleTypeDef *) hadc, (uint32_t *) buf, num_ch);
}

void bsp_adc_stop_dma(void *hadc, void *hdma) {
  if (!hadc)
    return;
  (void) hdma;
  HAL_ADC_Stop_DMA((ADC_HandleTypeDef *) hadc);
}
