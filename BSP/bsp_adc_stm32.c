// BSP ADC STM32 后端 — bsp_adc.h 的 STM32 实现 (HAL)
//
// 上层 Components/Devices 只调 bsp_adc_* (不透明 void* 句柄),
// 本文件把句柄还原为 HAL 句柄并调用 HAL_ADC_*.
// 由 cmake/HardC.CMake 的 st 分支编译; 系列由 bsp_stm32_hal.h 选择.

#include "bsp_adc.h"
#include "bsp_stm32_hal.h"

// ADC 校准 (A9 实现, 按系列分派):
//   F1:   无校准寄存器 (RM0008 §11: 硬件自动 offset 校正) — 无操作, 精度靠外部校准
//   F3/G4/H7: 支持单端+差分校准 → HAL_ADCEx_Calibration_Start(hadc, SingleDiff)
//   F4:   支持单端/差分校准 (stm32f4xx_hal_adc_ex.h)
// 前提: 必须在 ADC 使能且首次转换之前调用 (阻塞自校准); 由 board_init 执行
void bsp_adc_calibrate(void *hadc, BspAdcMode mode) {
  if (!hadc)
    return;
#if defined(HARDC_STM32_F1)
  (void) mode;  // F1 无校准流程 (硬件自动 offset 校正)
#else
  uint32_t single_diff = (mode == BSP_ADC_DIFFERENTIAL) ? ADC_DIFFERENTIAL_ENDED : ADC_SINGLE_ENDED;
  HAL_ADCEx_Calibration_Start((ADC_HandleTypeDef *) hadc, single_diff);
#endif
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

// DMA 完成 ISR 重装: 停 → 立即重启到备份块 (F3 HAL 在 DMA 完成回调内可安全 Stop+Start).
// 注意: 依赖 CubeMX 配置 MemoryDataAlignment=16bit (buf 按 uint16_t 转 uint32_t 直传 HAL).
// TODO: G4 DMA 回调内 Stop+Start 重触发行为有已知差异, 切换 G4 时需实测确认.
void bsp_adc_restart_dma(void *hadc, void *hdma, uint16_t *buf, int num_ch) {
  if (!hadc)
    return;
  (void) hdma;
  HAL_ADC_Stop_DMA((ADC_HandleTypeDef *) hadc);
  HAL_ADC_Start_DMA((ADC_HandleTypeDef *) hadc, (uint32_t *) buf, num_ch);
}
