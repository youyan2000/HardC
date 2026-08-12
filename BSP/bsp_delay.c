// BSP 延时函数 (SysTick 轮询, 频率自适应 SystemCoreClock)

#include "bsp_delay.h"
#include "bsp_stm32_hal.h"

// 微秒延时 (SysTick 轮询)
void Delay_us(uint32_t nus) {
  // SystemCoreClock 由 HAL 系统时钟初始化时更新 (CMSIS 全局变量)
  uint32_t ticks = nus * (SystemCoreClock / 1000000u);
  uint32_t reload = SysTick->LOAD;
  uint32_t told = SysTick->VAL;
  uint32_t tcnt = 0;

  while (1) {
    uint32_t tnow = SysTick->VAL;
    if (tnow != told) {
      if (tnow < told)
        tcnt += told - tnow;
      else
        tcnt += reload - tnow + told;
      told = tnow;
      if (tcnt >= ticks)
        break;
    }
  }
}

// 毫秒延时
void Delay_ms(uint32_t nms) {
  Delay_us(nms * 1000);
}
