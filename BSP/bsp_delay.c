// BSP 延时函数 (SysTick 轮询, 72MHz)

#include "bsp_delay.h"
#include "stm32f1xx_hal.h"

// 微秒延时 (SysTick 轮询, 72MHz)
void Delay_us(uint32_t nus) {
  uint32_t ticks  = nus * 72;
  uint32_t reload = SysTick->LOAD;
  uint32_t told   = SysTick->VAL;
  uint32_t tcnt   = 0;

  while (1) {
    uint32_t tnow = SysTick->VAL;
    if (tnow != told) {
      if (tnow < told) tcnt += told - tnow;
      else             tcnt += reload - tnow + told;
      told = tnow;
      if (tcnt >= ticks) break;
    }
  }
}

// 毫秒延时
void Delay_ms(uint32_t nms) {
  Delay_us(nms * 1000);
}
