// BSP 延时函数 (SysTick 轮询, 72MHz)

#ifndef BSP_DELAY_H
#define BSP_DELAY_H

#include <stdint.h>

// 微秒延时
void Delay_us(uint32_t nus);

// 毫秒延时
void Delay_ms(uint32_t nms);

#endif
