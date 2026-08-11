#ifndef COM_OLED_H
#define COM_OLED_H

// OLED 通信驱动 —— CommBase 子类, 薄包装 oled.h 硬件层
// send: 逐字节以数据模式写入 OLED  read: 返回 0

#include "comp_comm.h"

typedef struct {
  CommBase base;
} ComOled;

void com_oled_init(ComOled *me, CommName name);

// 格式化输出方法 —— Module 层通过 ComOled* 调用，不直接碰 BSP oled.h
void com_oled_show_string(ComOled *me, uint8_t x, uint8_t y,
                          const char *str, uint8_t len, uint8_t size);
void com_oled_show_num(ComOled *me, uint8_t x, uint8_t y,
                       uint32_t num, uint8_t len, uint8_t size);

#endif
