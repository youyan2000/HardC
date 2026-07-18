// SSD1306 OLED BSP 硬件层头文件 —— 3 线 SPI bit-bang 接口声明
// 源自: LitteCar 项目 (SmCar/LitteCar_STM32/c8t6_car_24H/User/BSP/oled.h)

#ifndef BSP_OLED_H
#define BSP_OLED_H

// SSD1306 OLED 硬件操作层 —— 直接操作 GPIO, 无 OOP 包装
// 3 线 SPI: SCL=PC14, SDA=PC15, RES=PC13
// 用法与 STM32 HAL 库一致, 无前缀

#include "main.h"

#define OLED_CMD  0 // 写命令
#define OLED_DATA 1 // 写数据

void oled_init(void);
void oled_write_byte(uint8_t dat, uint8_t cmd);
void oled_clear(void);
void oled_refresh(void);
void oled_draw_point(uint8_t x, uint8_t y);
void oled_clear_point(uint8_t x, uint8_t y);
void oled_show_char(uint8_t x, uint8_t y, uint8_t chr, uint8_t size);
void oled_show_string(uint8_t x, uint8_t y, const char *str, uint8_t len, uint8_t size);
void oled_show_num(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
void oled_show_fnum(uint8_t x, uint8_t y, float num, uint8_t len, uint8_t plen, uint8_t size);
void oled_on(void);
void oled_off(void);

#endif
