// OLED (SSD1306) 外设 — 3 线软件 SPI bit-bang
//
// 迁移: 原 Devices/sensor/oled.c + oled.h (阶段2 peripheral 域收编)
// 改造:
//   - 写死的 SCL/SDA/RES 引脚 (PC14/15/13) → 3 个 BspGpioPin (App 注入)
//   - HAL_GPIO_WritePin → bsp_gpio_write (HAL-free)
//   - 静态全局显存 gram → 实例 buf (调用者提供, 零 malloc)
//   - 函数加 per_oled_ 前缀
//
// 显存布局: buf[p*128+c] — 页 p (0..7) × 列 c (0..127), 共 128*8 = 1024 字节
// 无基类: OLED 是"显示"语义, 不属 SensorBase 测量 / OutputBase 开关, 独立成类.

#ifndef PER_OLED_H
#define PER_OLED_H

#include <stdint.h>
#include "bsp_gpio.h"

// SSD1306 显存尺寸 (128x64 → 8 页 × 128 列)
#define OLED_BUF_SIZE (128 * 8)

// 写命令/数据选择
#define OLED_CMD 0   // 写命令
#define OLED_DATA 1  // 写数据

typedef struct {
  const char *name;   // 实例名 (调试/诊断用)
  uint8_t inited;     // 初始化标志
  BspGpioPin scl;     // 时钟线 (3 线 SPI 第 1 脚)
  BspGpioPin sda;     // 数据线 (3 线 SPI 第 2 脚, 兼命令/数据位)
  BspGpioPin res;     // 复位线 (3 线 SPI 第 3 脚)
  uint8_t *buf;       // 显存缓冲 (128*8 字节, 调用者提供)
  uint16_t buf_size;  // 显存缓冲容量 (≥ OLED_BUF_SIZE)
} PerOled;

// 初始化: 绑引脚 + 显存 → 复位时序 → 初始化命令序列 → 清屏
void per_oled_init(PerOled *me, const char *name, BspGpioPin scl, BspGpioPin sda, BspGpioPin res, uint8_t *buf,
                   uint16_t buf_size);

// 反初始化: 关显示 → 清标志
void per_oled_deinit(PerOled *me);

// 显存操作 (直接改 buf, 需 refresh 才上屏)
void per_oled_clear(PerOled *me);                              // 清显存 + 上屏
void per_oled_refresh(PerOled *me);                            // 整屏刷新 (buf → SSD1306 GDDRAM)
void per_oled_draw_point(PerOled *me, uint8_t x, uint8_t y);   // 画点
void per_oled_clear_point(PerOled *me, uint8_t x, uint8_t y);  // 清点

// 直接写屏 (字符/数字, 不经过显存)
void per_oled_show_char(PerOled *me, uint8_t x, uint8_t y, uint8_t chr, uint8_t size);
void per_oled_show_string(PerOled *me, uint8_t x, uint8_t y, const char *str, uint8_t len, uint8_t size);
void per_oled_show_num(PerOled *me, uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
void per_oled_show_fnum(PerOled *me, uint8_t x, uint8_t y, float num, uint8_t len, uint8_t plen, uint8_t size);

// 显示开关
void per_oled_on(PerOled *me);
void per_oled_off(PerOled *me);

#endif  // PER_OLED_H
