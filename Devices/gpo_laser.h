#ifndef DEV_GPO_LASER_H
#define DEV_GPO_LASER_H

// 激光笔驱动 —— GpoBase 子类
// GPIO 开关控制, 安全优先: 初始化默认关闭, 绝不允许意外点亮

#include "comp_gpo.h"
#include "stm32f1xx_hal.h"

// 子类结构体 —— base 必须是第一个成员
typedef struct {
  GpoBase       base;        // 基类 (必须为第一个成员)
  GPIO_TypeDef *port;        // GPIO 端口
  uint16_t      pin;         // GPIO 引脚号
  bool          active_low;  // true=低电平亮, false=高电平亮
  bool          is_on;       // 当前开关状态
} GpoLaser;

void gpo_laser_init  (GpoLaser *me, GpoName name,
                      GPIO_TypeDef *port, uint16_t pin, bool active_low);
void gpo_laser_deinit(GpoLaser *me);

#endif
