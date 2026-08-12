#ifndef DEV_GPO_BEEP_H
#define DEV_GPO_BEEP_H

// 有源蜂鸣器驱动 —— GpoBase 子类
// GPIO 开关控制: 通电就叫, 断电就停
// 典型用途: 按键反馈音 (100ms 短鸣)

#include "comp_gpo.h"
#include "stm32f1xx_hal.h"

// 子类结构体 —— base 必须是第一个成员
typedef struct {
  GpoBase       base;        // 基类 (必须为第一个成员)
  GPIO_TypeDef *port;        // GPIO 端口
  uint16_t      pin;         // GPIO 引脚号
  bool          active_low;  // true=低电平响, false=高电平响
  bool          is_on;       // 当前开关状态
} GpoBeep;

void gpo_beep_init  (GpoBeep *me, GpoName name,
                     GPIO_TypeDef *port, uint16_t pin, bool active_low);
void gpo_beep_deinit(GpoBeep *me);

#endif
