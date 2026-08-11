#ifndef COM_KEY_H
#define COM_KEY_H

// 按键通信驱动 —— CommBase 子类, 内嵌双击状态机
// 每个按键实例独立检测: 单击(1) / 双击(2) / 长按(3)
// key_tick() 每 10ms 调用, key_event() 读取并清除事件

#include "comp_comm.h"
#include "stm32f1xx_hal.h"

// 双击状态机状态
typedef enum {
  KEY_IDLE  = 0,
  KEY_PRESS = 1,
  KEY_WAIT  = 2,
} KeySt;

// 按键事件类型
typedef enum {
  KEY_EVENT_NONE   = 0,
  KEY_EVENT_CLICK  = 1,
  KEY_EVENT_DOUBLE = 2,
  KEY_EVENT_LONG   = 3,
} KeyEvent;

#define KEY_DBL_GAP 30   // 双击窗口 30 tick = 300ms

typedef struct ComKey {
  CommBase      base;
  GPIO_TypeDef *port;
  uint16_t      pin;
  uint8_t       st;       // 双击状态机当前状态
  uint8_t       cnt;      // tick 计数
  uint8_t       event;    // 输出事件: 0=无, 1=单击, 2=双击, 3=长按
  uint8_t       prev;     // 上一次 GPIO 电平
} ComKey;

void    com_key_init(ComKey *me, CommName name, GPIO_TypeDef *port, uint16_t pin);
void    key_tick(ComKey *me);     // 每 10ms 调用, 更新内部状态机
uint8_t key_event(ComKey *me);    // 读取并清除事件

#endif
