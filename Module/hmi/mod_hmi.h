#ifndef MOD_HMI_H
#define MOD_HMI_H

// 人机交互模块 — HMI (Module 层)
// 来源: LitteCar_STM32 HMI (按键+OLED菜单+命令分发)
//       + MyFinal_Work user_hmi.c (3按键+OLED菜单)
//       + Car_Control_Study_Report §8 (传感器系统: 按键+OLED+MPU6050)
//
// 核心设计:
//   - 按键事件 → CarCmd 枚举 → cmd_dispatch_execute → 各模块回调
//   - 物理输入和通信输入走同一命令分发路径, 零代码重复
//   - 按键去抖在 tick 中完成 (ISR 安全, 无阻塞)
//
// 用法:
//   1. hmi_init(&me, keys, key_count, disp, oled);
//   2. ISR 每 tick: hmi_tick(&me);
//   3. hmi_dispatch(&me) 被 cmd_dispatch_execute 调用

#include <stdint.h>
#include <stdbool.h>

// 前向声明 — 实际类型在各自模块中定义
typedef struct CmdDispatcher CmdDispatcher;
typedef struct CommBase      CommBase;

// 按键事件类型
typedef enum {
  KEY_EVENT_CLICK  = 0,  // 单击
  KEY_EVENT_DOUBLE = 1,  // 双击
  KEY_EVENT_LONG   = 2,  // 长按
} KeyEvent;

// 单按键状态 (去抖状态机)
typedef struct {
  uint8_t pin_state;       // 当前引脚电平 (0=按下, 1=释放)
  uint8_t prev_state;      // 上一拍电平
  uint16_t press_ticks;    // 持续按下 tick 数
  uint16_t release_ticks;  // 释放后 tick 数 (双击检测)
  uint8_t click_count;     // 本次按下序列的单击计数
  bool    event_pending;   // 有待处理的事件
  KeyEvent pending_event;  // 待处理的事件类型
} KeyDebounce;

// 最大按键数
#define HMI_MAX_KEYS   4
// 长按阈值 (tick 数 @ 100Hz)
#define HMI_LONG_TICKS     80   // 0.8 秒
// 双击间隔阈值 (tick 数)
#define HMI_DOUBLE_TICKS   30   // 0.3 秒

// HMI 实例结构体
typedef struct {
  CmdDispatcher *disp;            // [必须] 命令分发器

  KeyDebounce   keys[HMI_MAX_KEYS]; // 按键去抖状态
  uint8_t       key_pin_states[HMI_MAX_KEYS]; // 当前引脚电平 (由应用层刷新)
  uint8_t       key_count;        // 实际按键数

  uint8_t       oled_page;        // OLED 当前页面 (0=状态, 1=传感器, 2=PID, 3=设置)
  bool          oled_dirty;       // OLED 需要刷新标志

  uint32_t      tick;             // 运行 tick 计数 (调试用)
} Hmi;

// ======== API ========

// 初始化: 绑定命令分发器
void hmi_init(Hmi *me, CmdDispatcher *disp);

// 注册按键: 在 init 后调用, key_index=0..3, 初始电平 (1=释放)
void hmi_add_key(Hmi *me, uint8_t key_index, uint8_t initial_state);

// 每控制周期调用一次 (ISR 中)
// 内部: 扫描所有按键 → 去抖 → 产生 KeyEvent → CarCmd → dispatch
void hmi_tick(Hmi *me);

// OLED 页面控制
void hmi_oled_next_page(Hmi *me);
void hmi_oled_set_dirty(Hmi *me);

#endif
