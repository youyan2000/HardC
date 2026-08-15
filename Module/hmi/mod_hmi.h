// 人机交互模块 — HMI (Module 层)
// HMI (按键+OLED菜单+命令分发)
//       + user_hmi.c (3按键+OLED菜单)
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

#ifndef MOD_HMI_H
#define MOD_HMI_H

#include <stdint.h>
#include <stdbool.h>

// 前向声明 — 实际类型在各自模块中定义
typedef struct CmdDispatcher CmdDispatcher;
typedef struct CommBase      CommBase;

// 按键事件类型 (HMI_KEY_EVENT_* 前缀防与外部工程旧枚举重名)
typedef enum {
  HMI_KEY_EVENT_CLICK  = 0,  // 单击
  HMI_KEY_EVENT_DOUBLE = 1,  // 双击
  HMI_KEY_EVENT_LONG   = 2,  // 长按
} HmiKeyEvent;

// 单按键状态 (去抖状态机)
typedef struct {
  uint8_t pin_state;       // 当前引脚电平 (0=按下, 1=释放)
  uint8_t prev_state;      // 上一拍电平
  uint16_t press_ticks;    // 持续按下 tick 数
  uint16_t release_ticks;  // 释放后 tick 数 (双击检测)
  uint8_t click_count;     // 本次按下序列的单击计数
  bool    event_pending;   // 有待处理的事件
  HmiKeyEvent pending_event;  // 待处理的事件类型
} KeyDebounce;

// 最大按键数
#define HMI_MAX_KEYS   4
// 长按阈值 (tick 数 @ 100Hz)
#define HMI_LONG_TICKS     80   // 0.8 秒
// 双击间隔阈值 (tick 数)
#define HMI_DOUBLE_TICKS   30   // 0.3 秒

// ======== 输出端口 (路由决策点) ========
// HMI 是唯一路由决策点: 按键事件按路由策略经这些端口发出 (用 CAN 发 / UART 发 / LED 指示 / 同时多路)
// 外部独立 (CAN 帧/字节流/电平各有语义), 内部统一 (事件回调), 路由只在 HMI 决定

// 端口事件回调 — App 在 board_init 写适配函数: can→mod_can_send_fn 包装, uart→proto_send_fn 包装, led→output_on/off
// 回调经 hmi_report 在 hmi_tick 调用者上下文 (控制 ISR/CTX_FAST) 执行, 必须非阻塞:
//   只做入队/置标志 (comp_ring/comp_latch), 真实 I/O 由 CTX_MAIN (BackgroundTask) 完成
typedef void (*HmiPortFn)(void *ctx, HmiKeyEvent evt);

// 单端口绑定 (on_event=NULL = 未绑定, 路由跳过该路)
typedef struct {
  HmiPortFn on_event;  // 事件回调
  void *ctx;           // 回调上下文 (协议实例 / 设备实例)
} HmiPort;

// 输出端口表 — hmi_report 按绑定 fan-out
typedef struct {
  HmiPort can;   // CAN 协议端口 (mod_can_proto)
  HmiPort uart;  // UART 协议端口 (mod_serial_proto)
  HmiPort led;   // LED 指示端口 (peripheral OutputBase)
} HmiPorts;

// HMI 实例结构体
typedef struct {
  CmdDispatcher *disp;            // [必须] 命令分发器

  KeyDebounce   keys[HMI_MAX_KEYS]; // 按键去抖状态
  uint8_t       key_pin_states[HMI_MAX_KEYS]; // 当前引脚电平 (由应用层刷新)
  uint8_t       key_count;        // 实际按键数

  uint8_t       oled_page;        // OLED 当前页面 (0=状态, 1=传感器, 2=PID, 3=设置)
  bool          oled_dirty;       // OLED 需要刷新标志

  HmiPorts      out;              // 输出端口表 (路由决策点, hmi_report fan-out)

  uint32_t      tick;             // 运行 tick 计数 (调试用)
} Hmi;

// ======== API ========

// 初始化: 绑定命令分发器
void hmi_init(Hmi *me, CmdDispatcher *disp);

// 注册按键: 在 init 后调用, key_index=0..3, 初始电平 (1=释放)
void hmi_add_key(Hmi *me, uint8_t key_index, uint8_t initial_state);

// 每控制周期调用一次 (ISR 中)
// 内部: 扫描所有按键 → 去抖 → 产生 HmiKeyEvent → CarCmd → dispatch → hmi_report fan-out
// ISR 约束: 端口回调 (HmiPortFn) 在调用者上下文执行, 必须非阻塞; 真实 I/O 归 CTX_MAIN
void hmi_tick(Hmi *me);

// OLED 页面控制
void hmi_oled_next_page(Hmi *me);
void hmi_oled_set_dirty(Hmi *me);

// 绑定输出端口 — App 在 board_init 调用 (port=&me->out.can / &me->out.uart / &me->out.led)
void hmi_port_bind(HmiPort *port, void *ctx, HmiPortFn cb);

// 报告事件 — 唯一路由决策点: 按端口绑定 fan-out (三路各可空, 全绑=同时多路)
// 调用上下文 = hmi_tick 调用者 (控制 ISR/CTX_FAST): 回调必须非阻塞, 只入队/置标志
void hmi_report(Hmi *me, HmiKeyEvent evt);

#endif
