// 统一命令分发框架 — COM-OOP Module 层
// car_cmd_dispatch() 统一分发模式 (mod_hmi.c)
//
// 核心思想: 所有输入源 (按键/串口/CAN) 映射到同一 CarCmd 枚举,
//           通过注册的回调函数统一分发, 零代码重复。
//
// 数据流:
//   按键事件 → hmi_dispatch → CarCmd → car_cmd_dispatch() → mot_*/turn_*/follower_*
//   串口 0xFA → car_cmd_rx    → CarCmd → car_cmd_dispatch() → (同上)
//   串口 0xEF → car_cmd_ef_rx → CarCmd → car_cmd_dispatch() → (同上)
//
// 用法:
//   1. CmdDispatcher disp;
//   2. cmd_dispatch_init(&disp);
//   3. cmd_dispatch_register(&disp, CMD_MOT_FORWARD, my_forward_handler);
//   4. 按键或串口触发: cmd_dispatch_execute(&disp, cmd, payload, len);

#include "mod_cmd_dispatch.h"
#include <stddef.h>

// ======== 初始化 ========

void cmd_dispatch_init(CmdDispatcher *me) {
  for (int i = 0; i < CMD_COUNT; i++) {
    me->handlers[i] = NULL;
  }
  me->last_cmd  = CMD_NONE;
  me->cmd_count = 0;
  me->err_count = 0;
}

// ======== 注册回调 ========

void cmd_dispatch_register(CmdDispatcher *me, CarCmd cmd, cmd_handler_fn handler) {
  if (cmd > CMD_NONE && cmd < CMD_COUNT) {
    me->handlers[cmd] = handler;
  }
}

// ======== 执行分发 ========

bool cmd_dispatch_execute(CmdDispatcher *me, CarCmd cmd, const uint8_t *payload, uint16_t len) {
  // 非法命令: 计入错误
  if (cmd == CMD_NONE || cmd >= CMD_COUNT) {
    me->err_count++;
    return false;
  }

  cmd_handler_fn fn = me->handlers[cmd];
  me->last_cmd = cmd;
  me->cmd_count++;

  if (fn) {
    return fn(cmd, payload, len);
  }

  // 未注册 handler: 静默忽略, 计入错误计数
  me->err_count++;
  return false;
}

// ======== 输入源适配模板 ========
// 以下为框架模板, 用户根据实际硬件映射填充。
// mod_hmi.c 中的 hmi_dispatch() / car_cmd_rx() / car_cmd_ef_rx()。

// 按键 → CarCmd 映射 (模板)
//
// 实际映射:
//   KEY1(button=0): 单击→CMD_TASK_1   双击→CMD_TURN_LEFT_90
//   KEY2(button=1): 单击→CMD_TASK_2   双击→CMD_TURN_RIGHT_90  长按→编码器标定
//   KEY3(button=2): 单击/长按→CMD_STOP  双击→CMD_OLED_PAGE
//   KEY4(button=3): 单击→CMD_TASK_3   双击→CMD_TURN_180      长按→CMD_CAL_START
//
// event: 0=单击(HMI_KEY_EVENT_CLICK) 1=双击(HMI_KEY_EVENT_DOUBLE) 2=长按(HMI_KEY_EVENT_LONG)
CarCmd cmd_from_button(uint8_t button_id, uint8_t event) {
  // 用户在此填充实际按键映射
  (void)button_id;
  (void)event;
  return CMD_NONE;
}

// 串口 0xFA 命令字节 → CarCmd 映射 (模板)
//
// car_cmd_rx 实际映射:
//   0x01→CMD_FORWARD   0x02→CMD_BACKWARD   0x03→CMD_STOP
//   0x11→CMD_TURN_LEFT_90   0x12→CMD_TURN_RIGHT_90   0x13→CMD_TURN_180
//   0x21→CMD_TRACE_TOGGLE   0x22→CMD_TRACE_ON   0x23→CMD_TRACE_OFF
//   0x31→CMD_OLED_PAGE
//   0x41→CMD_CAL_START   0x42→CMD_CAL_STEP   0x43→CMD_CAL_CANCEL
CarCmd cmd_from_serial_byte(uint8_t cmd_byte) {
  // 用户在此填充实际命令映射
  (void)cmd_byte;
  return CMD_NONE;
}

// 串口 0xEF 命令字节 → CarCmd 映射 (模板)
//
// car_cmd_ef_rx 实际映射:
//   0x0A→CMD_TURN_180   0x0B→CMD_TURN_RIGHT_90   0x0C→CMD_TURN_LEFT_90
//   0x0F→CMD_STOP   0x18→CMD_FORWARD   0x19→CMD_BACKWARD
//   0x1A→CMD_TURN_LEFT_SLOW   0x1B→CMD_TURN_RIGHT_SLOW   0x1C→CMD_TRACE_TOGGLE
CarCmd cmd_from_direct_byte(uint8_t cmd_byte) {
  // 用户在此填充实际命令映射
  (void)cmd_byte;
  return CMD_NONE;
}
