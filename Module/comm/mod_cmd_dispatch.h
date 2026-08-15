// 统一命令分发框架 — COM-OOP Module 层
// CarCmd 统一分发模式 (mod_hmi.h/c — car_cmd_dispatch)
//
// 核心设计:
//   物理输入 (按键/GPIO) ─→ CarCmd 枚举 ─→ cmd_dispatch_execute() ─→ 各模块回调
//   通信输入 (串口/CAN)   ─→ CarCmd 枚举 ─→ cmd_dispatch_execute() ─→ (同上)
//
// 测试时用串口, 运行时用按键, 零代码重复。
// 新增命令只需: (1) 扩展 CarCmd 枚举 (2) 注册 handler
//
// 用法:
//   1. CmdDispatcher disp;
//   2. cmd_dispatch_init(&disp);
//   3. cmd_dispatch_register(&disp, CMD_MOT_FORWARD, my_forward_handler);
//   4. 在按键扫描 / 串口解析中: cmd_dispatch_execute(&disp, cmd, payload, len);

#ifndef MOD_CMD_DISPATCH_H
#define MOD_CMD_DISPATCH_H

#include <stdint.h>
#include <stdbool.h>

// ======== 统一命令枚举 ========
// 所有物理输入和通信输入都映射到这个枚举
// 命令值用十六进制分组: 0x0x=系统, 0x1x=运动, 0x2x=校准, 0x3x=PID, 0x4x=传感器
// 用户可按需扩展, CMD_COUNT 自动适配
typedef enum {
  CMD_NONE = 0x00,

  // 系统命令 (0x0x)
  CMD_SYS_STOP = 0x01,    // 急停
  CMD_SYS_START = 0x02,   // 启动
  CMD_SYS_PAUSE = 0x03,   // 暂停
  CMD_SYS_RESET = 0x04,   // 复位
  CMD_SYS_STATUS = 0x05,  // 查询状态

  // 运动命令 (0x1x)
  CMD_MOT_FORWARD = 0x10,   // 前进
  CMD_MOT_BACKWARD = 0x11,  // 后退
  CMD_MOT_LEFT = 0x12,      // 左转
  CMD_MOT_RIGHT = 0x13,     // 右转
  CMD_MOT_BRAKE = 0x14,     // 刹车
  CMD_MOT_SPEED = 0x15,     // 速度模式 (附带速度值)

  // 校准命令 (0x2x)
  CMD_CAL_START = 0x20,   // 开始校准
  CMD_CAL_WHITE = 0x21,   // 白平衡采样
  CMD_CAL_BLACK = 0x22,   // 黑电平采样
  CMD_CAL_RESULT = 0x23,  // 校准结果确认
  CMD_CAL_ABORT = 0x24,   // 取消校准

  // PID 命令 (0x3x)
  CMD_PID_SET = 0x30,    // 设置 PID 参数
  CMD_PID_GET = 0x31,    // 读取 PID 参数
  CMD_PID_SAVE = 0x32,   // 保存 PID 到 Flash
  CMD_PID_RESET = 0x33,  // 恢复默认 PID

  // 传感器命令 (0x4x)
  CMD_SENSOR_ADC = 0x40,  // 读取 ADC
  CMD_SENSOR_MPU = 0x41,  // 读取 MPU6050
  CMD_SENSOR_ENC = 0x42,  // 读取编码器
  CMD_SENSOR_ALL = 0x4F,  // 读取所有传感器

  CMD_COUNT
} CarCmd;

// ======== 命令回调 ========
// 每个命令注册一个回调, 返回 true = 已处理
// payload: 命令附加数据 (如速度值、PID 参数), len: 数据长度, 无数据时为 NULL/0
typedef bool (*cmd_handler_fn)(CarCmd cmd, const uint8_t *payload, uint16_t len);

// ======== 命令分发器 ========
typedef struct CmdDispatcher {
  cmd_handler_fn handlers[CMD_COUNT];  // 每个命令一个处理器 (NULL = 未注册)
  CarCmd last_cmd;                     // 最后执行的命令
  uint32_t cmd_count;                  // 成功执行的命令计数
  uint32_t err_count;                  // 未注册/非法命令计数
} CmdDispatcher;

// ======== API ========

// 初始化分发器: 清零 handlers + 计数器
void cmd_dispatch_init(CmdDispatcher *me);

// 注册命令回调: cmd → handler, 重复注册会覆盖
void cmd_dispatch_register(CmdDispatcher *me, CarCmd cmd, cmd_handler_fn handler);

// 执行命令: 查找已注册的 handler 并回调, 返回 true = 已找到并执行
// payload/len 可为 NULL/0 (无附加数据的命令)
bool cmd_dispatch_execute(CmdDispatcher *me, CarCmd cmd, const uint8_t *payload, uint16_t len);

// ======== 多输入源适配 (模板函数, 用户按实际硬件填充) ========

// GPIO 按键事件 → CarCmd (hmi_dispatch 按键映射)
// button_id: 按键编号 (0-based), event: 0=单击 1=双击 2=长按
// 返回 CMD_NONE 表示该按键+事件组合无对应命令
CarCmd cmd_from_button(uint8_t button_id, uint8_t event);

// 串口 0xFA 帧命令字节 → CarCmd (car_cmd_rx 0x01=前进 等)
// cmd_byte: 0xFA 帧头后的 1 字节命令码
CarCmd cmd_from_serial_byte(uint8_t cmd_byte);

// 串口 0xEF 帧命令字节 → CarCmd (car_cmd_ef_rx 0x0A=掉头 等)
// 与 0xFA 协议共用 CarCmd 枚举, 但命令字节映射不同
CarCmd cmd_from_direct_byte(uint8_t cmd_byte);

#endif
