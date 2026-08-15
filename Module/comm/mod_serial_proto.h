// 调试串口协议框架 — COM-OOP Module 层
// 串口协议设计 (5 种帧类型: 0xFA/0xFB/0xFC/0xEE/0xEF)
//
// 设计原则:
//   - 每种帧有独立帧头, 接收状态机按帧头路由
//   - ISR 只收不处理 (存到 ring buffer 式的线性缓冲), 主循环处理 + 应答
//   - 支持 Runtime 调参, 无需重新烧录
//
// 帧头与用途对照 (参考 app_main.h):
//   0xFA — 控制命令 (映射 CarCmd, 2 字节帧)
//   0xFB — PID 批量调参 (固定长度, 含 pi_check 0x40490FDA 校验)
//   0xFC — 传感器查询 + 标定参数 (变长帧)
//   0xEE — 校准流程控制 (2 字节帧, 多步状态机)
//   0xEF — 底层直控 (2 字节帧, 映射 CarCmd)
//
// 数据流:
//   ISR: byte → serial_proto_feed() → 状态机累积 → frame_ready 标志
//   主循环: serial_proto_process() → 按帧头路由到对应处理器
//
// 用法:
//   1. SerialProto proto;
//   2. serial_proto_init(&proto);
//   3. proto.send_fn = my_uart_send;  // 绑定发送接口
//   4. proto.on_pid_tune = my_pid_handler;  // 注册 0xFB 帧处理器
//   5. HAL UART RX 回调: serial_proto_feed(&proto, byte);
//   6. 主循环:
//        if (serial_proto_is_frame_ready(&proto))
//          serial_proto_process(&proto, &my_cmd_disp);

#ifndef MOD_SERIAL_PROTO_H
#define MOD_SERIAL_PROTO_H

#include "mod_cmd_dispatch.h"
#include <stdint.h>
#include <stdbool.h>

// ======== 帧头定义 ========
#define FRAME_CTRL 0xFA      // 控制命令 (映射 CarCmd, 2 字节帧)
#define FRAME_PID_TUNE 0xFB  // PID 批量调参 (固定长度, 含 π 校验)
#define FRAME_SENSOR 0xFC    // 传感器查询 + 标定参数 (变长帧)
#define FRAME_CAL 0xEE       // 校准流程控制 (多步状态机)
#define FRAME_DIRECT 0xEF    // 底层直控 (电机 PWM/GPIO, 映射 CarCmd)

// ======== PID 调参帧 (0xFB) ========
// 固定长度 33 字节 (packed), ISR 收 → 主循环校验 + 应答
// pi_check = 0x40490FDA (π 的 IEEE 754 表示), 作为帧完整性魔数
#define PID_TUNE_PI_MAGIC 0x40490FDAu

// C28x 字寻址 (char=16bit): TI CGT 不支持 __attribute__((packed)) 也不认 #pragma pack —
// 0xFB 帧在 C2000 按字布局 (无子字字段), 字节布局 (33B) 是 STM32 专属 (下方断言已按平台守卫).
#if defined(__TI_COMPILER_VERSION__)
#define SERIAL_PID_FRAME_PACKED
#else
#define SERIAL_PID_FRAME_PACKED __attribute__((packed))
#endif

typedef struct SERIAL_PID_FRAME_PACKED {
  uint8_t header;     // [0]  0xFB
  uint32_t pi_check;  // [1-4] π magic (0x40490FDA) — 帧完整性校验
  uint8_t slot;       // [5]  PID 槽位 (0-9 对应 10 组独立 PID)
  float kp;           // [6-9]  比例系数
  float ki;           // [10-13] 积分系数
  float kd;           // [14-17] 微分系数
  float kf;           // [18-21] 前馈系数 (可选, 不用则填 0)
  float out_limit;    // [22-25] 输出限幅
  float i_limit;      // [26-29] 积分限幅
  uint16_t crc16;     // [30-31] CRC-16/XMODEM (覆盖 [0..29])
  uint8_t tail;       // [32] 帧尾 0xFE
} SerialPidFrame;
// sizeof(SerialPidFrame) = 1 + 4 + 1 + 6*4 + 2 + 1 = 33 bytes packed (STM32 字节布局)

// 编译期断言: 确保结构体无 padding (仅 STM32 字节布局成立; C2000 字寻址无子字对齐)
#ifndef __cplusplus
#if !defined(__TI_COMPILER_VERSION__)
_Static_assert(sizeof(SerialPidFrame) == 33, "SerialPidFrame must be 33 bytes packed");
#endif
#endif

// ======== 传感器查询帧 (0xFC) ========
// 变长帧: [0xFC][SensorCmd][data_len][data...][crc8]
typedef enum {
  SENSOR_QUERY_ADC = 0x01,      // 查询 ADC 原始值
  SENSOR_QUERY_MPU = 0x02,      // 查询 MPU6050 六轴数据
  SENSOR_QUERY_ENC = 0x03,      // 查询编码器计数
  SENSOR_QUERY_ALL = 0x0F,      // 查询全部传感器
  SENSOR_SET_THRESHOLD = 0x10,  // 设置 ADC 阈值
  SENSOR_GET_THRESHOLD = 0x11,  // 读取 ADC 阈值
  SENSOR_SAVE_CAL = 0x20,       // 保存标定参数到 Flash
} SensorCmd;

// ======== 校准帧 (0xEE) ========
// 2 字节帧: [0xEE][CalStep], 配合校准多步状态机使用
typedef enum {
  CAL_STEP_NONE = 0x00,    // 空闲
  CAL_STEP_START = 0x01,   // 进入校准模式
  CAL_STEP_WHITE = 0x02,   // 采集白平衡
  CAL_STEP_BLACK = 0x03,   // 采集黑电平
  CAL_STEP_TRACK = 0x04,   // 采集赛道特征
  CAL_STEP_RESULT = 0x05,  // 输出校准结果
  CAL_STEP_SAVE = 0x06,    // 保存校准数据
  CAL_STEP_ABORT = 0xFF,   // 终止校准 (任意步骤)
} CalStep;

// ======== 协议接收状态机 ========
typedef enum {
  RX_IDLE,    // 等待帧头
  RX_CTRL,    // 接收 0xFA 控制帧体 (1 字节命令)
  RX_PID,     // 接收 0xFB PID 帧体 (32 字节)
  RX_SENSOR,  // 接收 0xFC 传感器帧体 (变长)
  RX_CAL,     // 接收 0xEE 校准帧体 (1 字节步骤)
  RX_DIRECT,  // 接收 0xEF 直控帧体 (1 字节命令)
} RxState;

// ======== 协议上下文 ========
typedef struct SerialProto SerialProto;

// 帧处理器回调类型
typedef void (*proto_pid_fn)(const SerialPidFrame *frame);
typedef void (*proto_sensor_fn)(SensorCmd sub_cmd, const uint8_t *data, uint16_t len);
typedef void (*proto_cal_fn)(CalStep step);
typedef void (*proto_send_fn)(const uint8_t *data, uint16_t len);

struct SerialProto {
  // 状态机
  RxState state;              // 当前状态
  uint8_t header;             // 当前帧头 (0xFA/0xFB/0xFC/0xEE/0xEF)
  uint8_t buf[64];            // 接收缓冲 (含帧头, buf[0] = header)
  uint16_t idx;               // 缓冲写入索引
  uint16_t exp_len;           // 期望帧长 (帧头解析后确定)
  volatile bool frame_ready;  // 完整帧就绪标志 (ISR 置位, 主循环清零)

  // 统计
  uint32_t frame_count;    // 成功接收帧计数
  uint32_t crc_err_count;  // CRC/校验错误计数

  // 帧处理器回调 (用户注册, 在 serial_proto_process 中调用)
  proto_pid_fn on_pid_tune;       // 0xFB PID 帧处理器 (pi_check + CRC 通过后调用)
  proto_sensor_fn on_sensor_cmd;  // 0xFC 传感器命令处理器
  proto_cal_fn on_cal_step;       // 0xEE 校准步骤处理器

  // 发送函数 (用户注册, 绑定实际 UART)
  proto_send_fn send_fn;  // 底层发送接口
};

// ======== API ========

// 初始化协议上下文: 状态机归零, 回调清零
void serial_proto_init(SerialProto *me);

// 逐字节喂入状态机 (ISR 中调用, ISR 安全)
// 返回 true = 字节已被协议消费, false = 非帧头字节被丢弃
bool serial_proto_feed(SerialProto *me, uint8_t byte);

// 检查是否有完整帧就绪 (主循环轮询)
bool serial_proto_is_frame_ready(const SerialProto *me);

// 处理就绪帧 + 发送应答 (主循环调用)
// 按帧头路由: 0xFA/0xEF → CmdDispatcher, 0xFB/0xFC/0xEE → 对应回调
void serial_proto_process(SerialProto *me, CmdDispatcher *cmd);

// 发送应答帧 (通过 me->send_fn, 需先注册 send_fn)
// 返回 true = 发送成功, false = send_fn 未注册
bool serial_proto_send_response(SerialProto *me, const uint8_t *data, uint16_t len);

#endif
