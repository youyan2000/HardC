#ifndef MOD_COMM_H
#define MOD_COMM_H

// Module 层帧协议解析器
// 变长帧协议, 参考 Rx_FSM 三帧协议模式
//
// 帧格式:
//   [帧头 0xAA 1B][命令 CMD 1B][长度 LEN 1B][数据 DATA 0~255B][校验 CHK 1B]
//   校验 = HEAD ^ CMD ^ LEN ^ DATA[0] ^ ... ^ DATA[LEN-1]  (XOR 累加)
//
// 接缝化: 发送走回调接缝 me->send (用户注册, NULL=未绑定), 不直接依赖传输类
//
// 用法:
//   1. mod_comm_init(&app);                        // 初始化状态机
//   2. app.send = my_uart_send;                    // 注册发送接口
//   3. mod_comm_on(&app, 0x01, my_handler);        // 注册命令回调
//   4. mod_comm_feed(&app, byte);                  // HAL RX 回调中逐字节喂入
//   5. mod_comm_send(&app, 0x02, data, len);       // 组帧发送

#include <stdint.h>

// 命令回调签名: cmd=命令字, dat=数据指针, len=数据长度
typedef void (*mod_cmd_fn)(uint8_t cmd, const uint8_t *dat, uint8_t len);

// 发送接口签名: dat=帧数据指针, len=帧长度
typedef void (*mod_comm_send_fn)(const uint8_t *dat, uint16_t len);

// 接收状态机状态
typedef enum {
  R_IDLE,  // 等待帧头 0xAA
  R_CMD,   // 等待命令字
  R_LEN,   // 等待数据长度
  R_DAT,   // 接收数据
  R_CHK,   // 校验帧完整性
} RxSt;

// Module 层通信实例
typedef struct {
  mod_comm_send_fn send;     // 发送接口, 用户注册, NULL=未绑定
  RxSt st;                   // 状态机当前状态
  uint8_t cmd;               // 当前帧命令字
  uint8_t len;               // 当前帧数据长度
  uint8_t idx;               // 当前数据索引
  uint8_t data[256];         // 数据缓冲
  uint8_t chk;               // 校验累加值
  mod_cmd_fn handlers[256];  // 命令回调表 (按 cmd 索引, NULL=未注册)
} ModComm;

// 初始化协议解析器: 状态机归零 → 回调表清零 → send 接缝置空
void mod_comm_init(ModComm *me);

// 逐字节喂入状态机 (在 HAL UART RX 回调中调用)
// 当一帧完整且校验通过时, 自动调用已注册的命令回调
void mod_comm_feed(ModComm *me, uint8_t byte);

// 组帧并通过 me->send 发送: [HEAD][CMD][LEN][DATA...][CHK] (需先注册 send)
void mod_comm_send(ModComm *me, uint8_t cmd, const uint8_t *dat, uint8_t len);

// 注册命令回调: 收到 cmd 时自动调用 fn
void mod_comm_on(ModComm *me, uint8_t cmd, mod_cmd_fn fn);

#endif
