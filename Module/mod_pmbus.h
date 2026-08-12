// PMBus 协议栈 — SMBus 2.0 + PMBus 1.3 命令集子集
//
// 来源: TI controlSUITE comms/PMBus
// 翻译为 C-OOP 纯C 版本
//
// 定位: Module 层, 基于 I2C 从机的数字电源通信协议栈
// 协议: SMBus 2.0 (I2C 物理层) + PMBus 1.3 (命令层)
//
// 数据格式:
//   Linear11 — 5-bit exponent (N) + 11-bit mantissa (X), Value = X · 2^N
//   Linear16 — 5-bit exponent (N) + 16-bit mantissa (X)
//   Direct   — X = (1/m) · (Y·10^(-R) - b)
//
// 典型用法:
//   1. 定义命令表 (PmbusCmdEntry 数组)
//   2. pmbus_init() 绑定命令表
//   3. I2C RX ISR → pmbus_on_rx()
//   4. I2C TX 请求 → pmbus_on_tx_byte()

#ifndef MOD_PMBUS_H
#define MOD_PMBUS_H

#include <stdint.h>
#include <stdbool.h>

// PMBus 标准命令码 (子集 — 按需扩展)
typedef enum {
  PMBUS_CMD_PAGE              = 0x00,
  PMBUS_CMD_OPERATION         = 0x01,
  PMBUS_CMD_ON_OFF_CONFIG     = 0x02,
  PMBUS_CMD_CLEAR_FAULTS      = 0x03,
  PMBUS_CMD_WRITE_PROTECT     = 0x10,
  PMBUS_CMD_VOUT_MODE         = 0x20,
  PMBUS_CMD_VOUT_COMMAND      = 0x21,
  PMBUS_CMD_VOUT_MAX          = 0x24,
  PMBUS_CMD_VOUT_TRANSITION   = 0x27,
  PMBUS_CMD_VOUT_DROOP        = 0x28,
  PMBUS_CMD_VOUT_SCALE_LOOP   = 0x29,
  PMBUS_CMD_VOUT_SCALE_MONITOR= 0x2A,
  PMBUS_CMD_IOUT_CAL_GAIN     = 0x38,
  PMBUS_CMD_FREQUENCY_SWITCH  = 0x33,
  PMBUS_CMD_POWER_MODE        = 0x3A,
  PMBUS_CMD_STATUS_BYTE       = 0x78,
  PMBUS_CMD_STATUS_WORD       = 0x79,
  PMBUS_CMD_STATUS_VOUT       = 0x7A,
  PMBUS_CMD_STATUS_IOUT       = 0x7B,
  PMBUS_CMD_STATUS_INPUT      = 0x7C,
  PMBUS_CMD_STATUS_TEMP       = 0x7D,
  PMBUS_CMD_STATUS_CML        = 0x7E,
  PMBUS_CMD_READ_VIN          = 0x88,
  PMBUS_CMD_READ_VOUT         = 0x8B,
  PMBUS_CMD_READ_IOUT         = 0x8C,
  PMBUS_CMD_READ_TEMP_1       = 0x8D,
  PMBUS_CMD_READ_TEMP_2       = 0x8E,
  PMBUS_CMD_READ_DUTY         = 0x94,
  PMBUS_CMD_READ_FREQ         = 0x95,
  PMBUS_CMD_READ_POUT         = 0x96,
  PMBUS_CMD_READ_PIN          = 0x97,
  PMBUS_CMD_MFR_ID            = 0x99,
  PMBUS_CMD_MFR_MODEL         = 0x9A,
  PMBUS_CMD_MFR_REVISION      = 0x9B,
  PMBUS_CMD_MFR_SPECIFIC_BASE = 0xD0,  // 0xD0~0xFF 制造商自定义
  PMBUS_CMD_COUNT
} PmbusCmd;

// PMBus 线性数据格式 (VOUT_MODE)
typedef enum {
  PmbusFmt_Linear11,    // 5-bit exponent + 11-bit mantissa (标准)
  PmbusFmt_Linear16,    // 5-bit exponent + 16-bit mantissa
  PmbusFmt_Direct,      // 直接格式: X = (1/m) · (Y·10^(-R) - b)
} PmbusDataFmt;

// 每个命令的处理回调
// ctx: 用户上下文 (pmbus_init 时绑定)
// data: 读缓冲 (on_read 写入) / 写数据 (on_write 读取)
// max_len: 最大数据长度
// 返回: 实际数据长度 (字节)
typedef int (*PmbusReadHandler)(void *ctx, uint8_t *data, int max_len);
typedef int (*PmbusWriteHandler)(void *ctx, const uint8_t *data, int len);

// 命令表条目
typedef struct {
  PmbusCmd           cmd;         // 命令码
  const char         *name;       // 调试用名称
  bool               writable;    // 是否可写
  int                data_len;    // 数据长度 (字节)
  PmbusReadHandler   on_read;     // 读回调 (NULL = 不支持读)
  PmbusWriteHandler  on_write;    // 写回调 (NULL = 不支持写)
} PmbusCmdEntry;

// PMBus 实例 — 栈/静态分配, 零 malloc
typedef struct {
  // 命令表 (用户注册)
  const PmbusCmdEntry *cmd_table;
  int                  cmd_count;
  void                *cmd_ctx;       // 回调上下文, 传给 on_read/on_write

  // I2C 缓冲区
  uint8_t rx_buf[64];         // 接收缓冲区 (地址 + 命令 + 数据)
  uint8_t tx_buf[64];         // 发送缓冲区
  int     rx_len;             // 实际接收长度
  int     tx_len;             // 待发送长度
  int     tx_pos;             // 发送指针

  // 状态
  bool    addressed;          // 被主机寻址
  bool    read_pending;       // 主机请求读取
  uint8_t pending_cmd;        // 当前处理的命令

  // 状态寄存器 (PMBus 标准, 由命令回调维护)
  uint8_t  status_byte;       // STATUS_BYTE (0x78)
  uint16_t status_word;       // STATUS_WORD (0x79)
} Pmbus;

// ======== API ========

// 初始化 — 绑定命令表和用户上下文
void pmbus_init(Pmbus *me, const PmbusCmdEntry *cmd_table, int cmd_count,
                void *cmd_ctx);

// I2C 接收完成回调 — 在 I2C RX 中断中调用
// 解析命令码 → 查表 → 调 on_read/on_write
void pmbus_on_rx(Pmbus *me);

// I2C 发送请求回调 — 在 I2C TX 中断/主循环中调用
// 返回下一个待发送字节, -1 = 发送完成
int  pmbus_on_tx_byte(Pmbus *me);

// PMBus 数据格式转换
// Linear11: V = X · 2^N, X=11-bit mantissa (signed), N=5-bit exponent (signed)
int16_t pmbus_linear11_encode(float value, int8_t exponent);
float   pmbus_linear11_decode(int16_t raw);

// 读取状态字 (STATUS_BYTE / STATUS_WORD) — 电源通用
uint8_t  pmbus_get_status_byte(const Pmbus *me);
uint16_t pmbus_get_status_word(const Pmbus *me);

// 设置状态位 — 更新 status_byte 和 status_word (低字节同步)
void pmbus_set_status_bit(Pmbus *me, uint8_t bit, bool active);

#endif  // MOD_PMBUS_H
