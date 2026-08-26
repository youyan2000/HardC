// PMBus 协议栈 — SMBus 2.0 + PMBus 1.3 命令集子集实现
//
// 来源: TI controlSUITE comms/PMBus
// 翻译为 HardC 纯C 版本
//
// 协议流程:
//   I2C 写: START + ADDR(W) + CMD + DATA... + STOP
//     → pmbus_on_rx() 解析 → 调 on_write()
//   I2C 读: START + ADDR(W) + CMD + RESTART + ADDR(R) + DATA... + STOP
//     → pmbus_on_rx() 记录 pending_cmd
//     → pmbus_on_tx_byte() 调 on_read() 填充 tx_buf → 逐字节发送
//
// 数据格式:
//   Linear11: 16-bit word = [5-bit N (signed)] [11-bit X (signed)]
//     Value = X * 2^N
//     N ∈ [-16, +15], X ∈ [-1024, +1023]

#include "hmi_pmbus.h"
#include <string.h>  // memset
#include <math.h>    // powf, roundf

// ======== 初始化 ========

void pmbus_init(Pmbus *me, const PmbusCmdEntry *cmd_table, int cmd_count, void *cmd_ctx) {
  // 绑定命令表
  me->cmd_table = cmd_table;
  me->cmd_count = cmd_count;
  me->cmd_ctx = cmd_ctx;

  // 清零缓冲区
  memset(me->rx_buf, 0, sizeof(me->rx_buf));
  memset(me->tx_buf, 0, sizeof(me->tx_buf));
  me->rx_len = 0;
  me->tx_len = 0;
  me->tx_pos = 0;

  // 清零状态
  me->addressed = false;
  me->read_pending = false;
  me->pending_cmd = 0;

  // 清零状态寄存器
  me->status_byte = 0;
  me->status_word = 0;
}

// ======== 命令表查找 ========

// 在命令表中查找命令码, 返回索引, -1 = 未找到
static int pmbus_find_cmd(Pmbus *me, uint8_t cmd_code) {
  for (int i = 0; i < me->cmd_count; i++) {
    if (me->cmd_table[i].cmd == cmd_code) {
      return i;
    }
  }
  return -1;
}

// ======== I2C 接收处理 ========

void pmbus_on_rx(Pmbus *me) {
  // 至少需要地址 + 命令码 = 2 字节
  if (me->rx_len < 2) {
    return;
  }

  // rx_buf[0] = I2C 地址 (忽略, 硬件层已处理)
  // rx_buf[1] = 命令码
  uint8_t cmd_code = me->rx_buf[1];

  // 查表
  int idx = pmbus_find_cmd(me, cmd_code);
  if (idx < 0) {
    // 不支持的命令 — 置位 CML (STATUS_BYTE bit 1, per PMBus 1.3/SMBus 2.0)
    pmbus_set_status_bit(me, 1, true);
    return;
  }

  const PmbusCmdEntry *entry = &me->cmd_table[idx];

  // 判断读/写: 有后续数据 (rx_len > 2) = 写操作
  if (me->rx_len > 2) {
    // 写操作: 调 on_write
    if (entry->writable && entry->on_write) {
      int data_len = me->rx_len - 2;  // 减去地址和命令码
      if (data_len > entry->data_len) {
        data_len = entry->data_len;  // 截断到期望长度
      }
      entry->on_write(me->cmd_ctx, &me->rx_buf[2], data_len);
    }
  } else {
    // 读操作: rx_len == 2 (仅地址+命令), 设置 pending
    if (entry->on_read) {
      me->pending_cmd = cmd_code;
      me->read_pending = true;
    }
  }
}

// ======== I2C 发送处理 ========

int pmbus_on_tx_byte(Pmbus *me) {
  // 首次调用: 触发 on_read 填充 tx_buf
  if (me->read_pending) {
    int idx = pmbus_find_cmd(me, me->pending_cmd);
    if (idx >= 0 && me->cmd_table[idx].on_read) {
      const PmbusCmdEntry *entry = &me->cmd_table[idx];
      me->tx_len = entry->on_read(me->cmd_ctx, me->tx_buf, sizeof(me->tx_buf));
      if (me->tx_len <= 0) {
        me->tx_len = 0;  // 无数据返回
      }
    } else {
      me->tx_len = 0;  // 命令未注册读回调
    }
    me->tx_pos = 0;
    me->read_pending = false;
  }

  // 逐字节发送
  if (me->tx_pos < me->tx_len) {
    return (int) me->tx_buf[me->tx_pos++];
  }

  // 发送完成
  return -1;
}

// ======== Linear11 数据格式转换 ========

// Linear11 编码: float value + int8 exponent → int16 raw
// Packing: [5-bit exponent (signed)] [11-bit mantissa (signed)]
//   raw = ((N & 0x1F) << 11) | (X & 0x07FF)
//
// 算法: X = value / 2^N, 截断到 [-1024, +1023]
int16_t pmbus_linear11_encode(float value, int8_t exponent) {
  // 计算 mantissa = value / 2^exponent
  float mantissa_f = value / powf(2.0f, (float) exponent);

  // 四舍五入到整数
  int16_t X = (int16_t) roundf(mantissa_f);

  // 截断到 11-bit 有符号范围 [-1024, +1023]
  if (X > 1023)
    X = 1023;
  if (X < -1024)
    X = -1024;

  // 打包: N[4:0] 在 bits[15:11], X[10:0] 在 bits[10:0]
  uint16_t raw = ((uint16_t) (exponent & 0x1F) << 11) | ((uint16_t) X & 0x07FF);

  return (int16_t) raw;
}

// Linear11 解码: int16 raw → float value
// Unpack: N = bits[15:11] (sign-extend), X = bits[10:0] (sign-extend)
//   value = X * 2^N
float pmbus_linear11_decode(int16_t raw) {
  // 提取指数 N (5-bit 有符号 → sign-extend 到 8-bit)
  int8_t N = (int8_t) (((uint16_t) raw >> 11) & 0x1F);
  if (N & 0x10) {
    N |= 0xE0;  // 负数 sign-extend
  }

  // 提取尾数 X (11-bit 有符号 → sign-extend 到 16-bit)
  int16_t X = (int16_t) ((uint16_t) raw & 0x07FF);
  if (X & 0x0400) {
    X |= 0xF800;  // 负数 sign-extend
  }

  // 计算: value = X * 2^N
  return (float) X * powf(2.0f, (float) N);
}

// ======== 状态寄存器管理 ========

uint8_t pmbus_get_status_byte(const Pmbus *me) {
  return me->status_byte;
}

uint16_t pmbus_get_status_word(const Pmbus *me) {
  return me->status_word;
}

void pmbus_set_status_bit(Pmbus *me, uint8_t bit, bool active) {
  if (bit < 8) {
    // STATUS_BYTE (bits 0~7)
    if (active) {
      me->status_byte |= (1u << bit);
    } else {
      me->status_byte &= ~(1u << bit);
    }
    // 同步 STATUS_WORD 低字节
    if (active) {
      me->status_word |= (uint16_t) (1u << bit);
    } else {
      me->status_word &= ~(uint16_t) (1u << bit);
    }
  } else if (bit < 16) {
    // STATUS_WORD 高字节 (bits 8~15)
    if (active) {
      me->status_word |= (uint16_t) (1u << bit);
    } else {
      me->status_word &= ~(uint16_t) (1u << bit);
    }
  }
}
