#ifndef COM_CAN_H
#define COM_CAN_H

// CAN 总线通信驱动 —— CommBase 的子类
// CAN 是消息帧协议（非字节流），CommOps 做最小语义适配:
//   send: 将 dat 前4字节作 StdId, 后续作 payload, 调用 HAL_CAN_AddTxMessage
//   bgn:  启动 CAN + 激活 FIFO0 消息挂起中断
//   read: 轮询 FIFO0, 有新消息时缓存到 rx_data[] 并返回 rx_dlc
//
// 扩展 API（CAN 特有，不走 ops）:
//   can_send_msg()  — 标准帧发送（显式 ID + DLC）
//   can_poll()      — 轮询 RX FIFO
//   can_set_filter() — 配置硬件 ID 过滤器

#include "comp_comm.h"
#include "stm32f1xx_hal.h"

// 子类结构体 —— base 必须是第一个成员（保证 &can.base == &can）
typedef struct {
  CommBase          base;      // 基类
  CAN_HandleTypeDef *hcan;     // HAL CAN 句柄
  uint32_t          rx_id;     // 最近收到的 CAN ID
  uint8_t           rx_dlc;    // 最近收到的 DLC (0=无新消息)
  uint8_t           rx_data[8]; // 最近收到的 payload
} Can;

void can_init(Can *me, CommName name, CAN_HandleTypeDef *hcan);
void can_deinit(Can *me);

// 发送标准 CAN 帧 (11-bit ID, 0-8 字节数据)
void can_send_msg(Can *me, uint32_t id, const uint8_t *dat, uint8_t dlc);

// 轮询 RX FIFO0: 有新消息时填充 rx_id/rx_dlc/rx_data, 返回 dlc; 无消息返回 0
uint8_t can_poll(Can *me);

// 配置硬件接收过滤器 (单 filter, 掩码模式)
// filter_id: 期望的 ID 值   mask_id: 掩码 (1=必须匹配, 0=不关心)
void can_set_filter(Can *me, uint32_t filter_id, uint32_t mask_id);

#endif
