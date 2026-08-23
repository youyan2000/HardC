// BSP CAN 硬件抽象接口 — 平台无关中断收发 (不透明句柄, BSP 之上零 HAL)
//
// 定位: 供 Devices/comm com_can 学 libxr 模型接入:
//   - 接收: 硬件 FIFO 中断 (非轮询) → 帧回调分发 (订阅表在上层 com_can).
//   - 发送: 非阻塞入硬件 mailbox (即发即收, 无 DMA; CAN 硬件 FIFO 即缓冲).
//   CAN 无 DMA 循环模型 (对标 libxr STM32CAN: HAL_CAN_AddTxMessage + RxFifo 中断).
//
// 完成中断: 应用把 CAN RX/TX IRQ 路由进 HAL_CAN_IRQHandler.
//   BSP 以 __weak 定义 HAL_CAN_RxFifo0MsgPendingCallback / TxMailbox*CompleteCallback,
//   内部转调 bsp_can_on_rx / bsp_can_on_tx_done. 应用若自定义, 需自行调对应钩子.

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>
#include <stdbool.h>

// 不透明 CAN 句柄 (平台: STM32 = CAN_HandleTypeDef*; C2000 = CANA/B 基址)
typedef void BspCan;

// CAN 帧 (平台无关)
typedef struct {
  uint32_t id;      // 标准 11-bit / 扩展 29-bit ID
  uint8_t ide;      // 0=标准帧 1=扩展帧
  uint8_t dlc;      // 数据长度 (0..8)
  uint8_t data[8];  // 数据载荷
} BspCanFrame;

// 接收回调 (FIFO 中断上下文, 非阻塞; 上层查订阅表分发)
typedef void (*BspCanRxFn)(BspCan *me, const BspCanFrame *f, void *ctx);

// 发送完成回调 (TxMailbox 完成中断, 非阻塞; 上层从发送队列续发)
typedef void (*BspCanTxDoneFn)(BspCan *me, void *ctx);

// 绑定平台 CAN 句柄 → 不透明 BspCan* (NULL=参数错误)
BspCan *bsp_can_bind(void *h);

// 解绑 (停 CAN + 释放引用)
void bsp_can_unbind(BspCan *me);

// 注册接收回调 (FIFO 收到帧时调用; 可空=不通知)
void bsp_can_set_rx_cb(BspCan *me, BspCanRxFn cb, void *ctx);

// 注册发送完成回调 (可空=不通知)
void bsp_can_set_tx_done_cb(BspCan *me, BspCanTxDoneFn cb, void *ctx);

// 发送一帧 (非阻塞, 即发即收进硬件 mailbox; 返回 false = mailbox 满/参数错)
bool bsp_can_send(BspCan *me, const BspCanFrame *f);

// 平台完成钩子: HAL CAN 接收中断里调用 (把帧交给上层回调)
void bsp_can_on_rx(BspCan *me, const BspCanFrame *f);

// 平台完成钩子: HAL CAN 发送完成中断里调用
void bsp_can_on_tx_done(BspCan *me);

#endif  // BSP_CAN_H
