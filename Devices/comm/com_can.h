// CAN 传输类 —— CommBase 子类 (Devices/comm, 中断收发接入版)
//
// 语义: 帧订阅分发 (学 libxr STM32CAN 模型, 无 DMA):
//   - 发送: can_send 入发送队列 → kick 非阻塞 bsp_can_send (硬件 mailbox 即发即收);
//           mailbox 满时帧留在队列, 发送完成中断续发.
//   - 接收: 硬件 FIFO 中断 → bsp_can_on_rx → 查订阅表按 id 分发回调 (非轮询).
//   - can_poll 保留为兼容入口 (排空队列/兜底, 非必须).
//
// 数据面 (BspCan 句柄 + 订阅表 + 发送队列) 在子类结构体, 不进 CommBase 虚表;
// BSP 之上零 HAL (bsp_can.h 不透明句柄).

#ifndef COM_CAN_H
#define COM_CAN_H

#include "comp_comm.h"
#include "comp_io.h"
#include "comp_error_code.h"
#include "comp_ring.h"
#include "bsp_can.h"

// 订阅表最大条目数
#define CAN_SUB_MAX 8

// 发送队列容量 (帧数)
#define CAN_TXQ_MAX 8

// CAN 帧视图
typedef struct {
  uint32_t id;
  uint8_t dlc;  // 0~8
  uint8_t data[8];
} CanFrame;

// 前向声明 (can_rx_fn 引用)
typedef struct Can Can;

// 订阅回调: 收到匹配 id 的帧时调用
typedef void (*can_rx_fn)(Can *me, const CanFrame *frame, void *ctx);

typedef struct {
  BspCan *port;  // 不透明 CAN 后端 (bsp_can_bind 结果)
} CanConfig;

// CAN 类 — 帧发送队列 + 帧订阅分发
struct Can {
  CommBase base;  // 基类 (必须第一成员)
  BspCan *port;   // 不透明后端
  IoCompletion completion;
  CanFrame txq_frame[CAN_TXQ_MAX];        // 发送队列 (帧数组环形)
  uint8_t txq_head, txq_tail, txq_count;  // 帧队列头/尾/计数
  BspCanFrame tx_pending;                 // mailbox 满时的暂存帧 (发送完成中断重试)
  uint8_t tx_pending_valid;               // pending 是否有效
  struct {
    uint32_t id;
    can_rx_fn fn;
    void *ctx;
  } subs[CAN_SUB_MAX];
  uint8_t sub_count;
};

ErrorCode can_init(Can *me, const CanConfig *cfg);
ErrorCode can_set_config(Can *me, const CanConfig *cfg);
void can_deinit(Can *me);

// 发送标准帧 (11-bit id, data.len ≤ 8, 否则 ERR_SIZE) — 入队列, 非阻塞
ErrorCode can_send(Can *me, uint32_t id, CommConstData data, IoCompletion comp);

// 订阅帧 (表满 → ERR_FULL; 同 id 覆盖旧订阅)
ErrorCode can_register(Can *me, uint32_t id, can_rx_fn fn, void *ctx);

// 取消订阅 (按 id 移除)
void can_unregister(Can *me, uint32_t id);

// 兼容入口: 排空发送队列 (空闲时尝试发), 非必须 (中断驱动为主)
void can_poll(Can *me);

#endif  // COM_CAN_H
