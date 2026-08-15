// CAN 传输类 —— CommBase 子类 (Devices/comm, 阶段1 重构)
//
// 语义: 帧订阅分发. 发送直接收 id + data; 接收经订阅表按 id 分发回调.
// 删掉了旧版"buf 前 4 字节编码 StdId"的 hack — ID 现在是独立参数.
//
// 数据面 (订阅表 + 句柄) 在子类结构体, 不进 CommBase 虚表;
// 中断里 HAL_CAN_IRQHandler 交给 HAL (通知钩子), MAIN 里 can_poll 收帧分发.

#ifndef COM_CAN_H
#define COM_CAN_H

#include "comp_comm.h"
#include "comp_io.h"
#include "comp_error_code.h"
#include "bsp_stm32_hal.h"

// 订阅表最大条目数
#define CAN_SUB_MAX 8

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
  CAN_HandleTypeDef *hcan;
} CanConfig;

// CAN 类 — 帧发送 + 帧订阅分发 (tag 与上方 typedef 前向声明配套)
struct Can {
  CommBase base;  // 基类 (必须第一成员)
  CAN_HandleTypeDef *hcan;
  IoCompletion completion;
  struct {
    uint32_t id;
    can_rx_fn fn;
    void *ctx;
  } subs[CAN_SUB_MAX];
  uint8_t sub_count;
};

void can_init(Can *me, const CanConfig *cfg);
void can_set_config(Can *me, const CanConfig *cfg);
void can_deinit(Can *me);

// 发送标准帧 (11-bit id, data.len ≤ 8, 否则 ERR_SIZE)
ErrorCode can_send(Can *me, uint32_t id, CommConstData data, IoCompletion comp);

// 订阅帧 (表满 → ERR_FULL; 同 id 覆盖旧订阅)
ErrorCode can_register(Can *me, uint32_t id, can_rx_fn fn, void *ctx);

// 取消订阅 (按 id 移除)
void can_unregister(Can *me, uint32_t id);

// MAIN 轮询: 读 RX FIFO0 → 查订阅表分发
void can_poll(Can *me);

#endif  // COM_CAN_H
