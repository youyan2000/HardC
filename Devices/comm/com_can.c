// CAN 传输类实现 —— CommBase 子类 (Devices/comm, 中断收发接入版)
//
// 学 libxr STM32CAN: 发送队列 + 硬件 mailbox 即发即收 + FIFO 中断接收订阅分发.
//   - can_send: 入 txq 队列 → kick (空闲时取队首调 bsp_can_send)
//   - 发送完成中断: bsp_can_on_tx_done → can_tx_done → 队列续发下一帧
//   - 接收中断: bsp_can_on_rx → can_rx_dispatch → 查订阅表分发回调
//   - mailbox 满: 帧暂存 pending, 发送完成中断重试 (不乱序, 不丢帧)

#include "com_can.h"
#include "container_of.h"
#include "bsp_irq.h"  // 临界区: MAIN 侧保护发送队列
#include <string.h>

// ======== 内部: 帧发送队列 (数组环形) ========

// 帧入队 (满返回 false)
static bool can_txq_push(Can *me, const CanFrame *f) {
  if (me->txq_count >= CAN_TXQ_MAX) {
    return false;
  }
  me->txq_frame[me->txq_tail] = *f;
  me->txq_tail = (uint8_t) ((me->txq_tail + 1u) % CAN_TXQ_MAX);
  me->txq_count++;
  return true;
}

// 帧出队 (空返回 false)
static bool can_txq_pop(Can *me, CanFrame *f) {
  if (me->txq_count == 0u) {
    return false;
  }
  *f = me->txq_frame[me->txq_head];
  me->txq_head = (uint8_t) ((me->txq_head + 1u) % CAN_TXQ_MAX);
  me->txq_count--;
  return true;
}

// ======== 内部: 发送 kick ========

// 尝试发送一帧 (pending 优先, 其次队列队首); 返回 true=已入硬件
static bool can_tx_try_one(Can *me) {
  if (me->port == NULL) {
    return false;
  }
  BspCanFrame bf = {0};
  uint8_t has = 0u;

  // pending 优先 (mailbox 满时的暂存帧)
  if (me->tx_pending_valid) {
    bf.id = me->tx_pending.id;
    bf.ide = 0u;
    bf.dlc = me->tx_pending.dlc;
    memcpy(bf.data, me->tx_pending.data, me->tx_pending.dlc);
    has = 1u;
  } else {
    CanFrame f;
    if (can_txq_pop(me, &f)) {
      bf.id = f.id;
      bf.ide = 0u;
      bf.dlc = f.dlc;
      memcpy(bf.data, f.data, f.dlc);
      has = 1u;
    }
  }
  if (!has) {
    return false;  // 无帧可发
  }

  if (bsp_can_send(me->port, &bf)) {
    // 发送成功: 清 pending (若是 pending)
    me->tx_pending_valid = 0u;
    return true;
  }
  // mailbox 满: 暂存到 pending (若是从队列取出的帧)
  if (!me->tx_pending_valid) {
    me->tx_pending = bf;
    me->tx_pending_valid = 1u;
  }
  return false;  // 硬件忙, 等发送完成中断重试
}

// kick: 硬件空闲时尝试发一帧 (发送完成中断/MAIN can_poll 调用)
static void can_tx_kick(Can *me) {
  if (me->port == NULL) {
    return;
  }
  // 若硬件忙 (mailbox 满) 则等中断; 否则发一帧
  (void) can_tx_try_one(me);
}

// ======== BSP 回调适配 (中断上下文, 非阻塞) ========

// 接收中断 → 订阅表分发
static void can_rx_dispatch(BspCan *port, const BspCanFrame *f, void *ctx) {
  (void) port;
  Can *me = (Can *) ctx;
  CanFrame frame = {0};
  frame.id = f->id;
  frame.dlc = f->dlc;
  memcpy(frame.data, f->data, f->dlc);
  for (uint8_t i = 0; i < me->sub_count; i++) {
    if (me->subs[i].id == frame.id && me->subs[i].fn != NULL) {
      me->subs[i].fn(me, &frame, me->subs[i].ctx);
    }
  }
}

// 发送完成中断 → 重试 pending / 续发队列下一帧
// ISR 侧: 天然原子 (CTX_HMI 优先 2, 仅被 FAST/SLOW 抢占 — 它们不碰本队列), 无需锁
static void can_tx_done(BspCan *port, void *ctx) {
  (void) port;
  Can *me = (Can *) ctx;
  can_tx_kick(me);
}

// ======== 构造 / 析构 / 配置 ========

ErrorCode can_init(Can *me, const CanConfig *cfg) {
  if (me == NULL || cfg == NULL || cfg->port == NULL) {
    return ERR_ARG;
  }
  comm_base_init(&me->base, "can");
  me->port = cfg->port;
  me->completion = IO_ASYNC_FLAG;
  me->sub_count = 0;
  me->tx_pending_valid = 0u;
  for (int i = 0; i < CAN_SUB_MAX; i++) {
    me->subs[i].fn = NULL;
    me->subs[i].ctx = NULL;
  }
  me->txq_head = 0u;
  me->txq_tail = 0u;
  me->txq_count = 0u;
  // 注册 BSP 回调: 接收中断 → 订阅分发; 发送完成 → 续发
  bsp_can_set_rx_cb(me->port, can_rx_dispatch, me);
  bsp_can_set_tx_done_cb(me->port, can_tx_done, me);
  return ERR_OK;
}

ErrorCode can_set_config(Can *me, const CanConfig *cfg) {
  if (me == NULL || cfg == NULL || cfg->port == NULL) {
    return ERR_ARG;
  }
  me->port = cfg->port;
  bsp_can_set_rx_cb(me->port, can_rx_dispatch, me);
  bsp_can_set_tx_done_cb(me->port, can_tx_done, me);
  return ERR_OK;
}

void can_deinit(Can *me) {
  if (me == NULL) {
    return;
  }
  bsp_can_unbind(me->port);
  me->port = NULL;
  comm_base_deinit(&me->base);
}

// ======== 发送 / 订阅 ========

// 发送标准帧: 入队列 + kick (非阻塞; 队列满 → ERR_NO_BUFF)
ErrorCode can_send(Can *me, uint32_t id, CommConstData data, IoCompletion comp) {
  if (me == NULL || data.ptr == NULL) {
    return ERR_ARG;
  }
  (void) comp;  // CAN 发送天然异步 (fire-and-forget 语义)
  if (data.len > 8u) {
    return ERR_SIZE;
  }
  CanFrame f = {0};
  f.id = id;
  f.dlc = data.len;
  memcpy(f.data, data.ptr, data.len);
  // 临界区: MAIN 与 CTX_HMI(发送完成 ISR) 并发读写队列, 必须关中断保护
  bsp_irq_lock();
  bool pushed = can_txq_push(me, &f);
  if (pushed) {
    can_tx_kick(me);  // 空闲则立即发 (kick 内部读队列, 需在临界区内)
  }
  bsp_irq_unlock();
  return pushed ? ERR_OK : ERR_NO_BUFF;
}

// 订阅: 同 id 覆盖旧条目, 否则追加, 表满 → ERR_FULL
ErrorCode can_register(Can *me, uint32_t id, can_rx_fn fn, void *ctx) {
  if (me == NULL || fn == NULL) {
    return ERR_ARG;
  }
  for (int i = 0; i < me->sub_count; i++) {
    if (me->subs[i].id == id) {
      me->subs[i].fn = fn;
      me->subs[i].ctx = ctx;
      return ERR_OK;
    }
  }
  if (me->sub_count >= CAN_SUB_MAX) {
    return ERR_FULL;
  }
  me->subs[me->sub_count].id = id;
  me->subs[me->sub_count].fn = fn;
  me->subs[me->sub_count].ctx = ctx;
  me->sub_count++;
  return ERR_OK;
}

// 取消订阅: 按 id 移除, 后移填洞
void can_unregister(Can *me, uint32_t id) {
  if (me == NULL) {
    return;
  }
  for (int i = 0; i < me->sub_count; i++) {
    if (me->subs[i].id == id) {
      for (int j = i; j < me->sub_count - 1; j++) {
        me->subs[j] = me->subs[j + 1];
      }
      me->sub_count--;
      return;
    }
  }
}

// 兼容入口: 排空发送队列 (空闲时尝试发); 中断驱动为主, 此函数可选
void can_poll(Can *me) {
  if (me == NULL) {
    return;
  }
  bsp_irq_lock();
  can_tx_kick(me);
  bsp_irq_unlock();
}
