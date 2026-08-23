// BSP CAN STM32 后端 — bsp_can.h 的 STM32 (HAL) 实现
//
// 学 libxr STM32CAN 模型 (无 DMA):
//   - 发送: HAL_CAN_AddTxMessage 非阻塞入硬件 mailbox (即发即收).
//   - 接收: FIFO 中断 HAL_CAN_RxFifo0MsgPendingCallback → 读帧 → bsp_can_on_rx → 上层回调.
//   - 发送完成: HAL_CAN_TxMailboxCompleteCallback → bsp_can_on_tx_done → 上层续发.
//
// 由 cmake/HardC.CMake 的 st 分支编译; 系列由 bsp_stm32_hal.h 选择.

#include "bsp_can.h"
#include "bsp_stm32_hal.h"
#include <string.h>

// 每实例状态 (单 CAN 句柄; 多实例需扩展为表)
typedef struct {
  CAN_HandleTypeDef *hcan;  // HAL 句柄
  BspCanRxFn rx_cb;         // 接收回调
  void *rx_ctx;
  BspCanTxDoneFn tx_cb;  // 发送完成回调
  void *tx_ctx;
  uint8_t inited;
} BspCanStm32;

// 模块级单例 (当前支持单 CAN 实例; 多实例扩展为状态表)
static BspCanStm32 s_can = {0};

BspCan *bsp_can_bind(void *h) {
  CAN_HandleTypeDef *hcan = (CAN_HandleTypeDef *) h;
  if (hcan == NULL) {
    return NULL;
  }
  s_can.hcan = hcan;
  s_can.rx_cb = NULL;
  s_can.rx_ctx = NULL;
  s_can.tx_cb = NULL;
  s_can.tx_ctx = NULL;
  s_can.inited = 1u;
  return (BspCan *) hcan;  // 不透明 = HAL 句柄
}

void bsp_can_unbind(BspCan *me) {
  CAN_HandleTypeDef *hcan = (CAN_HandleTypeDef *) me;
  if (hcan != NULL) {
    (void) HAL_CAN_Stop(hcan);
  }
  s_can.hcan = NULL;
  s_can.rx_cb = NULL;
  s_can.rx_ctx = NULL;
  s_can.tx_cb = NULL;
  s_can.tx_ctx = NULL;
  s_can.inited = 0u;
}

void bsp_can_set_rx_cb(BspCan *me, BspCanRxFn cb, void *ctx) {
  (void) me;
  s_can.rx_cb = cb;
  s_can.rx_ctx = ctx;
}

void bsp_can_set_tx_done_cb(BspCan *me, BspCanTxDoneFn cb, void *ctx) {
  (void) me;
  s_can.tx_cb = cb;
  s_can.tx_ctx = ctx;
}

bool bsp_can_send(BspCan *me, const BspCanFrame *f) {
  CAN_HandleTypeDef *hcan = (CAN_HandleTypeDef *) me;
  if (hcan == NULL || f == NULL || f->dlc > 8u) {
    return false;
  }
  CAN_TxHeaderTypeDef tx = {0};
  tx.StdId = (f->ide == 0u) ? f->id : 0u;
  tx.ExtId = (f->ide == 1u) ? f->id : 0u;
  tx.IDE = (f->ide == 1u) ? CAN_ID_EXT : CAN_ID_STD;
  tx.RTR = CAN_RTR_DATA;
  tx.DLC = f->dlc;
  uint32_t mailbox = 0u;
  return HAL_CAN_AddTxMessage(hcan, &tx, (uint8_t *) f->data, &mailbox) == HAL_OK;
}

// 平台完成钩子: HAL CAN 接收中断调用
void bsp_can_on_rx(BspCan *me, const BspCanFrame *f) {
  (void) me;
  if (s_can.rx_cb != NULL) {
    s_can.rx_cb((BspCan *) s_can.hcan, f, s_can.rx_ctx);
  }
}

// 平台完成钩子: HAL CAN 发送完成中断调用
void bsp_can_on_tx_done(BspCan *me) {
  (void) me;
  if (s_can.tx_cb != NULL) {
    s_can.tx_cb((BspCan *) s_can.hcan, s_can.tx_ctx);
  }
}

// ---- HAL 弱回调重写 (应用把 CAN RX/TX IRQ 路由进 HAL_CAN_IRQHandler 后触发) ----

__weak void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan != s_can.hcan || !s_can.inited) {
    return;
  }
  // 读空 FIFO0, 每帧转上层回调
  CAN_RxHeaderTypeDef rx = {0};
  BspCanFrame f = {0};
  while (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, f.data) == HAL_OK) {
    f.id = (rx.IDE == CAN_ID_EXT) ? rx.ExtId : rx.StdId;
    f.ide = (rx.IDE == CAN_ID_EXT) ? 1u : 0u;
    f.dlc = (rx.DLC > 8u) ? 8u : (uint8_t) rx.DLC;
    bsp_can_on_rx((BspCan *) hcan, &f);
  }
}

__weak void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
  if (hcan == s_can.hcan && s_can.inited) {
    bsp_can_on_tx_done((BspCan *) hcan);
  }
}

__weak void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
  if (hcan == s_can.hcan && s_can.inited) {
    bsp_can_on_tx_done((BspCan *) hcan);
  }
}

__weak void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
  if (hcan == s_can.hcan && s_can.inited) {
    bsp_can_on_tx_done((BspCan *) hcan);
  }
}
