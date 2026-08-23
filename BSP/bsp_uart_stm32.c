// BSP UART STM32 后端 — bsp_uart.h 的 STM32 (HAL) 实现
//
// RX: 位置式循环 DMA (对齐 comp_dma_rx DmaRxModel).
//   HAL_UARTEx_ReceiveToIdle_DMA 以循环模式启动, DMA 写满回绕;
//   remaining = __HAL_DMA_GET_COUNTER(huart->hdmarx) — 位置式推算写位置.
// TX: 单发 DMA (HAL_UART_Transmit_DMA), 完成中断续发由上层 com_uart 处理.
//
// 完成中断: 应用把 USARTx_IRQn/DMA IRQ 路由进 HAL_UART_IRQHandler / HAL_DMA_IRQHandler,
//   在 HAL_UARTEx_RxEventCallback / HAL_UART_TxCpltCallback 里调 com_uart 的
//   on_rx_event / on_tx_done (见 com_uart API).
//
// 由 cmake/HardC.CMake 的 st 分支编译; 系列由 bsp_stm32_hal.h 选择.

#include "bsp_uart.h"
#include "bsp_stm32_hal.h"

BspUart *bsp_uart_bind(void *h) {
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *) h;
  if (huart == NULL || huart->hdmarx == NULL) {
    return NULL;  // 需已配置 RX DMA (huart->hdmarx)
  }
  // TX 完成回调 (HAL_UART_TxCpltCallback 调用, 通知上层续发)
  static BspUartTxDoneFn s_tx_done_cb = NULL;
  static void *s_tx_done_ctx = NULL;

  // 不透明句柄 = UART_HandleTypeDef* 本身 (上层不接触其字段)
  return (BspUart *) huart;
}

void bsp_uart_set_tx_done_cb(BspUart *me, BspUartTxDoneFn cb, void *ctx) {
  (void) me;
  s_tx_done_cb = cb;
  s_tx_done_ctx = ctx;
}

void bsp_uart_unbind(BspUart *me) {
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *) me;
  if (huart == NULL) {
    return;
  }
  // 停 RX/TX DMA (尽力; 未启动时 HAL 返回非 OK 可忽略)
  (void) HAL_UART_DMAStop(huart);
}

// ---- RX (位置式循环 DMA) ----

void bsp_uart_dma_rx_start(BspUart *me, uint8_t *buf, uint16_t cap) {
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *) me;
  if (huart == NULL || buf == NULL || cap == 0u) {
    return;
  }
  // 循环模式: ReceiveToIdle_DMA 在 cap 字节写满后回绕继续 (位置式单缓冲语义).
  // 注意: 需 huart->Init 已使能 UART 的 DMA 请求 (CubeMX: DMA Settings).
  (void) HAL_UARTEx_ReceiveToIdle_DMA(huart, buf, cap);
}

uint16_t bsp_uart_dma_rx_remaining(BspUart *me) {
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *) me;
  if (huart == NULL || huart->hdmarx == NULL) {
    return 0u;
  }
  // DMA 剩余未搬字节数 (0..cap): 位置式 model 用 cap - remaining = 写位置.
  return (uint16_t) __HAL_DMA_GET_COUNTER(huart->hdmarx);
}

void bsp_uart_dma_rx_stop(BspUart *me) {
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *) me;
  if (huart == NULL) {
    return;
  }
  (void) HAL_UART_DMAStop(huart);
}

// ---- TX (单发 DMA) ----

bool bsp_uart_dma_tx_start(BspUart *me, const uint8_t *data, uint16_t len) {
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *) me;
  if (huart == NULL || data == NULL || len == 0u) {
    return false;
  }
  // 单发 DMA: 非阻塞, 完成走 HAL_UART_TxCpltCallback (上层续发下一段).
  return HAL_UART_Transmit_DMA(huart, (uint8_t *) data, len) == HAL_OK;
}

bool bsp_uart_dma_tx_busy(BspUart *me) {
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *) me;
  if (huart == NULL) {
    return false;
  }
  // HAL 状态: BUSY_TX 表示 DMA 在途; READY = 空闲.
  return huart->gState == HAL_UART_STATE_BUSY_TX;
}

// HAL TX 完成弱回调: 通知上层续发下一段 (com_uart 的 uart_tx_complete)
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (s_tx_done_cb != NULL) {
    s_tx_done_cb((BspUart *) huart, s_tx_done_ctx);
  }
}
