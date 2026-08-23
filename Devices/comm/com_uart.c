// UART 字节流传输类实现 — DMA model 接入版 (Devices/comm)
//
// 数据流:
//   RX: 后端循环 DMA 写 rxm 缓冲 → uart_rx_event (RxEventCallback/SCIRXINT 调)
//       用 bsp_uart_dma_rx_remaining 喂 dma_rx_model_update → 推 rxq → MAIN uart_read
//   TX: MAIN uart_write 入 txq → dma_tx_model_submit (空闲则取队首装 active 并启动 DMA)
//       → 完成中断 uart_tx_complete → dma_tx_model_on_done → 续发下一段
// 平台: 全走 BSP/bsp_uart.h, BSP 之上零 HAL.

#include "com_uart.h"
#include "container_of.h"

// BSP 回调适配前向声明 (定义在文件后部)
static void uart_tx_done_cb(BspUart *port, void *ctx);
static void uart_rx_event_cb(BspUart *port, void *ctx);
#include <string.h>

// ======== 构造 / 析构 / 配置 ========

ErrorCode uart_init(Uart *me, const UartConfig *cfg) {
  if (me == NULL || cfg == NULL || cfg->port == NULL) {
    return ERR_ARG;
  }
  if (cfg->rx_dma_buf == NULL || cfg->rxq_buf == NULL || cfg->tx_storage == NULL || cfg->txq_buf == NULL) {
    return ERR_ARG;  // 缓冲未提供 (零 malloc 契约)
  }
  comm_base_init(&me->base, "uart");
  me->port = cfg->port;
  me->completion = IO_ASYNC_FLAG;

  // RX: 装配位置式循环 DMA model + 软件队列
  dma_rx_model_init(&me->rxm, cfg->rx_dma_buf, cfg->rx_dma_cap);
  ring_init(&me->rxq, cfg->rxq_buf, cfg->rxq_size);

  // TX: 装配双缓冲+队列 model (backend = bsp_uart_dma_tx_start)
  ring_init(&me->txq, cfg->txq_buf, cfg->txq_size);
  dma_tx_model_init(&me->txm, &me->txq, &me->db, cfg->tx_storage, cfg->tx_storage_size, me->port,
                    bsp_uart_dma_tx_start);

  // 启动 RX 循环 DMA
  bsp_uart_dma_rx_start(me->port, cfg->rx_dma_buf, cfg->rx_dma_cap);
  dma_rx_model_reset_position(&me->rxm);

  // 注册 TX 完成回调: DMA 发送完成 → dma_tx_model_on_done 续发下一段
  bsp_uart_set_tx_done_cb(me->port, uart_tx_done_cb, me);

  // 注册 RX 事件回调: 后端有新字节 → uart_rx_event (model update → rxq)
  bsp_uart_set_rx_event_cb(me->port, uart_rx_event_cb, me);
  return ERR_OK;
}

ErrorCode uart_set_config(Uart *me, const UartConfig *cfg) {
  if (me == NULL || cfg == NULL || cfg->port == NULL) {
    return ERR_ARG;
  }
  bsp_uart_dma_rx_stop(me->port);
  me->port = cfg->port;
  dma_rx_model_init(&me->rxm, cfg->rx_dma_buf, cfg->rx_dma_cap);
  ring_init(&me->rxq, cfg->rxq_buf, cfg->rxq_size);
  ring_init(&me->txq, cfg->txq_buf, cfg->txq_size);
  dma_tx_model_init(&me->txm, &me->txq, &me->db, cfg->tx_storage, cfg->tx_storage_size, me->port,
                    bsp_uart_dma_tx_start);
  bsp_uart_dma_rx_start(me->port, cfg->rx_dma_buf, cfg->rx_dma_cap);
  dma_rx_model_reset_position(&me->rxm);
  return ERR_OK;
}

void uart_deinit(Uart *me) {
  if (me == NULL) {
    return;
  }
  bsp_uart_dma_rx_stop(me->port);
  me->port = NULL;
  comm_base_deinit(&me->base);
}

// ======== 数据操作 (MAIN 上下文) ========

// 写: 入 TX 队列并启动 DMA (永不阻塞; 队列满只入能装下的)
ErrorCode uart_write(Uart *me, CommConstData data, IoCompletion comp) {
  if (me == NULL || data.ptr == NULL) {
    return ERR_ARG;
  }
  (void) comp;  // DMA model 天然异步 (IO_ASYNC_FLAG 语义)
  uint16_t n = dma_tx_model_submit(&me->txm, data.ptr, data.len);
  return (n == data.len) ? ERR_OK : ERR_NO_BUFF;
}

// 读: 从 RX 软件队列读出 (空 → ERR_EMPTY)
ErrorCode uart_read(Uart *me, CommData *data, IoCompletion comp) {
  if (me == NULL || data == NULL || data->ptr == NULL) {
    return ERR_ARG;
  }
  (void) comp;
  uint16_t n = ring_read(&me->rxq, data->ptr, data->len);
  if (n == 0u) {
    return ERR_EMPTY;
  }
  data->len = n;
  return ERR_OK;
}

// ======== ISR / 回调入口 ========

// 旧逐字节入口保留兼容 (直接入 rxq; DMA 模式优先走 uart_rx_event)
void uart_rx_push(Uart *me, uint8_t byte) {
  if (me == NULL) {
    return;
  }
  (void) ring_push(&me->rxq, byte);
}

// RX DMA 事件: 用后端 remaining 喂 DmaRxModel → 推 rxq (非阻塞)
void uart_rx_event(Uart *me) {
  if (me == NULL || me->port == NULL) {
    return;
  }
  uint16_t remaining = bsp_uart_dma_rx_remaining(me->port);
  (void) dma_rx_model_update(&me->rxm, remaining, &me->rxq);
}

// BSP RX 事件回调 (中断上下文): 转 uart_rx_event (model update -> rxq)
static void uart_rx_event_cb(BspUart *port, void *ctx) {
  (void) port;
  Uart *me = (Uart *) ctx;
  uart_rx_event(me);
}

// BSP TX 完成回调 (中断上下文): 转 model on_done 续发
static void uart_tx_done_cb(BspUart *port, void *ctx) {
  (void) port;
  Uart *me = (Uart *) ctx;
  dma_tx_model_on_done(&me->txm);
}

// TX DMA 完成: model on_done → 续发下一段
void uart_tx_complete(Uart *me) {
  if (me == NULL) {
    return;
  }
  dma_tx_model_on_done(&me->txm);
}
