// UART 字节流传输类 —— CommBase 子类 (Devices/comm, DMA model 接入版)
//
// 语义: 字节流. RX = 位置式循环 DMA (comp_dma_rx) → 软件队列 (MAIN 读);
//       TX = 双缓冲+队列+单发 DMA (comp_dma_tx, 完成续发).
// 平台: 全部经 BSP/bsp_uart.h 不透明句柄 (BspUart), BSP 之上零 HAL.
//   内部: bsp_uart_dma_rx_start/remaining (RX) + bsp_uart_dma_tx_start/busy (TX).
//
// 完成中断 (应用把 UART/DMA IRQ 路由进 HAL/driverlib 中断处理):
//   RX 事件 (RxEventCallback / SCIRXINT): 调 uart_rx_event(me) — 内部用
//     bsp_uart_dma_rx_remaining 喂 DmaRxModel.update → 推入 rxq.
//   TX 完成 (TxCpltCallback / SCITXINT): 调 uart_tx_complete(me) — 内部
//     dma_tx_model_on_done → 续发下一段.
//
// API 保持兼容: Uart / UartConfig / uart_init/read/write/rx_push/tx_complete
//   (motor_encoder 等按 Uart* + uart_write 引用; 缓冲经 UartConfig 注入, 零 malloc).

#ifndef COM_UART_H
#define COM_UART_H

#include "comp_comm.h"
#include "comp_io.h"
#include "comp_error_code.h"
#include "comp_ring.h"
#include "comp_dma_rx.h"
#include "comp_dma_tx.h"
#include "bsp_uart.h"

// 配置 POD — 调用者提供后端句柄与全部缓冲 (零 malloc, 静态分配)
typedef struct {
  BspUart *port;  // 不透明 UART 后端 (bsp_uart_bind 结果)
  // RX: 循环 DMA 缓冲 + 软件队列
  uint8_t *rx_dma_buf;  // DMA 循环缓冲 (位置式, cap 满回绕)
  uint16_t rx_dma_cap;  // RX DMA 缓冲容量
  uint8_t *rxq_buf;     // RX 软件队列缓冲 (MAIN 读)
  uint16_t rxq_size;    // RX 软件队列容量
  // TX: 双缓冲存储 + 待发队列
  uint8_t *tx_storage;  // TX 双缓冲存储 (对半切 active/pending)
  uint16_t tx_storage_size;
  uint8_t *txq_buf;   // TX 待发队列缓冲
  uint16_t txq_size;  // TX 待发队列容量
} UartConfig;

// UART 类 — 字节流, 内部 = DMA model + 软件队列 (零 HAL)
typedef struct {
  CommBase base;            // 基类 (必须第一成员)
  BspUart *port;            // 不透明后端
  DmaRxModel rxm;           // 位置式循环 RX model
  DoubleBuffer db;          // TX 双缓冲存储 (DmaTxModel 内部 active/pending)
  DmaTxModel txm;           // 双缓冲+队列 TX model
  Ring rxq;                 // RX 软件队列 (DMA 事件生产, MAIN 消费)
  Ring txq;                 // TX 待发队列 (MAIN 生产, model 消费)
  IoCompletion completion;  // 默认 IO_ASYNC_FLAG
} Uart;

ErrorCode uart_init(Uart *me, const UartConfig *cfg);
ErrorCode uart_set_config(Uart *me, const UartConfig *cfg);
void uart_deinit(Uart *me);

// 入 TX 队列并启动发送 (环满 → 只入能装下的, 返回实际入队字节数; 永不阻塞)
ErrorCode uart_write(Uart *me, CommConstData data, IoCompletion comp);

// 从 RX 软件队列读出 (空 → ERR_EMPTY; 实际读出数写回 data->len, 返回 ERR_OK)
ErrorCode uart_read(Uart *me, CommData *data, IoCompletion comp);

// ISR 入口: 旧逐字节 push 保留兼容 (直接入 rxq; 若用 DMA 循环则走 uart_rx_event)
void uart_rx_push(Uart *me, uint8_t byte);

// RX DMA 事件 (RxEventCallback/SCIRXINT 调): 用后端 remaining 喂 DmaRxModel → 推 rxq
void uart_rx_event(Uart *me);

// TX DMA 完成回调 (TxCpltCallback/SCITXINT 调): model on_done → 续发下一段
void uart_tx_complete(Uart *me);

#endif  // COM_UART_H
