// UART 字节流传输类 —— CommBase 子类 (Devices/comm, 阶段1 重构)
//
// 语义: 字节流. RX 由 ISR 逐字节收入 SPSC 环, MAIN 读; TX 由 MAIN 入环,
//   逐字节中断发送 (单字节 IT, 够 CTX_MAIN 用, 无需 DMA 空闲中断).
//
// 数据面 (SPSC 环 + 发送状态) 全部在子类结构体, 不进 CommBase 虚表;
// 总线 IRQ 收字节走专用 ISR 入口 uart_rx_push (只入环, 永不阻塞).
//
// 传输语义外部独立: 完成方式在调用时经 IoCompletion 绑定.

#ifndef COM_UART_H
#define COM_UART_H

#include "comp_comm.h"
#include "comp_io.h"
#include "comp_error_code.h"
#include "comp_ring.h"
#include "bsp_stm32_hal.h"

// 配置 POD — 调用者提供环缓冲 (零 malloc)
typedef struct {
  UART_HandleTypeDef *huart;  // App 注入 HAL 句柄
  uint8_t *rx_buf;            // RX 环缓冲
  uint16_t rx_size;           // RX 环容量
  uint8_t *tx_buf;            // TX 环缓冲
  uint16_t tx_size;           // TX 环容量
} UartConfig;

// UART 类 — 字节流, RX: ISR→SPSC 环→MAIN 读; TX: MAIN 入环→逐字节中断发
typedef struct {
  CommBase base;              // 基类 (必须第一成员)
  UART_HandleTypeDef *huart;  // HAL UART 句柄
  Ring rx;                    // RX 环 (ISR 生产, MAIN 消费)
  Ring tx;                    // TX 环 (MAIN 生产, ISR 消费)
  IoCompletion completion;    // 默认 IO_ASYNC_FLAG
  volatile uint8_t tx_busy;   // TX 传输进行中 (ISR 清 / MAIN 忙等读, 必须 volatile)
  uint8_t rx_byte;            // HAL IT 接收单字节缓存
  uint8_t tx_byte;            // HAL IT 发送单字节缓存
} Uart;

void uart_init(Uart *me, const UartConfig *cfg);
void uart_set_config(Uart *me, const UartConfig *cfg);
void uart_deinit(Uart *me);

// 入 TX 环并启动发送 (环满 → ERR_NO_BUFF; comp=IO_SYNC 则忙等发送结束)
ErrorCode uart_write(Uart *me, CommConstData data, IoCompletion comp);

// 从 RX 环读出 (空 → ERR_EMPTY; 实际读出数写回 data->len, 返回 ERR_OK)
ErrorCode uart_read(Uart *me, CommData *data, IoCompletion comp);

// ISR 入口: 入 RX 环 (ring_push, 满则丢弃)
void uart_rx_push(Uart *me, uint8_t byte);

// HAL TX 完成回调调用: 取下一字节继续发, 环空则清 tx_busy
void uart_tx_complete(Uart *me);

#endif  // COM_UART_H
