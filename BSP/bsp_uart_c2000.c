// BSP UART C2000 后端 — bsp_uart.h 的 TMS320F28004x 实现 (driverlib)
//
// 语义映射 (与 bsp_c2000_adc.c 同约定): C2000 SCI 无循环 DMA 便捷通道,
//   "DMA 搬移"由 SCIRXINT 接收中断承担: 每收到一字节 → 写入 DmaRxModel 的
//   循环缓冲 (cap 满回绕), remaining 由内部"未读字节计数"提供 → 与 STM32
//   位置式循环 DMA 在 bsp_uart.h 接口层面等价.
//
// TX: SCI 无单发 DMA 便捷路径 → 用 SCITXINT 逐字节状态机 (整块 data[0..len) 逐字节发,
//   末字节触发完成回调 → 上层续发下一段).  [待 C2000 工具链验证 driverlib API 名]
//
// 前提: SysConfig board.c (或工程 setup) 已完成 SCI GPIO/波特率/使能 + RX/TX 中断注册.
// 本层只做中断搬运 + remaining/tx_busy 记账, 不碰 GPIO 配置 — 与 bsp_c2000_adc/epwm 同约定.
//
// 约束: 单 SCI 实例 (SCIA) 模块级单例 + 单 ISR. 多 SCI (A/B/C/D) 需扩展为实例状态表.
// 注意: 本文件为 driverlib 调用骨架, 需在 C2000 工具链下验证具体 API 名称.

#include "bsp_uart.h"
#include <stdbool.h>
#include "driverlib.h"

// ======== 模块级状态 (单 SCIA) ========

static uint32_t s_sci_base = 0u;    // SCI 基址 (SCIA_BASE)
static uint8_t *volatile s_rx_buf;  // DmaRxModel 循环缓冲 (cap 满回绕)
static volatile uint16_t s_rx_cap;  // 循环缓冲容量
static volatile uint16_t s_rx_wr;   // 内部写位置 (0..cap-1)
static volatile bool s_isr_registered;
static BspUartRxEventFn s_rx_evt_cb = NULL;  // RX 事件回调 (ISR 调用)
static void *s_rx_evt_ctx = NULL;
// TX 逐字节状态机 (SCITXINT 驱动): 整块 data[0..len) 逐字节发, 末字节触发完成回调
static BspUartTxDoneFn s_tx_done_cb = NULL;
static void *s_tx_done_ctx = NULL;
static const uint8_t *s_tx_data;  // 当前发送块
static uint16_t s_tx_len;         // 块总长
static uint16_t s_tx_pos;         // 已发字节数

// ======== SCI RX 接收中断 — 硬件"DMA": 逐字节写入循环缓冲 ========
// 与 bsp_c2000_adc_isr 同职责: 只搬移 + 清中断, 不做业务.
__interrupt void bsp_c2000_uart_rx_isr(void) {
  // 读尽 RX 缓冲 (SCI RXFF 满或每字节触发)
  while ((SCI_getRxStatus(s_sci_base) & SCI_RX_EMPTY) == 0u) {  // 非空才读 (TI: RXEMPTY=1 表示空)
    uint8_t byte = (uint8_t) SCI_readCharNonBlocking(s_sci_base);
    if (s_rx_buf && s_rx_cap > 0u) {
      s_rx_buf[s_rx_wr] = byte;
      s_rx_wr = (uint16_t) ((s_rx_wr + 1u) % s_rx_cap);
    }
    if (SCI_getRxStatus(s_sci_base) & SCI_RX_EMPTY) {  // 读空即停
      break;
    }
  }
  SCI_clearInterruptStatus(s_sci_base, SCI_INT_RXFF);
  Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);  // SCI-A RX -> PIE 组 9

  // RX 事件: 通知上层有新字节 (com_uart -> uart_rx_event -> model update)
  if (s_rx_evt_cb != NULL) {
    s_rx_evt_cb((BspUart *) s_sci_base, s_rx_evt_ctx);
  }
}

// ======== SCI TX 发送中断 — 逐字节续发 + 末字节完成回调 ========
// 每次 TXFF 中断发一个字节; 发完末字节 → 清状态 + 触发 TX 完成回调 (上层续发下一段).
__interrupt void bsp_c2000_uart_tx_isr(void) {
  if (s_tx_len == 0u) {
    // 无在途块 (防御): 清中断即可
    SCI_clearInterruptStatus(s_sci_base, SCI_INT_TXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);  // SCI-A TX 同组 [待验证]
    return;
  }
  if (s_tx_pos < s_tx_len) {
    // 还有字节: 发下一个
    SCI_writeCharNonBlocking(s_sci_base, s_tx_data[s_tx_pos]);
    s_tx_pos++;
  } else {
    // 整块发完: 清状态 + 完成回调
    s_tx_len = 0u;
    s_tx_pos = 0u;
    s_tx_data = NULL;
    SCI_clearInterruptStatus(s_sci_base, SCI_INT_TXFF);
    if (s_tx_done_cb != NULL) {
      s_tx_done_cb((BspUart *) s_sci_base, s_tx_done_ctx);
    }
  }
  Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);  // [待验证]
}

// ======== BSP 接口 ========

void bsp_uart_set_rx_event_cb(BspUart *me, BspUartRxEventFn cb, void *ctx) {
  (void) me;
  s_rx_evt_cb = cb;
  s_rx_evt_ctx = ctx;
}

void bsp_uart_set_tx_done_cb(BspUart *me, BspUartTxDoneFn cb, void *ctx) {
  (void) me;
  s_tx_done_cb = cb;
  s_tx_done_ctx = ctx;
}

BspUart *bsp_uart_bind(void *h) {
  uint32_t base = (uint32_t) h;
  if (base == 0u) {
    return NULL;
  }
  s_sci_base = base;
  s_rx_evt_cb = NULL;
  s_rx_evt_ctx = NULL;
  s_tx_done_cb = NULL;
  s_tx_done_ctx = NULL;
  s_tx_data = NULL;
  s_tx_len = 0u;
  s_tx_pos = 0u;
  return (BspUart *) base;  // 不透明 = SCI 基址
}

void bsp_uart_unbind(BspUart *me) {
  if (me == NULL) {
    return;
  }
  // 尽力停收: 关 RX 中断 (SCI 外设使能由工程/SysConfig 负责)
  SCI_disableInterrupt(s_sci_base, SCI_INT_RXFF);
  SCI_disableInterrupt(s_sci_base, SCI_INT_TXFF);  // [待验证]
  s_tx_data = NULL;
  s_tx_len = 0u;
  s_tx_pos = 0u;
  s_rx_buf = NULL;
  s_rx_cap = 0u;
}

// ---- RX (位置式循环等价: 中断逐字节入缓冲) ----

void bsp_uart_dma_rx_start(BspUart *me, uint8_t *buf, uint16_t cap) {
  if (me == NULL || buf == NULL || cap == 0u) {
    return;
  }
  s_rx_buf = buf;
  s_rx_cap = cap;
  s_rx_wr = 0u;

  if (!s_isr_registered) {
    // F28004x SCI-A RX: PIE 组 9, 向量 INT_SCIA_RX_INT (工具链核对)
    Interrupt_register(INT_SCIA_RX_INT, &bsp_c2000_uart_rx_isr);
    // 同时注册 TX ISR (SCITXINT 续发; [待验证] INT_SCIA_TX_INT 向量名)
    Interrupt_register(INT_SCIA_TX_INT, &bsp_c2000_uart_tx_isr);  // [待验证]
    s_isr_registered = true;
  }
  SCI_clearInterruptStatus(s_sci_base, SCI_INT_RXFF);
  SCI_enableInterrupt(s_sci_base, SCI_INT_RXFF);
  Interrupt_enable(INT_SCIA_RX_INT);
}

uint16_t bsp_uart_dma_rx_remaining(BspUart *me) {
  (void) me;
  // remaining 语义对齐 comp_dma_rx / STM32 __HAL_DMA_GET_COUNTER:
  //   返回"DMA 还剩多少字节没写到 cap"(即 cap - 当前写位置).
  //   上层 cur = cap - remaining = 写位置 → 与 STM32 一致.
  // C2000 用中断逐字节写 s_rx_wr (DMA 写位置等价); remaining = cap - s_rx_wr.
  if (s_rx_cap == 0u) {
    return 0u;
  }
  return (uint16_t) (s_rx_cap - s_rx_wr);
}

void bsp_uart_dma_rx_stop(BspUart *me) {
  (void) me;
  SCI_disableInterrupt(s_sci_base, SCI_INT_RXFF);
  SCI_disableInterrupt(s_sci_base, SCI_INT_TXFF);  // [待验证]
  s_tx_data = NULL;
  s_tx_len = 0u;
  s_tx_pos = 0u;
}

// ---- TX (逐字节发送, 非阻塞; 完成续发由上层 com_uart) ----

bool bsp_uart_dma_tx_start(BspUart *me, const uint8_t *data, uint16_t len) {
  (void) me;
  if (data == NULL || len == 0u || s_tx_len != 0u) {
    return false;  // 参数错或已有在途 TX
  }
  // 启动: 记录整块, 若 TX 空闲立即发首字节 (余下由 SCITXINT 中断续发)
  s_tx_data = data;
  s_tx_len = len;
  s_tx_pos = 0u;
  if (SCI_getTxStatus(s_sci_base) & SCI_TX_EMPTY) {
    SCI_writeCharNonBlocking(s_sci_base, data[0]);
    s_tx_pos = 1u;
  }
  // 使能 TXFF 中断 (待验证: SCI_INT_TXFF 宏名) — 后续字节由 ISR 续发
  SCI_enableInterrupt(s_sci_base, SCI_INT_TXFF);  // [待验证]
  return true;
}

bool bsp_uart_dma_tx_busy(BspUart *me) {
  (void) me;
  // 忙 = 有在途 TX 块
  return s_tx_len != 0u;
}
