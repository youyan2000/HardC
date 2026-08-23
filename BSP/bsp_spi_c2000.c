// BSP SPI C2000 后端 — bsp_spi.h 的 TMS320F28004x 实现 (driverlib)
//
// 中断事务: C2000 SPI 无便捷 DMA 单发路径, 用 SPIRXINT/SPITXINT FIFO 中断逐字节
//   搬移 (与 bsp_uart_c2000.c 同约定)。每事务一个字节收发, 完成触发回调。
//
// [待验证] driverlib API 名 (SPI_getRxFifoStatus/SPI_putDataNonBlocking/
//   SPI_INT_RXFF/SPI_INT_TXFF/INT_SPIA_RX_INT/INT_SPIA_TX_INT 等) 需在 C2000 工具链核对.
//
// 前提: SysConfig board.c (或工程 setup) 已完成 SPI GPIO/波特率/使能 + FIFO 中断注册.
// 本层只做中断搬运 + 完成回调, 不碰 GPIO 配置 — 与 bsp_c2000_adc/epwm 同约定.

#include "bsp_spi.h"
#include <stdbool.h>
#include "driverlib.h"

// ======== 模块级状态 (单 SPIA) ========

static uint32_t s_spi_base = 0u;  // SPI 基址 (SPIA_BASE)
static BspSpiCb s_cb = NULL;      // 完成回调
static void *s_ctx = NULL;        // 回调上下文
static const uint8_t *s_tx;       // 发送缓冲 (NULL=无)
static uint8_t *s_rx;             // 接收缓冲 (NULL=无)
static uint16_t s_len;            // 事务总长
static uint16_t s_pos;            // 已搬移字节数
static bool s_busy;               // 是否有在途事务
static bool s_isr_registered;

// ======== SPI RX 接收中断 — 逐字节收 + 逐字节发 (全双工) ========
// 与 bsp_c2000_adc_isr 同职责: 只搬移 + 清中断, 不做业务.
__interrupt void bsp_c2000_spi_rx_isr(void) {
  if (!s_busy) {
    // 无在途事务 (防御): 清中断
    SPI_clearInterruptStatus(s_spi_base, SPI_INT_RXFF);  // [待验证]
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);       // [待验证]
    return;
  }
  // 收 1 字节 (SPI 主发: 每发一个字节, RX 移入一个 — 首字节由 start 预写触发)
  uint8_t in = (uint8_t) SPI_readDataNonBlocking(s_spi_base);  // [待验证]
  if (s_rx != NULL) {
    s_rx[s_pos] = in;  // rx[s_pos] 对应已发 tx[s_pos]
  }
  s_pos++;

  if (s_pos >= s_len) {
    // 已收满 len 字节 → 事务完成 (勿在写 FIFO 时提前收尾)
    s_busy = false;
    s_len = 0u;
    s_tx = NULL;
    s_rx = NULL;
    SPI_clearInterruptStatus(s_spi_base, SPI_INT_RXFF);  // [待验证]
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);       // [待验证]
    if (s_cb != NULL) {
      s_cb((BspSpi *) s_spi_base, 0, s_ctx);
    }
    return;
  }

  // 发下一个字节 (驱动时钟; 全双工/单工发: 取 s_tx[s_pos]; 单工收: 补 0xFF)
  uint8_t out = 0xFFu;
  if (s_tx != NULL) {
    out = s_tx[s_pos];
  }
  SPI_writeDataNonBlocking(s_spi_base, out);           // [待验证]
  SPI_clearInterruptStatus(s_spi_base, SPI_INT_RXFF);  // [待验证]
  Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);       // [待验证]
}

// ======== BSP 接口 ========

BspSpi *bsp_spi_bind(void *h) {
  uint32_t base = (uint32_t) h;
  if (base == 0u) {
    return NULL;
  }
  s_spi_base = base;
  s_cb = NULL;
  s_ctx = NULL;
  s_tx = NULL;
  s_rx = NULL;
  s_len = 0u;
  s_pos = 0u;
  s_busy = false;
  return (BspSpi *) base;  // 不透明 = SPI 基址
}

void bsp_spi_unbind(BspSpi *me) {
  (void) me;
  if (s_isr_registered) {
    Interrupt_disable(INT_SPIA_RX_INT);  // [待验证]
  }
  s_busy = false;
  s_cb = NULL;
  s_tx = NULL;
  s_rx = NULL;
  s_len = 0u;
}

// 启动一次 SPI 中断事务 (全双工或单工, tx/rx 可空)
static bool spi_start_async(const uint8_t *tx, uint8_t *rx, uint16_t len, BspSpiCb cb, void *ctx) {
  if (len == 0u || s_busy) {
    return false;
  }
  s_tx = tx;
  s_rx = rx;
  s_len = len;
  s_pos = 0u;
  s_busy = true;
  s_cb = cb;
  s_ctx = ctx;

  if (!s_isr_registered) {
    Interrupt_register(INT_SPIA_RX_INT, &bsp_c2000_spi_rx_isr);  // [待验证]
    s_isr_registered = true;
  }
  // 发首字节 (s_pos=0: 触发 RX 移入 rx[0]; ISR 随后发 tx[1] 收 rx[1]...)
  uint8_t out = (tx != NULL) ? tx[0] : 0xFFu;
  SPI_writeDataNonBlocking(s_spi_base, out);           // [待验证]
  s_pos = 0u;                                          // 首个 RX 移入 rx[0]
  SPI_clearInterruptStatus(s_spi_base, SPI_INT_RXFF);  // [待验证]
  SPI_enableInterrupt(s_spi_base, SPI_INT_RXFF);       // [待验证]
  Interrupt_enable(INT_SPIA_RX_INT);                   // [待验证]
  return true;
}

bool bsp_spi_transfer_async(BspSpi *me, const uint8_t *tx, uint8_t *rx, uint16_t len, BspSpiCb cb, void *ctx) {
  (void) me;
  if (tx == NULL || rx == NULL) {
    return false;
  }
  return spi_start_async(tx, rx, len, cb, ctx);
}

bool bsp_spi_write_async(BspSpi *me, const uint8_t *tx, uint16_t len, BspSpiCb cb, void *ctx) {
  (void) me;
  if (tx == NULL) {
    return false;
  }
  return spi_start_async(tx, NULL, len, cb, ctx);
}

bool bsp_spi_read_async(BspSpi *me, uint8_t *rx, uint16_t len, BspSpiCb cb, void *ctx) {
  (void) me;
  if (rx == NULL) {
    return false;
  }
  return spi_start_async(NULL, rx, len, cb, ctx);
}

void bsp_spi_on_done(BspSpi *me, int ec) {
  (void) me;
  // C2000 由 ISR 直接调 s_cb; 本钩子供 HAL 风格平台 (STM32) 使用, C2000 空实现.
  (void) ec;
}
