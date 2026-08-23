// SPI 传输类实现 —— CommBase 子类 (Devices/comm, 中断事务接入版)
//
// 传输经 bsp_spi.h 中断事务 (非阻塞, 完成回调收 ec); CS 经 bsp_gpio 管理.
//   CS 在完成回调里释放 (异步安全: CS 全程保持到事务真正完成, 不从机截断).
//   spi_write/read/transfer 保持签名; spi_read_reg 用单段全双工 (天然异步安全).
//   comp=IO_SYNC 时轮询完成 (仅 MAIN).

#include "com_spi.h"
#include <stddef.h>

// bsp_spi 完成回调: 记录 ec + 释放 CS + 清 busy (中断上下文, 非阻塞)
static void spi_on_done(BspSpi *port, int ec, void *ctx) {
  (void) port;
  Spi *me = (Spi *) ctx;
  me->last_ec = ec;
  if (me->cs_active) {
    bsp_gpio_write(&me->cs, true);  // 事务真正完成才释放 CS
    me->cs_active = 0;
  }
  me->tx_busy = 0;  // 释放在途 (单事务)
}

// 完成等待辅助: IO_SYNC 时轮询 (仅 CTX_MAIN 允许)
static ErrorCode spi_wait_done(Spi *me, IoCompletion comp) {
  if (comp == IO_SYNC) {
    while (me->tx_busy) {}
    return (me->last_ec == 0) ? ERR_OK : ERR_FAILED;
  }
  return ERR_OK;  // 异步: 完成由回调记录 last_ec + 释放 CS
}

// ======== 构造 / 析构 / 配置 ========

ErrorCode spi_init(Spi *me, const SpiConfig *cfg) {
  if (me == NULL || cfg == NULL || cfg->port == NULL) {
    return ERR_ARG;
  }
  comm_base_init(&me->base, "spi");
  me->port = cfg->port;
  me->cs = cfg->cs;
  me->completion = IO_ASYNC_FLAG;
  me->tx_busy = 0;
  me->last_ec = 0;
  me->cs_active = 0;
  bsp_gpio_cfg_output(&me->cs);
  bsp_gpio_write(&me->cs, true);  // CS 空闲拉高
  return ERR_OK;
}

ErrorCode spi_set_config(Spi *me, const SpiConfig *cfg) {
  if (me == NULL || cfg == NULL || cfg->port == NULL) {
    return ERR_ARG;
  }
  me->port = cfg->port;
  me->cs = cfg->cs;
  bsp_gpio_cfg_output(&me->cs);
  bsp_gpio_write(&me->cs, true);
  return ERR_OK;
}

void spi_deinit(Spi *me) {
  if (me == NULL) {
    return;
  }
  bsp_gpio_write(&me->cs, true);
  me->port = NULL;
  comm_base_deinit(&me->base);
}

// ======== 数据操作 (MAIN 上下文, 中断事务) ========

// 写: CS 低 → 异步发 → 完成回调释放 CS
ErrorCode spi_write(Spi *me, CommConstData data, IoCompletion comp) {
  if (me == NULL || me->port == NULL || data.ptr == NULL) {
    return ERR_ARG;
  }
  if (me->tx_busy) {
    return ERR_BUSY;  // 已有在途事务
  }
  bsp_gpio_write(&me->cs, false);
  me->cs_active = 1;
  me->tx_busy = 1;  // 先置忙再启动 (防完成回调先到)
  if (!bsp_spi_write_async(me->port, data.ptr, data.len, spi_on_done, me)) {
    me->tx_busy = 0;
    me->cs_active = 0;
    bsp_gpio_write(&me->cs, true);
    return ERR_BUSY;
  }
  return spi_wait_done(me, comp);
}

// 读: CS 低 → 异步收 → 完成回调释放 CS
ErrorCode spi_read(Spi *me, CommData data, IoCompletion comp) {
  if (me == NULL || me->port == NULL || data.ptr == NULL) {
    return ERR_ARG;
  }
  if (me->tx_busy) {
    return ERR_BUSY;
  }
  bsp_gpio_write(&me->cs, false);
  me->cs_active = 1;
  me->tx_busy = 1;
  if (!bsp_spi_read_async(me->port, data.ptr, data.len, spi_on_done, me)) {
    me->tx_busy = 0;
    me->cs_active = 0;
    bsp_gpio_write(&me->cs, true);
    return ERR_BUSY;
  }
  return spi_wait_done(me, comp);
}

// 全双工: 同时收发
ErrorCode spi_transfer(Spi *me, CommConstData tx, CommData rx, IoCompletion comp) {
  if (me == NULL || me->port == NULL || tx.ptr == NULL || rx.ptr == NULL) {
    return ERR_ARG;
  }
  if (rx.len < tx.len) {
    return ERR_SIZE;
  }
  if (me->tx_busy) {
    return ERR_BUSY;
  }
  bsp_gpio_write(&me->cs, false);
  me->cs_active = 1;
  me->tx_busy = 1;
  if (!bsp_spi_transfer_async(me->port, tx.ptr, rx.ptr, tx.len, spi_on_done, me)) {
    me->tx_busy = 0;
    me->cs_active = 0;
    bsp_gpio_write(&me->cs, true);
    return ERR_BUSY;
  }
  return spi_wait_done(me, comp);
}

// 寄存器读: 单段全双工 (tx=[reg,0..0] len+1, rx=[dummy,data] len+1) — 天然异步安全
ErrorCode spi_read_reg(Spi *me, uint8_t reg, uint8_t *dat, uint16_t len, IoCompletion comp) {
  if (me == NULL || me->port == NULL || dat == NULL) {
    return ERR_ARG;
  }
  if (me->tx_busy) {
    return ERR_BUSY;
  }
  // 栈上组 tx (reg + len 个 0xFF 驱动时钟) 与 rx (dummy + 数据)
  // 注: tx 需在事务期间保持有效 — 中断事务是异步的, 栈缓冲在 spi_wait_done(IO_SYNC)
  //     返回前有效; 异步模式 (comp != IO_SYNC) 下栈缓冲会在函数返回后失效 → 风险.
  // 因此: IO_SYNC 用栈缓冲; 异步模式暂不支持 read_reg (返回 ERR_NOT_SUPPORT),
  //       避免悬垂指针 (见头文件注释).
  if (comp != IO_SYNC) {
    return ERR_NOT_SUPPORT;  // 异步 read_reg 需调用方持有缓冲, 暂不支持
  }
  uint8_t tx[2 + 64];
  uint8_t rx[2 + 64];
  if (len > 64u) {
    return ERR_SIZE;  // 栈缓冲上限 (传感器寄存器访问通常 ≤ 8)
  }
  tx[0] = reg;
  for (uint16_t i = 1; i <= len; i++) {
    tx[i] = 0xFFu;  // 驱动时钟
  }
  bsp_gpio_write(&me->cs, false);
  me->cs_active = 1;
  me->tx_busy = 1;
  if (!bsp_spi_transfer_async(me->port, tx, rx, (uint16_t) (len + 1u), spi_on_done, me)) {
    me->tx_busy = 0;
    me->cs_active = 0;
    bsp_gpio_write(&me->cs, true);
    return ERR_BUSY;
  }
  ErrorCode ec = spi_wait_done(me, comp);
  if (ec == ERR_OK) {
    for (uint16_t i = 0; i < len; i++) {
      dat[i] = rx[i + 1];  // 跳过 dummy 字节
    }
  }
  return ec;
}
