// BSP SPI STM32 后端 — bsp_spi.h 的 STM32 (HAL) 实现
//
// 中断事务: HAL_SPI_TransmitReceive_IT / Transmit_IT / Receive_IT (非阻塞).
// 完成回调: 应用把 SPIx IRQ 路由进 HAL_SPI_IRQHandler,
//   在 HAL_SPI_TxRxCpltCallback / TxCpltCallback / RxCpltCallback / ErrorCallback
//   里调 bsp_spi_on_done (本层转调用户 cb).
//
// 由 cmake/HardC.CMake 的 st 分支编译; 系列由 bsp_stm32_hal.h 选择.

#include "bsp_spi.h"
#include "bsp_stm32_hal.h"

// 每实例在途事务状态 (单 SPI 句柄 → 单事务; 多实例需扩展为表)
typedef struct {
  SPI_HandleTypeDef *hspi;  // HAL 句柄
  BspSpiCb cb;              // 完成回调
  void *ctx;                // 回调上下文
  uint8_t busy;             // 是否有在途事务
} BspSpiStm32;

// 模块级单例 (当前支持单 SPI 实例; 多实例扩展为状态表)
static BspSpiStm32 s_spi = {0};

BspSpi *bsp_spi_bind(void *h) {
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *) h;
  if (hspi == NULL) {
    return NULL;
  }
  s_spi.hspi = hspi;
  s_spi.cb = NULL;
  s_spi.ctx = NULL;
  s_spi.busy = 0u;
  return (BspSpi *) hspi;  // 不透明 = HAL 句柄
}

void bsp_spi_unbind(BspSpi *me) {
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *) me;
  if (hspi != NULL) {
    (void) HAL_SPI_Abort(hspi);
  }
  s_spi.hspi = NULL;
  s_spi.cb = NULL;
  s_spi.ctx = NULL;
  s_spi.busy = 0u;
}

static bool spi_start(SPI_HandleTypeDef *hspi, const uint8_t *tx, uint8_t *rx, uint16_t len, BspSpiCb cb, void *ctx) {
  if (hspi == NULL || len == 0u || s_spi.busy) {
    return false;  // 参数错或已有在途事务
  }
  s_spi.cb = cb;
  s_spi.ctx = ctx;
  s_spi.busy = 1u;
  HAL_StatusTypeDef st;
  if (tx != NULL && rx != NULL) {
    st = HAL_SPI_TransmitReceive_IT(hspi, (uint8_t *) tx, rx, len);
  } else if (tx != NULL) {
    st = HAL_SPI_Transmit_IT(hspi, (uint8_t *) tx, len);
  } else {
    st = HAL_SPI_Receive_IT(hspi, rx, len);
  }
  if (st != HAL_OK) {
    s_spi.busy = 0u;
    return false;
  }
  return true;
}

bool bsp_spi_transfer_async(BspSpi *me, const uint8_t *tx, uint8_t *rx, uint16_t len, BspSpiCb cb, void *ctx) {
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *) me;
  if (tx == NULL || rx == NULL) {
    return false;
  }
  return spi_start(hspi, tx, rx, len, cb, ctx);
}

bool bsp_spi_write_async(BspSpi *me, const uint8_t *tx, uint16_t len, BspSpiCb cb, void *ctx) {
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *) me;
  if (tx == NULL) {
    return false;
  }
  return spi_start(hspi, tx, NULL, len, cb, ctx);
}

bool bsp_spi_read_async(BspSpi *me, uint8_t *rx, uint16_t len, BspSpiCb cb, void *ctx) {
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *) me;
  if (rx == NULL) {
    return false;
  }
  return spi_start(hspi, NULL, rx, len, cb, ctx);
}

// 平台完成钩子: HAL SPI 完成/错误回调调用
void bsp_spi_on_done(BspSpi *me, int ec) {
  (void) me;
  BspSpiCb cb = s_spi.cb;
  void *ctx = s_spi.ctx;
  s_spi.busy = 0u;  // 释放在途
  s_spi.cb = NULL;
  s_spi.ctx = NULL;
  if (cb != NULL) {
    cb((BspSpi *) s_spi.hspi, ec, ctx);
  }
}

// ---- HAL 弱回调重写 (应用把 SPIx IRQ 路由进 HAL_SPI_IRQHandler 后触发) ----

__weak void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi == s_spi.hspi) {
    bsp_spi_on_done((BspSpi *) hspi, 0);
  }
}

__weak void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi == s_spi.hspi) {
    bsp_spi_on_done((BspSpi *) hspi, 0);
  }
}

__weak void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi == s_spi.hspi) {
    bsp_spi_on_done((BspSpi *) hspi, 0);
  }
}

__weak void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
  if (hspi == s_spi.hspi) {
    bsp_spi_on_done((BspSpi *) hspi, 1);  // ec=1: 错误
  }
}
