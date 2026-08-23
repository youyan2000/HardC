// BSP I2C STM32 后端 — bsp_i2c.h 的 STM32 (HAL) 实现
//
// 中断事务: HAL_I2C_Mem_Write_IT / Mem_Read_IT (非阻塞).
// 完成回调: 应用把 I2Cx_EV/ER IRQ 路由进 HAL_I2C_EV_IRQHandler / HAL_I2C_ER_IRQHandler,
//   在 HAL_I2C_MemTxCpltCallback / HAL_I2C_MemRxCpltCallback / HAL_I2C_ErrorCallback
//   里调 bsp_i2c_on_done (本层转调用户 cb).
//
// 由 cmake/HardC.CMake 的 st 分支编译; 系列由 bsp_stm32_hal.h 选择.

#include "bsp_i2c.h"
#include "bsp_stm32_hal.h"

// F1 HAL 用 I2C_MEMADD_SIZE_8BIT, F3/F4/G4/H7 用 I2C_MEMSIZE_8BIT — 系列桥接 (M4)
#if defined(I2C_MEMSIZE_8BIT)
#define BSP_I2C_MEMSIZE I2C_MEMSIZE_8BIT
#elif defined(I2C_MEMADD_SIZE_8BIT)
#define BSP_I2C_MEMSIZE I2C_MEMADD_SIZE_8BIT
#else
#define BSP_I2C_MEMSIZE 0u  // 未知系列: 占位 (需在目标 HAL 核对)
#endif

// 每实例在途事务状态 (单 I2C 句柄 → 单事务; 多实例需扩展为表)
typedef struct {
  I2C_HandleTypeDef *hi2c;  // HAL 句柄
  BspI2cCb cb;              // 完成回调
  void *ctx;                // 回调上下文
  uint8_t busy;             // 是否有在途事务
} BspI2cStm32;

// 模块级单例 (当前支持单 I2C 实例; 多实例扩展为状态表)
static BspI2cStm32 s_i2c = {0};

BspI2c *bsp_i2c_bind(void *h) {
  I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *) h;
  if (hi2c == NULL) {
    return NULL;
  }
  s_i2c.hi2c = hi2c;
  s_i2c.cb = NULL;
  s_i2c.ctx = NULL;
  s_i2c.busy = 0u;
  return (BspI2c *) hi2c;  // 不透明 = HAL 句柄
}

void bsp_i2c_unbind(BspI2c *me) {
  I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *) me;
  if (hi2c != NULL) {
    (void) HAL_I2C_DeInit(hi2c);
  }
  s_i2c.hi2c = NULL;
  s_i2c.cb = NULL;
  s_i2c.ctx = NULL;
  s_i2c.busy = 0u;
}

bool bsp_i2c_mem_write_async(BspI2c *me, uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint16_t len, BspI2cCb cb,
                             void *ctx) {
  I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *) me;
  if (hi2c == NULL || data == NULL || len == 0u || s_i2c.busy) {
    return false;  // 参数错或已有在途事务
  }
  s_i2c.cb = cb;
  s_i2c.ctx = ctx;
  s_i2c.busy = 1u;
  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Write_IT(hi2c, (uint16_t) (dev_addr << 1), reg, BSP_I2C_MEMSIZE, (uint8_t *) data, len);
  if (st != HAL_OK) {
    s_i2c.busy = 0u;
    return false;
  }
  return true;
}

bool bsp_i2c_mem_read_async(BspI2c *me, uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint16_t len, BspI2cCb cb,
                            void *ctx) {
  I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *) me;
  if (hi2c == NULL || buf == NULL || len == 0u || s_i2c.busy) {
    return false;
  }
  s_i2c.cb = cb;
  s_i2c.ctx = ctx;
  s_i2c.busy = 1u;
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read_IT(hi2c, (uint16_t) (dev_addr << 1), reg, BSP_I2C_MEMSIZE, buf, len);
  if (st != HAL_OK) {
    s_i2c.busy = 0u;
    return false;
  }
  return true;
}

// 平台完成钩子: HAL I2C 完成/错误回调调用
void bsp_i2c_on_done(BspI2c *me, int ec) {
  (void) me;
  BspI2cCb cb = s_i2c.cb;
  void *ctx = s_i2c.ctx;
  s_i2c.busy = 0u;  // 释放在途
  s_i2c.cb = NULL;
  s_i2c.ctx = NULL;
  if (cb != NULL) {
    cb((BspI2c *) s_i2c.hi2c, ec, ctx);
  }
}

// ---- HAL 弱回调重写 (应用把 I2C EV/ER IRQ 路由进 HAL_I2C_*_IRQHandler 后触发) ----

__weak void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c == s_i2c.hi2c) {
    bsp_i2c_on_done((BspI2c *) hi2c, 0);
  }
}

__weak void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c == s_i2c.hi2c) {
    bsp_i2c_on_done((BspI2c *) hi2c, 0);
  }
}

__weak void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c == s_i2c.hi2c) {
    bsp_i2c_on_done((BspI2c *) hi2c, 1);  // ec=1: 错误
  }
}
