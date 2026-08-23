// I2C 传输类实现 —— CommBase 子类 (Devices/comm, 仅硬件 I2C, 中断事务)
//
// SW bit-bang 已删除 (用户裁定: SW I2C 用不了 DMA 就必须阻塞 → HardC 不支持).
// 仅硬件 I2C: 寄存器寻址事务经 bsp_i2c.h 中断事务 (非阻塞, 完成回调收 ec).
//   iic_write_reg/read_reg 保持签名; comp=IO_SYNC 时轮询完成 (仅 MAIN).
// BSP 之上零 HAL: 持 BspI2c* 不透明句柄.

#include "com_i2c.h"
#include <stddef.h>

// bsp_i2c 完成回调: 记录 ec + 清 busy (中断上下文, 非阻塞)
static void iic_on_done(BspI2c *port, int ec, void *ctx) {
  (void) port;
  Iic *me = (Iic *) ctx;
  me->last_ec = ec;
  me->tx_busy = 0;  // 释放在途 (单事务)
}

// -------- 构造 / 析构 --------

// 硬件 I2C 初始化 (port = bsp_i2c_bind 结果)
ErrorCode iic_init_hw(Iic *me, BspI2c *port, uint8_t dev_addr) {
  if (me == NULL || port == NULL) {
    return ERR_ARG;
  }
  comm_base_init(&me->base, "i2c");
  me->mode = IIC_HW;
  me->port = port;
  me->dev_addr = dev_addr;
  me->completion = IO_ASYNC_FLAG;
  me->tx_busy = 0;
  me->last_ec = 0;
  return ERR_OK;
}

// 反初始化: 解绑 + 清状态
void iic_deinit(Iic *me) {
  if (me == NULL) {
    return;
  }
  bsp_i2c_unbind(me->port);
  me->port = NULL;
  me->mode = IIC_HW;
  me->dev_addr = 0;
  comm_base_deinit(&me->base);
}

// -------- 公共寄存器事务 (中断, 非阻塞) --------

// 写寄存器 → 写 len 字节 (非阻塞; IO_SYNC 时轮询完成, 仅 MAIN)
ErrorCode iic_write_reg(Iic *me, uint8_t reg, const uint8_t *dat, uint16_t len, IoCompletion comp) {
  if (me == NULL || me->port == NULL || dat == NULL) {
    return ERR_ARG;
  }
  if (!bsp_i2c_mem_write_async(me->port, me->dev_addr, reg, dat, len, iic_on_done, me)) {
    return ERR_BUSY;  // 参数错或已有在途事务
  }
  me->tx_busy = 1;
  if (comp == IO_SYNC) {
    while (me->tx_busy) {}
    return (me->last_ec == 0) ? ERR_OK : ERR_FAILED;
  }
  return ERR_OK;  // 异步: 完成由回调记录 last_ec
}

// 写寄存器 → 读 len 字节 (非阻塞)
ErrorCode iic_read_reg(Iic *me, uint8_t reg, uint8_t *dat, uint16_t len, IoCompletion comp) {
  if (me == NULL || me->port == NULL || dat == NULL) {
    return ERR_ARG;
  }
  if (!bsp_i2c_mem_read_async(me->port, me->dev_addr, reg, dat, len, iic_on_done, me)) {
    return ERR_BUSY;
  }
  me->tx_busy = 1;
  if (comp == IO_SYNC) {
    while (me->tx_busy) {}
    return (me->last_ec == 0) ? ERR_OK : ERR_FAILED;
  }
  return ERR_OK;
}
