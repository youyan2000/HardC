// I2C 通信驱动 —— CommBase 子类实现
// 主模式 I2C 封装: send(阻塞写入) / bgn(中断接收) / read(读当前字节)
// 设备地址在 init 时绑定，所有操作使用同一个地址
//
// 扩展 API: iic_read_reg — 写寄存器地址后读 N 字节

#include "com_i2c.h"
#include "container_of.h"

// -------- ops 实现 --------

// 阻塞写入: HAL_I2C_Master_Transmit(dev_addr, dat, len, timeout)
static void send_impl(CommBase *base, const uint8_t *dat, uint16_t len) {
  Iic *me = container_of(base, Iic, base);
  HAL_I2C_Master_Transmit(me->hi2c, me->dev_addr,
                          (uint8_t *)dat, len, 100);
}

// 启动单字节中断接收: HAL_I2C_Master_Receive_IT(dev_addr, &cur, 1)
static void bgn_impl(CommBase *base) {
  Iic *me = container_of(base, Iic, base);
  HAL_I2C_Master_Receive_IT(me->hi2c, me->dev_addr,
                            &base->cur, 1);
}

// 读取当前接收到的字节
static uint8_t read_impl(CommBase *base) {
  return base->cur;
}

// I2C 虚函数表
static const CommOps iic_ops = {
  .send = send_impl,
  .bgn  = bgn_impl,
  .read = read_impl,
};

// -------- 构造 / 析构 --------

// 初始化 I2C 驱动: 调基类构造 → 绑定 HAL 句柄+设备地址 → 注册 ops
void iic_init(Iic *me, CommName name, I2C_HandleTypeDef *hi2c, uint8_t dev_addr) {
  comm_base_init(&me->base);
  me->base.name = name;
  me->hi2c      = hi2c;
  me->dev_addr  = dev_addr;
  me->base.ops  = &iic_ops;
}

// 反初始化: 清空 ops 和 HAL 句柄
void iic_deinit(Iic *me) {
  me->base.ops = NULL;
  me->hi2c     = NULL;
}

// -------- 扩展 API: 寄存器读 --------

// 先写寄存器地址 (1字节) → 再读 N 字节到 dst
// 返回值: 0=成功, 非0=HAL 错误码
uint8_t iic_read_reg(Iic *me, uint8_t reg, uint8_t *dst, uint8_t len) {
  HAL_StatusTypeDef st;
  st = HAL_I2C_Master_Transmit(me->hi2c, me->dev_addr,
                               &reg, 1, 100);
  if (st != HAL_OK) return st;
  st = HAL_I2C_Master_Receive(me->hi2c, me->dev_addr,
                              dst, len, 100);
  return st;
}
