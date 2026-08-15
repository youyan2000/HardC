// I2C 传输类实现 —— CommBase 子类 (Devices/comm, 阶段1 重构)
//
// 双模式:
//   IIC_HW — HAL_I2C_Mem_Write/Mem_Read 阻塞 (100ms 超时), 寄存器寻址模式
//   IIC_SW — 软件 bit-bang 标准 I2C 协议 (START/STOP/ACK/NAK), 纯 C 走 bsp_gpio, host 可测
//
// SW 说明: SCL 固定推挽输出, SDA 在发送时输出、读 ACK/数据时切输入释放
//   (依赖外部上拉, 标准 I2C 要求). 时序间隔由简单循环给出, SCL 频限由调用速率决定.

#include "com_i2c.h"

// 从机无 ACK 重试上限 (SW 模式, 防死锁)
#define I2C_SW_MAX_RETRY 1000u

// HAL 状态 → ErrorCode 映射
static ErrorCode hal_to_ec(HAL_StatusTypeDef st) {
  switch (st) {
  case HAL_OK:
    return ERR_OK;
  case HAL_TIMEOUT:
    return ERR_TIMEOUT;
  case HAL_BUSY:
    return ERR_BUSY;
  default:
    return ERR_FAILED;
  }
}

// -------- SW bit-bang 时序原语 --------

// 最小时序间隔 — SCL 频限由调用速率决定
static void i2c_sw_delay(Iic *me) {
  (void) me;
  for (volatile int i = 0; i < 8; i++) {}
}

// SCL 拉高/拉低 (带时序间隔)
static void i2c_sw_scl_hi(Iic *me) {
  bsp_gpio_write(&me->scl, true);
  i2c_sw_delay(me);
}

static void i2c_sw_scl_lo(Iic *me) {
  bsp_gpio_write(&me->scl, false);
  i2c_sw_delay(me);
}

// SDA 输出电平 (带时序间隔)
static void i2c_sw_sda_out(Iic *me, bool v) {
  bsp_gpio_write(&me->sda, v);
  i2c_sw_delay(me);
}

// START: SCL 高时 SDA 拉低
static void i2c_sw_start(Iic *me) {
  bsp_gpio_cfg_output(&me->scl);
  bsp_gpio_cfg_output(&me->sda);
  i2c_sw_sda_out(me, true);
  i2c_sw_scl_hi(me);
  i2c_sw_sda_out(me, false);
  i2c_sw_scl_lo(me);
}

// STOP: SCL 高时 SDA 拉高
static void i2c_sw_stop(Iic *me) {
  bsp_gpio_cfg_output(&me->sda);
  i2c_sw_sda_out(me, false);
  i2c_sw_scl_hi(me);
  i2c_sw_sda_out(me, true);
}

// 读 ACK: 释放 SDA → 第 9 个时钟高电平采样 (ACK=SDA 低)
static bool i2c_sw_read_ack(Iic *me) {
  bsp_gpio_cfg_input(&me->sda, BSP_GPIO_PULL_NONE);  // 释放 SDA, 依赖外部上拉
  i2c_sw_scl_hi(me);
  bool ack = !bsp_gpio_read(&me->sda);
  i2c_sw_scl_lo(me);
  bsp_gpio_cfg_output(&me->sda);  // 恢复输出
  return ack;
}

// 发送一字节 (MSB 先出), 返回从机 ACK
static bool i2c_sw_put_byte(Iic *me, uint8_t byte) {
  bsp_gpio_cfg_output(&me->sda);
  for (int i = 7; i >= 0; i--) {
    bool bit = (byte >> i) & 1u;
    i2c_sw_scl_lo(me);
    i2c_sw_sda_out(me, bit);  // SDA 就绪后 SCL 上升沿采样
    i2c_sw_scl_hi(me);
  }
  i2c_sw_scl_lo(me);
  return i2c_sw_read_ack(me);
}

// 发送一字节并重试: 无 ACK 则 STOP→START 再试 (从机忙场景)
static bool i2c_sw_put_byte_retry(Iic *me, uint8_t byte) {
  for (uint32_t i = 0; i < I2C_SW_MAX_RETRY; i++) {
    if (i2c_sw_put_byte(me, byte)) {
      return true;
    }
    i2c_sw_stop(me);
    i2c_sw_start(me);
  }
  return false;
}

// 接收一字节 (MSB 先出), ack=true 发 ACK / false 发 NAK (最后一字节)
static uint8_t i2c_sw_get_byte(Iic *me, bool ack) {
  uint8_t byte = 0;
  bsp_gpio_cfg_input(&me->sda, BSP_GPIO_PULL_NONE);
  for (int i = 7; i >= 0; i--) {
    i2c_sw_scl_hi(me);
    byte = (uint8_t) ((byte << 1) | (bsp_gpio_read(&me->sda) ? 1u : 0u));
    i2c_sw_scl_lo(me);
  }
  bsp_gpio_cfg_output(&me->sda);
  i2c_sw_sda_out(me, !ack);  // 应答位: ACK=拉低, NAK=释放
  i2c_sw_scl_hi(me);
  i2c_sw_scl_lo(me);
  return byte;
}

// -------- SW 寄存器事务 --------

// SW 写寄存器: START + 写方向地址 + reg + 数据... + STOP
static ErrorCode iic_sw_write_reg(Iic *me, uint8_t reg, const uint8_t *dat, uint16_t len) {
#define I2C_WRITE ((uint8_t) ((me->dev_addr << 1) | 0u))
  i2c_sw_start(me);
  if (!i2c_sw_put_byte_retry(me, I2C_WRITE)) {
    i2c_sw_stop(me);
    return ERR_TIMEOUT;
  }
  if (!i2c_sw_put_byte(me, reg)) {
    i2c_sw_stop(me);
    return ERR_TIMEOUT;
  }
  for (uint16_t i = 0; i < len; i++) {
    if (!i2c_sw_put_byte(me, dat[i])) {
      i2c_sw_stop(me);
      return ERR_TIMEOUT;
    }
  }
  i2c_sw_stop(me);
  return ERR_OK;
#undef I2C_WRITE
}

// SW 读寄存器: 写方向写 reg → 重复 START → 读方向收 len 字节 → STOP
static ErrorCode iic_sw_read_reg(Iic *me, uint8_t reg, uint8_t *dat, uint16_t len) {
#define I2C_WRITE ((uint8_t) ((me->dev_addr << 1) | 0u))
#define I2C_READ ((uint8_t) ((me->dev_addr << 1) | 1u))
  i2c_sw_start(me);
  if (!i2c_sw_put_byte_retry(me, I2C_WRITE)) {
    i2c_sw_stop(me);
    return ERR_TIMEOUT;
  }
  if (!i2c_sw_put_byte(me, reg)) {
    i2c_sw_stop(me);
    return ERR_TIMEOUT;
  }
  i2c_sw_start(me);  // 重复 START, 切换读方向
  if (!i2c_sw_put_byte_retry(me, I2C_READ)) {
    i2c_sw_stop(me);
    return ERR_TIMEOUT;
  }
  for (uint16_t i = 0; i < len; i++) {
    dat[i] = i2c_sw_get_byte(me, i < (uint16_t) (len - 1u));  // 最后一字节 NAK
  }
  i2c_sw_stop(me);
  return ERR_OK;
#undef I2C_WRITE
#undef I2C_READ
}

// -------- HW 寄存器事务 --------

// HW 写寄存器: HAL_I2C_Mem_Write (7-bit 地址左移 1 位)
static ErrorCode iic_hw_write_reg(Iic *me, uint8_t reg, const uint8_t *dat, uint16_t len) {
  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Write(me->hi2c, (uint16_t) (me->dev_addr << 1), reg, I2C_MEMSIZE_8BIT, (uint8_t *) dat, len, 100);
  return hal_to_ec(st);
}

// HW 读寄存器: HAL_I2C_Mem_Read
static ErrorCode iic_hw_read_reg(Iic *me, uint8_t reg, uint8_t *dat, uint16_t len) {
  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Read(me->hi2c, (uint16_t) (me->dev_addr << 1), reg, I2C_MEMSIZE_8BIT, dat, len, 100);
  return hal_to_ec(st);
}

// -------- 构造 / 析构 --------

// HW 模式初始化
void iic_init_hw(Iic *me, I2C_HandleTypeDef *hi2c, uint8_t dev_addr) {
  comm_base_init(&me->base, "i2c");
  me->mode = IIC_HW;
  me->hi2c = hi2c;
  me->dev_addr = dev_addr;
  me->completion = IO_ASYNC_FLAG;
}

// SW 模式初始化: SCL 固定输出, SDA 按事务切换输入/输出
void iic_init_sw(Iic *me, BspGpioPin scl, BspGpioPin sda, uint8_t dev_addr) {
  comm_base_init(&me->base, "i2c");
  me->mode = IIC_SW;
  me->scl = scl;
  me->sda = sda;
  me->dev_addr = dev_addr;
  me->completion = IO_ASYNC_FLAG;
  bsp_gpio_cfg_output(&me->scl);
}

// 反初始化: 清模式/句柄/地址
void iic_deinit(Iic *me) {
  me->mode = IIC_HW;
  me->hi2c = NULL;
  me->dev_addr = 0;
  comm_base_deinit(&me->base);
}

// -------- 公共寄存器事务 (按模式分发) --------

// 写寄存器 → 写 len 字节 (阻塞事务, IO_SYNC 天然成立)
ErrorCode iic_write_reg(Iic *me, uint8_t reg, const uint8_t *dat, uint16_t len, IoCompletion comp) {
  (void) comp;  // 阻塞事务, 无需异步完成处理
  if (me->mode == IIC_HW) {
    return iic_hw_write_reg(me, reg, dat, len);
  }
  return iic_sw_write_reg(me, reg, dat, len);
}

// 写寄存器 → 读 len 字节 (阻塞事务)
ErrorCode iic_read_reg(Iic *me, uint8_t reg, uint8_t *dat, uint16_t len, IoCompletion comp) {
  (void) comp;
  if (me->mode == IIC_HW) {
    return iic_hw_read_reg(me, reg, dat, len);
  }
  return iic_sw_read_reg(me, reg, dat, len);
}
