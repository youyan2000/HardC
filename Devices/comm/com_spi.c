// SPI 传输类实现 —— CommBase 子类 (Devices/comm, 阶段1 重构)
//
// 传输语义: 全双工 + 寄存器. CS 经 bsp_gpio 管理, 不再直接碰 HAL GPIO.
// 阻塞 HAL 传输 (100ms 超时) 只允许在 MAIN 上下文调用 — comm 线本就是第三优先级,
//   事务短 (寄存器级), 忙等可接受; comp 参数为接口统一而保留, 阻塞下 IO_SYNC 天然成立.

#include "com_spi.h"
#include "container_of.h"

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

// CS 拉低 (使能从设备)
static void cs_low(Spi *me) {
  bsp_gpio_write(&me->cs, false);
}

// CS 拉高 (释放从设备)
static void cs_high(Spi *me) {
  bsp_gpio_write(&me->cs, true);
}

// 自检: 句柄已绑定且已初始化
static int self_check_impl(CommBase *base) {
  Spi *me = container_of(base, Spi, base);
  if (me->hspi == NULL || base->inited == 0) {
    return -1;
  }
  return 0;
}

// 诊断虚表 — 数据面 (句柄/CS) 不进虚表
static const CommOps spi_ops = {
    .self_check = self_check_impl,
    .reset = NULL,
};

// -------- 构造 / 析构 / 配置 --------

// 初始化: 契约身份 + 句柄/CS + CS 配置为输出并拉高 (空闲)
void spi_init(Spi *me, const SpiConfig *cfg) {
  comm_base_init(&me->base, "spi");
  me->hspi = cfg->hspi;
  me->cs = cfg->cs;
  me->completion = IO_ASYNC_FLAG;
  me->base.ops = &spi_ops;
  bsp_gpio_cfg_output(&me->cs);
  bsp_gpio_write(&me->cs, true);
}

// 重配: 换句柄/CS (不改变契约身份)
void spi_set_config(Spi *me, const SpiConfig *cfg) {
  me->hspi = cfg->hspi;
  me->cs = cfg->cs;
  bsp_gpio_cfg_output(&me->cs);
  bsp_gpio_write(&me->cs, true);
}

// 反初始化: CS 拉高释放 + 清状态
void spi_deinit(Spi *me) {
  bsp_gpio_write(&me->cs, true);
  me->base.ops = NULL;
  me->hspi = NULL;
  comm_base_deinit(&me->base);
}

// -------- 数据操作 (MAIN 上下文, 阻塞) --------

// 写: CS 低→发送→CS 高
ErrorCode spi_write(Spi *me, CommConstData data, IoCompletion comp) {
  (void) comp;  // 阻塞传输, 无需异步完成处理
  cs_low(me);
  HAL_StatusTypeDef st = HAL_SPI_Transmit(me->hspi, (uint8_t *) data.ptr, data.len, 100);
  cs_high(me);
  return hal_to_ec(st);
}

// 读: CS 低→接收→CS 高
ErrorCode spi_read(Spi *me, CommData data, IoCompletion comp) {
  (void) comp;
  cs_low(me);
  HAL_StatusTypeDef st = HAL_SPI_Receive(me->hspi, data.ptr, data.len, 100);
  cs_high(me);
  return hal_to_ec(st);
}

// 全双工: 同时收发 (rx.len 需 ≥ tx.len)
ErrorCode spi_transfer(Spi *me, CommConstData tx, CommData rx, IoCompletion comp) {
  (void) comp;
  if (rx.len < tx.len) {
    return ERR_SIZE;
  }
  cs_low(me);
  HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(me->hspi, (uint8_t *) tx.ptr, rx.ptr, tx.len, 100);
  cs_high(me);
  return hal_to_ec(st);
}

// 寄存器读: 发 reg 地址 → 收 len 字节
ErrorCode spi_read_reg(Spi *me, uint8_t reg, uint8_t *dat, uint16_t len, IoCompletion comp) {
  (void) comp;
  cs_low(me);
  HAL_StatusTypeDef st = HAL_SPI_Transmit(me->hspi, &reg, 1, 100);
  if (st == HAL_OK) {
    st = HAL_SPI_Receive(me->hspi, dat, len, 100);
  }
  cs_high(me);
  return hal_to_ec(st);
}
