// SPI 通信驱动 —— CommBase 子类实现
// 主模式 SPI 封装: send(阻塞发送+CS控制) / bgn(中断接收) / read(读当前字节)
// CS 由驱动自动管理: 发送/接收前拉低, 完成后拉高

#include "com_spi.h"
#include "container_of.h"

// -------- CS 控制辅助 --------

// 拉低 CS（使能从设备）
static void cs_low(Spi *me) {
  HAL_GPIO_WritePin(me->cs_port, me->cs_pin, GPIO_PIN_RESET);
}

// 拉高 CS（释放从设备）
static void cs_high(Spi *me) {
  HAL_GPIO_WritePin(me->cs_port, me->cs_pin, GPIO_PIN_SET);
}

// -------- ops 实现 --------

// 阻塞发送: 拉低 CS → HAL_SPI_Transmit → 拉高 CS
static void send_impl(CommBase *base, const uint8_t *dat, uint16_t len) {
  Spi *me = container_of(base, Spi, base);
  cs_low(me);
  HAL_SPI_Transmit(me->hspi, (uint8_t *)dat, len, 100);
  cs_high(me);
}

// 启动单字节中断接收: 拉低 CS → HAL_SPI_Receive_IT
// CS 在 HAL_SPI_RxCpltCallback 中拉高（应用层负责）
static void bgn_impl(CommBase *base) {
  Spi *me = container_of(base, Spi, base);
  cs_low(me);
  HAL_SPI_Receive_IT(me->hspi, &base->cur, 1);
}

// 读取当前接收到的字节
static uint8_t read_impl(CommBase *base) {
  return base->cur;
}

// SPI 虚函数表
static const CommOps spi_ops = {
  .send = send_impl,
  .bgn  = bgn_impl,
  .read = read_impl,
};

// -------- 构造 / 析构 --------

// 初始化 SPI 驱动: 调基类构造 → 绑定 HAL 句柄+CS引脚 → 注册 ops → CS 默认拉高
void spi_init(Spi *me, CommName name, SPI_HandleTypeDef *hspi,
              GPIO_TypeDef *cs_port, uint16_t cs_pin) {
  comm_base_init(&me->base);
  me->base.name = name;
  me->hspi      = hspi;
  me->cs_port   = cs_port;
  me->cs_pin    = cs_pin;
  me->base.ops  = &spi_ops;
  cs_high(me);  // CS 初始为高（空闲）
}

// 反初始化: 清空 ops → CS 拉高释放 → HAL 句柄置 NULL
void spi_deinit(Spi *me) {
  cs_high(me);
  me->base.ops = NULL;
  me->hspi     = NULL;
}
