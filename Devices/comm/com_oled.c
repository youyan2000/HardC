// OLED 通信驱动 —— CommBase 子类, 薄包装 oled.h 硬件层
// send: 将字节流逐字节写入 OLED (SPI 3 线模式, 每个字节 cmd=DATA)
// bgn/read: OLED 单向输出设备, 不支持读取, 实现为空

#include "com_oled.h"
#include "oled.h"
#include "container_of.h"

// 发送: 将数据逐字节写入 OLED, 每个字节标记为 DATA (非 CMD)
static void send_impl(CommBase *base, const uint8_t *dat, uint16_t len) {
  (void)base;
  for (uint16_t i = 0; i < len; i++)
    oled_write_byte(dat[i], OLED_DATA);
}

// OLED 为单向输出设备, 不实现接收功能
static void bgn_impl(CommBase *base) {
  (void)base;
}

// OLED 为单向输出设备, 无读取功能
static uint8_t read_impl(CommBase *base) {
  (void)base;
  return 0;
}

// OLED 虚函数表
static const CommOps com_oled_ops = {
  .send = send_impl,
  .bgn  = bgn_impl,
  .read = read_impl,
};

// 初始化 OLED 驱动: 调基类构造 → 注册 ops (仅 send 可用, bgn/read 为 stub)
void com_oled_init(ComOled *me, CommName name) {
  comm_base_init(&me->base);
  me->base.name = name;
  me->base.ops  = &com_oled_ops;
}

// -------- 格式化输出 (Module 层入口, 内部委托 BSP oled.h) --------

void com_oled_show_string(ComOled *me, uint8_t x, uint8_t y,
                          const char *str, uint8_t len, uint8_t size) {
  (void)me;
  oled_show_string(x, y, str, len, size);
}

void com_oled_show_num(ComOled *me, uint8_t x, uint8_t y,
                       uint32_t num, uint8_t len, uint8_t size) {
  (void)me;
  oled_show_num(x, y, num, len, size);
}
