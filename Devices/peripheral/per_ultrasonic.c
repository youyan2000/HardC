// 超声波测距外设 — SensorBase 子类实现 (UART 总线轮询模型)
// 迁移自 Devices/comm/_legacy/com_ultrasonic.c, 触发/收环改走 Uart 类总线

#include "per_ultrasonic.h"
#include "comp_checksum.h"

// -------- 构造 / 析构 --------

// 初始化: 绑总线 + 清状态 (timeout 默认 2000ms)
void per_ultrasonic_init(PerUltrasonic *me, Uart *bus) {
  sensor_base_init(&me->base, "ultrasonic");
  me->bus = bus;
  me->index = 0;
  me->dist = 0;
  me->prev = 0;
  me->ot_cnt = 0;
  me->chk_err = 0;
  me->timeout_ms = 2000;
  me->valid = 0;
  for (uint8_t i = 0; i < 4; i++) {
    me->data[i] = 0;
  }
  // 轮询型设备不绑 measure (触发/收环在 MAIN 由 tick/process 驱动)
  me->base.measure = NULL;
}

// 设置超时 (ms)
void per_ultrasonic_set_timeout(PerUltrasonic *me, uint16_t ms) {
  me->timeout_ms = ms;
}

// -------- 定时调用: 触发新测距 / 超时检测 --------

// 每 10ms: 空闲发 0xA0 触发测距; 等待接收中超时强制复位
int per_ultrasonic_tick(PerUltrasonic *me) {
  if (me->index == 0) {
    // 空闲 → 发测距指令, 进入接收
    me->ot_cnt = 0;
    uint8_t cmd = 0xa0;
    CommConstData d = {.ptr = &cmd, .len = 1};
    uart_write(me->bus, d, IO_NONE);
    me->index = 1;
    return ERR_OK;
  }
  // 等待接收中 → 超时后强制复位
  me->ot_cnt++;
  uint16_t threshold = me->timeout_ms / 10;  // tick 周期 10ms
  if (me->ot_cnt >= threshold) {
    me->index = 0;
    me->ot_cnt = 0;
    me->valid = 0;  // 超时标记无效
    return ERR_TIMEOUT;
  }
  return ERR_PENDING;
}

// -------- 串口轮询: 从 rx 环读字节组帧 + 校验 --------

// MAIN 轮询: 读出环上所有可用字节, 组装 3 字节回复帧 (DataH+DataL+CHK)
int per_ultrasonic_process(PerUltrasonic *me) {
  uint8_t byte;
  CommData d = {.ptr = &byte, .len = 1};
  while (uart_read(me->bus, &d, IO_NONE) == ERR_OK) {
    me->data[me->index] = byte;
    if (me->index >= 3) {
      // 收满 3 字节 → data[1]=DataH, data[2]=DataL, data[3]=CHK
      uint8_t chk = math_sum_u8(&me->data[1], 2);  // 校验和: DataH + DataL (低 8 位)
      if (chk != me->data[3]) {
        // 校验失败 → 丢弃, 计数, 标记无效
        me->chk_err++;
        me->valid = 0;
        me->index = 0;
        return ERR_CHECK;
      }
      // 校验通过 → 组装距离值
      me->prev = (uint16_t) (((uint16_t) me->data[1] << 8) | me->data[2]);
      if (me->prev <= 4500) {
        me->dist = me->prev;  // 合理范围才更新有效值 (超声波最大约 4.5m)
        me->valid = 1;
      } else {
        me->valid = 0;  // 超出范围标记无效
      }
      me->index = 0;  // 回到空闲, 下一 tick 自动触发新测距
      return ERR_OK;
    }
    me->index++;
    d.len = 1;  // uart_read 写回实际读出数, 需复位
  }
  return ERR_EMPTY;  // 环空
}

// -------- 增强 API --------

// 返回最新有效距离 (mm), 调用前应先 per_ultrasonic_is_valid 确认
uint16_t per_ultrasonic_get_distance(const PerUltrasonic *me) {
  return me->dist;
}

// 最新读数是否通过校验和与量程检查
uint8_t per_ultrasonic_is_valid(const PerUltrasonic *me) {
  return me->valid;
}

// 返回校验和错误累计次数 (可用于诊断链路质量)
uint16_t per_ultrasonic_get_checksum_errors(const PerUltrasonic *me) {
  return me->chk_err;
}
