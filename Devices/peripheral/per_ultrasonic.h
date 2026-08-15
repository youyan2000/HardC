#ifndef PER_ULTRASONIC_H
#define PER_ULTRASONIC_H

// 超声波测距外设 — SensorBase 子类, UART 总线轮询模型
//
// 迁移: 原 Devices/comm/_legacy/com_ultrasonic.h/c (阶段2 peripheral 域收编)
// 改造:
//   - CommBase 虚表 (send/bgn/read) → Uart* 总线直调 (uart_write 触发 / uart_read 收环)
//   - HAL_UART_Receive_IT 逐字节回调 → MAIN 轮询 per_ultrasonic_process 读 SPSC rx 环组帧
//   - 帧格式/校验和/超时/诊断逻辑原样保留 (0xA0 触发, DataH+DataL+CHK, CHK=DataH+DataL)
//
// 测量模型: 不绑 SensorBase.measure (轮询型设备), 由 App 在 CTX_MAIN 每 10ms 调
//   per_ultrasonic_tick (触发/超时) + per_ultrasonic_process (收环组帧).

#include "comp_sensor.h"
#include "com_uart.h"

typedef struct {
  SensorBase base;      // 测量契约 (第一成员)
  Uart *bus;            // UART 总线 (MAIN 轮询读 rx 环)
  uint8_t index;        // 0=空闲, 1-3=接收中
  uint8_t data[4];      // 帧缓冲 (data[1..3] 有效)
  uint16_t dist;        // 最新有效测距 (mm)
  uint16_t prev;        // 前一次测距
  uint16_t ot_cnt;      // 超时计数
  uint16_t chk_err;     // 校验错误计数
  uint16_t timeout_ms;  // 超时, 默认 2000
  uint8_t valid;        // 最新读数有效
} PerUltrasonic;

// 初始化: 绑总线 + 清状态
void per_ultrasonic_init(PerUltrasonic *me, Uart *bus);

// 设置超时 (ms), tick 周期 10ms 时内部转换为 tick 数
void per_ultrasonic_set_timeout(PerUltrasonic *me, uint16_t ms);

// 每 10ms (CTX_MAIN): 空闲发 0xA0 触发测距; 接收中超时强制复位
//   返回 ERR_OK=已触发, ERR_PENDING=等待接收中, ERR_TIMEOUT=本次超时复位
int per_ultrasonic_tick(PerUltrasonic *me);

// 从 bus 的 rx 环读字节组帧 + 校验 (MAIN 轮询)
//   返回 ERR_OK=收满有效帧, ERR_CHECK=校验失败, ERR_EMPTY=环空
int per_ultrasonic_process(PerUltrasonic *me);

// 增强 API
uint16_t per_ultrasonic_get_distance(const PerUltrasonic *me);         // 最新有效距离 (mm)
uint8_t per_ultrasonic_is_valid(const PerUltrasonic *me);              // 最新读数是否有效
uint16_t per_ultrasonic_get_checksum_errors(const PerUltrasonic *me);  // 校验错误计数

#endif  // PER_ULTRASONIC_H
