#ifndef COM_ULTRASONIC_H
#define COM_ULTRASONIC_H

// 超声波测距驱动 —— CommBase 子类
// 通过 USART3 发 0xA0 触发测距, 收 DataH+DataL+CHK, 16-bit 毫米值
// ultrasonic_tick: TIM3 ISR 中每 10ms 调用, 管理触发+超时
// ultrasonic_process: USART3 RX 回调中调用, 处理收到字节
//
// LitteCar 增强 (2026-07):
// - 校验和验证 (CHK = DataH + DataL), 拒绝坏帧
// - 可配置超时 (timeout_ms), 替代硬编码 200 tick
// - 诊断计数器: chk_err (校验错误), ot_cnt (超时次数)
// - valid 标志指示最新读数是否通过所有检查

#include "comp_comm.h"
#include "stm32f1xx_hal.h"
#include <stdbool.h>

typedef struct {
  CommBase           base;       // 基类 (必须第一个)
  UART_HandleTypeDef *huart;     // HAL UART3 句柄
  uint8_t            index;      // 0=空闲, 1-3=接收中
  uint8_t            data[4];    // 接收缓冲 (data[1..3] 有效)
  uint16_t           dist;       // 最新有效测距值 (mm)
  uint16_t           prev;       // 前一次原始测距值
  uint16_t           ot_cnt;     // 超时计数 (每 tick+1)
  uint16_t           chk_err;    // 校验和错误计数
  uint16_t           timeout_ms; // 可配置超时 (ms), 默认 2000
  bool               valid;      // 最新读数通过所有检查
} Ultrasonic;

void ultrasonic_init(Ultrasonic *me, CommName name, UART_HandleTypeDef *huart);
void ultrasonic_deinit(Ultrasonic *me);
void ultrasonic_tick(Ultrasonic *me);     // TIM3 ISR 中调用 (每 10ms)
void ultrasonic_process(Ultrasonic *me);  // USART3 RX 回调中调用

// LitteCar 增强 API
uint16_t ultrasonic_get_distance       (const Ultrasonic *me);  // 返回最新有效距离
bool     ultrasonic_is_valid           (const Ultrasonic *me);  // 最新读数是否有效
uint16_t ultrasonic_get_checksum_errors(const Ultrasonic *me);  // 校验错误计数
void     ultrasonic_set_timeout        (Ultrasonic *me, uint16_t timeout_ms);  // 设置超时

#endif
