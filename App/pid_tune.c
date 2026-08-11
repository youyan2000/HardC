// 串口 PID 调参协议 —— 帧接收 + 校验 + 回调分发
//
// 帧格式: 0xFB + 48 字节 PidTuneFrame
//   校验: 帧尾 4 字节 float == π (3.1415927f)
//   校验通过 → pid_tune_pending = true
//   主循环 pid_tune_respond() → 调 apply 回调 → printf 响应

#include "pid_tune.h"
#include <stdio.h>

// ======== 内部状态 ========

static PidTuneApplyFn apply_cb = NULL;

static uint8_t  rx_st  = 0;    // 0=等 0xFB, 1=收数据
static uint8_t  rx_buf[PID_TUNE_FRAME_SIZE];
static uint16_t rx_idx = 0;

volatile bool pid_tune_pending = false;
static volatile bool tune_fail = false;

// ======== API 实现 ========

void pid_tune_set_apply_cb(PidTuneApplyFn cb) {
  apply_cb = cb;
}

// ISR: 状态机接收
bool pid_tune_rx(uint8_t byte) {
  if (rx_st == 0) {
    if (byte == 0xFB) {
      rx_st  = 1;
      rx_idx = 0;
      return true;                // 消费前缀
    }
    return false;                 // 不是 0xFB, 交给下游
  }

  rx_buf[rx_idx++] = byte;
  if (rx_idx >= PID_TUNE_FRAME_SIZE) {
    rx_st = 0;

    // 校验: 帧尾 4 字节 == π
    const PidTuneFrame *f = (const PidTuneFrame *)rx_buf;
    if (f->CHECK == PID_TUNE_CHECK_CODE) {
      pid_tune_pending = true;
      tune_fail = false;
    } else {
      tune_fail = true;
    }
  }
  return true;                    // 已消费
}

// 主循环: 应用配置 + 响应
void pid_tune_respond(void) {
  if (!pid_tune_pending && !tune_fail) return;

  if (pid_tune_pending) {
    const PidTuneFrame *f = (const PidTuneFrame *)rx_buf;

    // 调用户回调: 槽位 → PID 实例
    if (apply_cb) {
      apply_cb(f->Coef);
    }

    printf("\r\nCONFIG OK\r\n");
    pid_tune_pending = false;
  }

  if (tune_fail) {
    printf("\r\nCONFIG FAIL\r\n");
    tune_fail = false;
  }
}
