#ifndef PID_TUNE_H
#define PID_TUNE_H

// 串口 PID 调参协议 —— 独立模块, 不依赖任何 PID 类型
//
// 使用模式 (参考 SmCar 0xFB 协议):
//
//   项目初始化:
//     pid_tune_set_apply_cb(my_apply);   // 注册回调: 槽位 → 你的 PID 实例
//
//   ISR (USART 每字节):
//     if (pid_tune_rx(byte)) return;     // 消费调参帧
//
//   主循环 (BackgroundTask):
//     pid_tune_respond();                // 校验通过 → 调回调 → printf 响应

#include <stdint.h>
#include <stdbool.h>

// ======== 帧格式 ========

#define PID_TUNE_FRAME_SIZE  48          // 总字节数
#define PID_TUNE_COEF_COUNT  10          // 参数槽位个数
#define PID_TUNE_CHECK_CODE  3.1415927f  // π, 帧尾校验

// 槽位映射 (与调用者约定, 库不写死):
//   [0] kp_outer  [1] ki_outer  [2] kd_outer
//   [3] kp_inner  [4] ki_inner  [5] kd_inner
//   [6] kpp       [7] kp_p2pd   [8] kd_p2pd
//   [9] 预留

typedef struct __attribute__((packed)) {
  uint8_t  HEAD;                          // [0]  预留 0x00
  uint8_t  Command;                       // [1]  预留 0x14
  float    Coef[PID_TUNE_COEF_COUNT];     // [2-41] PID 参数槽位
  float    CHECK;                         // [42-45] 校验: 须 == 3.1415927f
  uint16_t reserved;                      // [46-47] 对齐到 48 字节
} PidTuneFrame;

// ======== 回调类型 ========

// 收到有效帧后调用: 把 coef[] 写入你的 PID 实例
typedef void (*PidTuneApplyFn)(const float coef[PID_TUNE_COEF_COUNT]);

// ======== API ========

// 注册应用回调 (在项目初始化时调用一次)
void pid_tune_set_apply_cb(PidTuneApplyFn cb);

// ISR: 每收到一个字节喂入, 返回 true 表示已消费 (调用者应 return)
bool pid_tune_rx(uint8_t byte);

// 主循环: 校验通过 → 调 apply 回调 → printf "CONFIG OK/FAIL"
void pid_tune_respond(void);

// 帧校验通过后置位 (主循环轮询用, 也可直接用 pid_tune_respond)
extern volatile bool pid_tune_pending;

#endif
