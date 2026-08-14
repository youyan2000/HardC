// I/O 完成契约 — 绑定在 I/O 发起时的完成行为 (纯C, 枚举)
//
// 来源: C-OOP 运行时契约, 对应 LibXR 的 ErrorCode 返回约定 (所有 I/O 调用定完成方式, 不许悄悄丢掉)
//
// 每个异步 I/O 发起点必须在调用时声明完成方式, 禁止隐含默认:
//   IO_ASYNC_CB   — 异步, 完成后回调 (调用者注册完成回调)
//   IO_ASYNC_FLAG — 异步, 完成后置标志 (消费者轮询检查)
//   IO_SYNC       — 同步阻塞等待完成 (仅允许 CTX_MAIN)
//   IO_NONE       — 显式忽略完成事件 (fire-and-forget)
//
// 对应 agent.md §1.2 "I/O 完成契约": 完成行为在发起时绑定,
// 消费者在正确的上下文收取完成 (回调在 ISR, 标志在主循环, 阻塞只在主循环)。

#ifndef COMP_IO_H
#define COMP_IO_H

typedef enum {
  IO_ASYNC_CB = 0,    // 异步完成 → 回调 (ISR / 轮询驱动)
  IO_ASYNC_FLAG = 1,  // 异步完成 → 置标志 (消费者轮询)
  IO_SYNC = 2,        // 同步阻塞等待 (仅 CTX_MAIN)
  IO_NONE = 3         // 显式忽略完成
} IoCompletion;

#endif  // COMP_IO_H
