// 命令邮箱 — MAIN→FAST 命令/参数交接, 周期边界生效 (纯C, 零 malloc, 无锁)
//
// 来源: LibXR (bsp-dev-c/Jiu-xiao) src/middleware/message/ — Topic<T> + QueuedSubscriber (队列缓冲)
// 差异: LibXR QueuedSubscriber 用 SPSCQueue 排队全部命令; 本单槽实现刻意取"最新命令生效"(控制回路设定值覆盖语义)。
//       需排队(FIFO 不丢命令)的场景用 comp_ring.h (SPSC)。
//
// 用途: MAIN 下发命令/参数, FAST 在每个控制周期边界取走并应用
//   - 生产者 (MAIN) 调用 mailbox_post — 后写覆盖先写, 最新命令生效
//   - 消费者 (FAST) 调用 mailbox_poll — 每周期只调一次
//   - 单槽: 投递速率超过消费速率时新命令覆盖旧命令, 匹配"参数/命令"语义
//
// 机制:
//   - MAIN 先写 arg/cmd, 最后置 pending; FAST 先读 pending
//     全序 + 单一抢占源下, 生产者完整写序列对消费者原子可见 (见 agent.md §1.2)
//
// 使用示例 (MAIN 下发电压指令 → FAST 每周期读取):
//   Mailbox cmd_mb;
//   mailbox_init(&cmd_mb);
//   // MAIN: mailbox_post(&cmd_mb, CMD_SET_VREF, 12.0f);
//   // FAST: uint32_t c; float a;
//   //       if (mailbox_poll(&cmd_mb, &c, &a)) apply_cmd(c, a);

#ifndef COMP_MAILBOX_H
#define COMP_MAILBOX_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
  volatile uint32_t cmd;     // 命令码 (0 = 空槽)
  volatile float arg;        // 命令参数
  volatile uint8_t pending;  // 1 = 有命令待取
} Mailbox;

// 初始化: 清空槽位
static inline void mailbox_init(Mailbox *me) {
  me->cmd = 0u;
  me->arg = 0.0f;
  me->pending = 0u;
}

// 生产者 (MAIN): 投递命令 — 后写覆盖先写 (最新命令生效)
static inline void mailbox_post(Mailbox *me, uint32_t cmd, float arg) {
  me->arg = arg;
  me->cmd = cmd;
  me->pending = 1u;
}

// 消费者 (FAST): 周期边界取走命令 — 有命令返回 true 并输出 cmd/arg
//   每周期只调一次; 返回 false 表示无新命令 (保持上一周期行为)
static inline bool mailbox_poll(Mailbox *me, uint32_t *cmd, float *arg) {
  if (me->pending == 0u) {
    return false;
  }
  *cmd = me->cmd;
  *arg = me->arg;
  me->pending = 0u;
  return true;
}

// 消费者: 丢弃待取命令 (如进入故障态时)
static inline void mailbox_clear(Mailbox *me) {
  me->pending = 0u;
  me->cmd = 0u;
}

#endif  // COMP_MAILBOX_H
