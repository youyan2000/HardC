// 通信基类实现 —— CommBase 父类的构造/析构/诊断
//
// 提供统一基类契约: 绑定契约身份 (name) + inited 标志、诊断分派 (self_check/reset)
// 传输子类 (com_uart/spi/i2c/can/gpio) 在 init 时绑定自身 static ops

#include "comp_comm.h"
#include <stddef.h>

// 基类构造 —— 绑定契约身份 (name), inited 置 1, ops 由子类 init 时绑定
void comm_base_init(CommBase *me, const char *name) {
  me->name = name;
  me->inited = 1;
  me->ops = NULL;
}

// 基类析构 —— 清空 name/inited/ops
void comm_base_deinit(CommBase *me) {
  me->name = NULL;
  me->inited = 0;
  me->ops = NULL;
}

// 诊断分发: 自检 — ops 或 self_check 为空 → 返回 0 (通过)
int comm_self_check(CommBase *me) {
  if (me->ops && me->ops->self_check) {
    return me->ops->self_check(me);
  }
  return 0;
}

// 诊断分发: 复位 — ops 或 reset 为空 → 忽略
void comm_reset(CommBase *me) {
  if (me->ops && me->ops->reset) {
    me->ops->reset(me);
  }
}
