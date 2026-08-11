#include "comp_comm.h"
#include <assert.h>
#include <stddef.h>

// 基类构造 —— 成员清零，ops 由子类 init 时绑定
void comm_base_init(CommBase *me) {
  me->ops = NULL;
  me->buf = NULL;
  me->cur = 0;
}

// 基类析构 —— 清除 ops 和缓冲区
void comm_base_deinit(CommBase *me) {
  me->ops = NULL;
  me->name = 0;
  me->buf = NULL;
  me->cur = 0;
}

// 分发函数: 断言 ops->send 必须存在，然后调用子类实现
void comm_send(CommBase *me, const uint8_t *dat, uint16_t len) {
  assert(me->ops->send);
  me->ops->send(me, dat, len);
}

// 分发函数: 断言 ops->bgn 必须存在，然后调用子类实现
void comm_bgn(CommBase *me) {
  assert(me->ops->bgn);
  me->ops->bgn(me);
}

// 分发函数: 断言 ops->read 必须存在，然后调用子类实现
uint8_t comm_read(CommBase *me) {
  assert(me->ops->read);
  return me->ops->read(me);
}
