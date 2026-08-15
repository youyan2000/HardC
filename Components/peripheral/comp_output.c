// 输出设备基类实现 —— OutputBase 父类的构造/析构
//
// 提供统一基类契约: 绑定名称 + inited, 派生设备 (per_led/laser/beep/buzzer/fan) 继承
// 并绑定各自 static ops (on/off/toggle/set 等)

#include "comp_output.h"
#include <stddef.h>

// 嵌入式平台断言 — 默认 fail-safe 挂起 (不静默返回后解引用空指针崩溃)
// 行为: 条件为假 → for(;;) 无限循环挂起等待 (传统: 触发即死循环)
// 可选 USE_STM32_ASSERT → assert() (debug 打印断言信息; release NDEBUG 下编译掉, 不再挂起)
// 原 comp_gpo.c 默认 ((void)0) 静默, 释放版 ops 空会直接空指针解引用 — 本版改为挂起等待
#ifndef OUTPUT_ASSERT
#ifdef USE_STM32_ASSERT
#include <assert.h>
#define OUTPUT_ASSERT(expr) assert(expr)
#else
#define OUTPUT_ASSERT(expr) \
  do {                      \
    if (!(expr)) {          \
      for (;;) {}           \
    }                       \
  } while (0)
#endif
#endif

// 基类构造 — 绑定名称, ops 由子类 init 时覆盖
void output_base_init(OutputBase *me, const char *name) {
  me->ops = NULL;
  me->name = name;
}

// 基类析构 — 清除 ops 指针
void output_base_deinit(OutputBase *me) {
  me->ops = NULL;
  me->name = NULL;
}

// 分发: 打开输出 — 断言 on 必须存在 → 委托子类
void output_on(OutputBase *me) {
  OUTPUT_ASSERT(me->ops != NULL);
  OUTPUT_ASSERT(me->ops->on != NULL);
  me->ops->on(me);
}

// 分发: 关闭输出 — 断言 off 必须存在 → 委托子类
void output_off(OutputBase *me) {
  OUTPUT_ASSERT(me->ops != NULL);
  OUTPUT_ASSERT(me->ops->off != NULL);
  me->ops->off(me);
}

// 分发: 设置电平 — 可选操作, NULL 时静默跳过
void output_set(OutputBase *me, uint32_t level) {
  if (me->ops != NULL && me->ops->set != NULL) {
    me->ops->set(me, level);
  }
}

// 翻转: 优先调用子类 toggle 实现, 无则退化为 on
void output_toggle(OutputBase *me) {
  if (me->ops != NULL && me->ops->toggle != NULL) {
    me->ops->toggle(me);
  } else {
    // 退化: 无状态感知时只能打开
    output_on(me);
  }
}
