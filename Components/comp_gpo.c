#include "comp_gpo.h"
#include <stddef.h>

// 嵌入式平台断言 —— 可根据目标平台替换为 assert() 或自定义实现
// STM32 环境建议使用 assert_param() 或配置为 while(1) 死循环
#ifndef GPO_ASSERT
  #ifdef USE_STM32_ASSERT
    #include "assert.h"
    #define GPO_ASSERT(expr) assert(expr)
  #else
    #define GPO_ASSERT(expr) ((void)0)
  #endif
#endif

// 基类构造 —— 绑定名称，ops 由子类 init 时覆盖
void gpo_base_init(GpoBase *me, GpoName name) {
  me->ops  = NULL;
  me->name = name;
}

// 基类析构 —— 清除 ops 指针
void gpo_base_deinit(GpoBase *me) {
  me->ops  = NULL;
  me->name = 0;
}

// 分发: 打开输出 —— 断言 on 必须存在 → 委托子类
void gpo_on(GpoBase *me) {
  GPO_ASSERT(me->ops != NULL);
  GPO_ASSERT(me->ops->on != NULL);
  me->ops->on(me);
}

// 分发: 关闭输出 —— 断言 off 必须存在 → 委托子类
void gpo_off(GpoBase *me) {
  GPO_ASSERT(me->ops != NULL);
  GPO_ASSERT(me->ops->off != NULL);
  me->ops->off(me);
}

// 分发: 设置占空比 —— 可选操作，NULL 时静默跳过
void gpo_set(GpoBase *me, uint32_t duty) {
  if (me->ops != NULL && me->ops->set != NULL) {
    me->ops->set(me, duty);
  }
}

// 翻转: 优先调用子类的 toggle 实现，无则退化为 gpo_on
void gpo_toggle(GpoBase *me) {
  if (me->ops != NULL && me->ops->toggle != NULL) {
    me->ops->toggle(me);
  } else {
    // 退化: 无状态感知时只能打开
    gpo_on(me);
  }
}
