// 按键 GPIO 驱动 —— CommBase 子类实现
// 提供双击状态机: IDLE → PRESS → WAIT → 单击/双击/长按事件
// key_tick 每 10ms 调用一次, key_event 读取并清除事件

#include "com_key.h"
#include "container_of.h"

// 按键为纯输入设备, 不支持发送
static void send_impl(CommBase *base, const uint8_t *dat, uint16_t len) {
  (void)base; (void)dat; (void)len;
}

// 读取 GPIO 当前电平, 存入 base.cur 供 key_tick 使用
static void bgn_impl(CommBase *base) {
  ComKey *me = container_of(base, ComKey, base);
  me->base.cur = HAL_GPIO_ReadPin(me->port, me->pin);
}

// 读取 GPIO 当前电平
static uint8_t read_impl(CommBase *base) {
  ComKey *me = container_of(base, ComKey, base);
  return HAL_GPIO_ReadPin(me->port, me->pin);
}

static const CommOps com_key_ops = {
  .send = send_impl,
  .bgn  = bgn_impl,
  .read = read_impl,
};

// 初始化按键驱动: 调基类构造 → 绑定 GPIO 端口/引脚 → 注册 ops → 状态机归零
void com_key_init(ComKey *me, CommName name, GPIO_TypeDef *port, uint16_t pin) {
  comm_base_init(&me->base);
  me->base.name = name;
  me->port      = port;
  me->pin       = pin;
  me->base.ops  = &com_key_ops;
  me->st        = KEY_IDLE;
  me->cnt       = 0;
  me->event     = 0;
  me->prev      = HAL_GPIO_ReadPin(port, pin);
}

// -------- 双击状态机 (每 10ms 调用) --------
// IDLE → 按下 → PRESS → 释放+短按 → WAIT → 超时=单击 / 再按=双击
//                     → 释放+长按 → IDLE + 长按事件

// 每 10ms 状态机推进: 检测按下/释放边沿, 判断单击/双击/长按
void key_tick(ComKey *me) {
  me->event = 0;
  uint8_t cur = HAL_GPIO_ReadPin(me->port, me->pin);

  switch (me->st) {

  case KEY_IDLE:
    if (me->prev && !cur) {         // 下降沿 → 按下
      me->st  = KEY_PRESS;
      me->cnt = 0;
    }
    break;

  case KEY_PRESS:
    me->cnt++;
    if (!me->prev && cur) {         // 上升沿 → 释放
      if (me->cnt < KEY_DBL_GAP) {
        me->st  = KEY_WAIT;         // 短按 → 等待双击
        me->cnt = 0;
      } else {
        me->st    = KEY_IDLE;       // 长按 → 直接上报
        me->event = KEY_EVENT_LONG;
        me->cnt   = 0;
      }
    }
    break;

  case KEY_WAIT:
    me->cnt++;
    if (me->cnt > KEY_DBL_GAP) {    // 超时 → 单击
      me->st    = KEY_IDLE;
      me->event = KEY_EVENT_CLICK;
      me->cnt   = 0;
    } else if (me->prev && !cur) {  // 窗口内再次按下 → 双击
      me->st    = KEY_IDLE;
      me->event = KEY_EVENT_DOUBLE;
      me->cnt   = 0;
    }
    break;
  }
  me->prev = cur;
}

// 读取并清除当前事件, 返回 0=无事件 / KEY_EVENT_CLICK / KEY_EVENT_DOUBLE / KEY_EVENT_LONG
uint8_t key_event(ComKey *me) {
  uint8_t e = me->event;
  me->event = 0;
  return e;
}
