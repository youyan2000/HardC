// 人机交互模块 — HMI (Module 层实现)
// 来源: LitteCar_STM32 mod_hmi.c (按键→CarCmd 映射 + OLED 菜单)
//       + MyFinal_Work user_hmi.c (3 按键去抖 + OLED 菜单状态机)
//       + Car_Control_Study_Report §8 (按键+OLED+MPU6050 传感器系统)
//
// 架构: 按键事件 → CarCmd → cmd_dispatch_execute → 各模块回调
//       物理输入和通信输入走同一路径, 零代码重复

#include "mod_hmi.h"
#include "mod_cmd_dispatch.h"
#include <stddef.h>

// ======== 初始化 ========

void hmi_init(Hmi *me, CmdDispatcher *disp) {
  me->disp      = disp;
  me->key_count = 0;
  me->oled_page = 0;
  me->oled_dirty = false;
  me->tick      = 0;

  for (int i = 0; i < HMI_MAX_KEYS; i++) {
    me->keys[i].pin_state     = 1;  // 默认释放
    me->keys[i].prev_state    = 1;
    me->keys[i].press_ticks   = 0;
    me->keys[i].release_ticks = 0;
    me->keys[i].click_count   = 0;
    me->keys[i].event_pending = false;
    me->keys[i].pending_event = KEY_EVENT_CLICK;
    me->key_pin_states[i]     = 1;
  }
}

void hmi_add_key(Hmi *me, uint8_t key_index, uint8_t initial_state) {
  if (key_index >= HMI_MAX_KEYS) return;
  me->keys[key_index].pin_state  = initial_state;
  me->keys[key_index].prev_state = initial_state;
  me->key_pin_states[key_index]  = initial_state;
  if (key_index >= me->key_count) {
    me->key_count = key_index + 1;
  }
}

// ======== 单按键去抖 ========

static void key_debounce_tick(KeyDebounce *kd) {
  // 读取当前引脚状态 (由应用层在 hmi_tick 前通过读 GPIO 刷新到 key_pin_states)
  uint8_t raw = kd->pin_state;

  // 边沿检测
  bool falling = (kd->prev_state == 1 && raw == 0);  // 按下
  bool rising  = (kd->prev_state == 0 && raw == 1);  // 释放
  kd->prev_state = raw;

  if (falling) {
    // 按下: 如果释放后在双击窗口内, 递增单击计数
    if (kd->release_ticks > 0 && kd->release_ticks <= HMI_DOUBLE_TICKS) {
      kd->click_count++;
    } else {
      kd->click_count = 1;
    }
    kd->press_ticks   = 0;
    kd->release_ticks = 0;
    kd->event_pending = false;
  }

  if (raw == 0) {
    // 正在按下: 计时
    kd->press_ticks++;
    // 长按检测: 达到阈值时触发
    if (kd->press_ticks == HMI_LONG_TICKS) {
      kd->event_pending = true;
      kd->pending_event = KEY_EVENT_LONG;
    }
  }

  if (rising) {
    // 释放
    kd->release_ticks = 0;
    // 不是长按 → 在释放时产生单击/双击事件
    if (!kd->event_pending) {
      kd->event_pending = true;
      if (kd->click_count >= 2) {
        kd->pending_event = KEY_EVENT_DOUBLE;
      } else {
        kd->pending_event = KEY_EVENT_CLICK;
      }
    }
    kd->press_ticks = 0;  // 复位按下计时
  } else if (raw == 1) {
    // 正在释放: 计时 (双击窗口)
    if (kd->release_ticks < 255) {
      kd->release_ticks++;
    }
  }
}

// ======== 每 tick 更新 ========

void hmi_tick(Hmi *me) {
  me->tick++;

  // 扫描所有注册按键
  for (uint8_t i = 0; i < me->key_count; i++) {
    // 应用层刷新引脚电平
    me->keys[i].pin_state = me->key_pin_states[i];

    key_debounce_tick(&me->keys[i]);

    // 消费待处理事件 → CarCmd → dispatch
    if (me->keys[i].event_pending) {
      me->keys[i].event_pending = false;
      KeyEvent evt = me->keys[i].pending_event;

      CarCmd cmd = cmd_from_button(i, (uint8_t)evt);
      if (cmd != CMD_NONE && me->disp) {
        cmd_dispatch_execute(me->disp, cmd, NULL, 0);
      }
    }
  }
}

// ======== OLED 页面 ========

void hmi_oled_next_page(Hmi *me) {
  me->oled_page++;
  if (me->oled_page > 3) me->oled_page = 0;
  me->oled_dirty = true;
}

void hmi_oled_set_dirty(Hmi *me) {
  me->oled_dirty = true;
}
