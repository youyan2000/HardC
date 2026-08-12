#ifndef DEV_GPOS_H
#define DEV_GPOS_H

// 全局 GPO 句柄 —— 应用层通过此文件访问所有 GPO 实例
// 遵循分层架构: Application → Module → Devices → Components → BSP
//
// 用法:
//   #include "gpos.h"
//   gpo_on(g_led_board);        // 打开板载 LED
//   gpo_set(g_buzzer, 500);     // 设置蜂鸣器音调
//   gpo_on(g_laser);            // 打开激光笔

#include "comp_gpo.h"

// 全局 GPO 句柄 (由 board_init.c 绑定到具体子类实例)
extern GpoBase *g_led_board;     // 板载 LED (GPIO / PWM)
extern GpoBase *g_led_user;      // 用户 LED (预留)
extern GpoBase *g_laser;         // 激光笔 (GPIO)
extern GpoBase *g_beep;          // 有源蜂鸣器 (GPIO)
extern GpoBase *g_buzzer;        // 无源蜂鸣器 (PWM)
extern GpoBase *g_fan;           // 风扇 (PWM)

// 用户扩展句柄
extern GpoBase *g_user0;
extern GpoBase *g_user1;

#endif
