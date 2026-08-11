#ifndef PWMS_H
#define PWMS_H

// PWM 全局句柄 —— 应用层只需要 include 这一个头文件
//
// 使用方式:
//   #include "pwms.h"
//   pwm_start(g_buck_main);
//   pwm_set_duty(g_buck_main, 0, 0.5f);
//   pwm_set_phase(g_full_bridge, 0, 45.0f);
//   pwm_emergency_stop(g_buck_main);
//
// 句柄在 board_init.c 中绑定具体子类实例

#include "comp_pwm.h"

// ======== 全局句柄声明 (由 board_init.c 定义) ========

// 各拓扑的通用句柄, 应用层通过 PwmBase* 多态调用
extern PwmBase *g_buck;          // 降压 PWM
extern PwmBase *g_boost;         // 升压 PWM
extern PwmBase *g_buckboost;     // 升降压 PWM
extern PwmBase *g_half_bridge;   // 半桥 PWM
extern PwmBase *g_full_bridge;   // 全桥移相 PWM
extern PwmBase *g_interleaved;   // 交错并联 PWM
extern PwmBase *g_resonant;      // 谐振变频 PWM

// 用户扩展句柄
extern PwmBase *g_user0;
extern PwmBase *g_user1;
extern PwmBase *g_user2;
extern PwmBase *g_user3;

#endif
