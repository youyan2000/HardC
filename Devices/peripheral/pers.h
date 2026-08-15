#ifndef DEV_PERS_H
#define DEV_PERS_H

// 全局外设句柄 — 替代已删的 gpos.h (Devices/gpo)
// board_init.c 绑定到具体子类实例, Module/App 层通过此文件访问 (值包含, 零 malloc)
//
// 用法:
//   #include "pers.h"
//   output_on(&g_led_board->base);       // 打开板载 LED
//   output_set(&g_buzzer->base, 500);    // 设置蜂鸣器占空比
//   per_ultrasonic_tick(g_ultrasonic);   // 每 10ms 触发测距

#include "comp_output.h"
#include "per_oled.h"
#include "per_mpu6050.h"
#include "per_ultrasonic.h"
#include "per_led.h"
#include "per_laser.h"
#include "per_beep.h"
#include "per_buzzer.h"
#include "per_fan.h"

// 全局外设句柄 (由 board_init.c 绑定到具体子类实例)
extern PerOled *g_oled;              // OLED 显示 (128x64)
extern PerMpu6050 *g_mpu;            // MPU6050 六轴 IMU
extern PerUltrasonic *g_ultrasonic;  // 超声波测距
extern PerLed *g_led_board;          // 板载 LED
extern PerLed *g_led_user;           // 用户 LED
extern PerLaser *g_laser;            // 激光
extern PerBeep *g_beep;              // 有源蜂鸣器
extern PerBuzzer *g_buzzer;          // 无源蜂鸣器
extern PerFan *g_fan;                // 风扇

#endif  // DEV_PERS_H
