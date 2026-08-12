#ifndef COMP_GPO_H
#define COMP_GPO_H

// GPO 平台层 —— 通用输出抽象基类
// 基类只定义三个能力: 开 / 关 / 调电压(PWM set)
// 子类按设备类型分:
//   GpoLed    — LED (GPIO 开关 / PWM 调光)
//   GpoLaser  — 激光笔 (GPIO)
//   GpoBeep   — 有源蜂鸣器 (GPIO)
//   GpoBuzzer — 无源蜂鸣器 (PWM)
//   GpoFan    — 风扇 (PWM)

#include <stdint.h>
#include <stdbool.h>

// GPO 实例名 —— 用于标识具体硬件实例
typedef enum {
  GpoLedBoard,     // 板载 LED
  GpoLedUser,      // 用户 LED
  GpoLaser,        // 激光笔
  GpoBeep,         // 有源蜂鸣器
  GpoBuzzer,       // 无源蜂鸣器
  GpoFan,          // 风扇
  GpoUser0,        // 用户扩展
  GpoUser1,
  GpoUser2,
  GpoUser3,
} GpoName;

typedef struct GpoBase GpoBase;

// 虚函数指针类型
typedef void (*gpo_on_fn)    (GpoBase *me);
typedef void (*gpo_off_fn)   (GpoBase *me);
typedef void (*gpo_set_fn)   (GpoBase *me, uint32_t duty);  // 调电压/亮度/频率
typedef void (*gpo_toggle_fn)(GpoBase *me);                  // 翻转输出

// 虚函数表 (ops)
typedef struct {
  gpo_on_fn     on;      // [必须] 打开输出
  gpo_off_fn    off;     // [必须] 关闭输出
  gpo_set_fn    set;     // [可选] 设置电压/亮度/频率，开关型设备为 NULL
  gpo_toggle_fn toggle;  // [可选] 翻转输出，为 NULL 时退化为 gpo_on
} GpoOps;

// 基类结构体 —— 只含虚表指针 + 名称
struct GpoBase {
  const GpoOps *ops;
  GpoName       name;
};

// 基类构造 / 析构
void gpo_base_init  (GpoBase *me, GpoName name);
void gpo_base_deinit(GpoBase *me);

// 分发函数 —— 通过 ops 调用子类实现
void gpo_on  (GpoBase *me);                    // 打开 (断言 ops->on 必须存在)
void gpo_off (GpoBase *me);                    // 关闭 (断言 ops->off 必须存在)
void gpo_set (GpoBase *me, uint32_t duty);     // 设置占空比 (若 ops->set 为 NULL 则跳过)
void gpo_toggle(GpoBase *me);                  // 翻转输出 (通过 on/off 组合实现)

#endif
