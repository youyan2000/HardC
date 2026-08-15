#ifndef COMP_OUTPUT_H
#define COMP_OUTPUT_H

// OutputBase — 通用输出抽象基类
//
// 来源: 原 Components/gpo/comp_gpo.h 改造 (阶段2 peripheral 域收编)
//   - GpoBase → OutputBase, GpoOps → OutputOps, gpo_* 分发 → output_*
//   - 去 GpoName 枚举 → const char *name (实例名直接传字符串, LibXR 惯例)
//
// 能力: on/off (必须) + set (可选, 开关型 NULL) + toggle (可选, NULL 退化 on)
// 子类: per_led/per_laser/per_beep (GPIO 开关) + per_buzzer/per_fan (TIM PWM), Devices/peripheral

#include <stdint.h>

typedef struct OutputBase OutputBase;

// 虚函数指针类型
typedef void (*output_on_fn)(OutputBase *me);                   // 打开输出
typedef void (*output_off_fn)(OutputBase *me);                  // 关闭输出
typedef void (*output_set_fn)(OutputBase *me, uint32_t level);  // [可选] 调电平/亮度/频率
typedef void (*output_toggle_fn)(OutputBase *me);               // [可选] 翻转输出

// 虚函数表 (ops)
typedef struct {
  output_on_fn on;          // [必须] 打开输出
  output_off_fn off;        // [必须] 关闭输出
  output_set_fn set;        // [可选] 设置电平/亮度/频率, 开关型设备为 NULL
  output_toggle_fn toggle;  // [可选] 翻转输出, 为 NULL 时退化为 on
} OutputOps;

// 基类结构体 — 只含虚表指针 + 名称
struct OutputBase {
  const OutputOps *ops;
  const char *name;
};

// 基类构造 / 析构
void output_base_init(OutputBase *me, const char *name);
void output_base_deinit(OutputBase *me);

// 分发函数 — 通过 ops 调用子类实现
void output_on(OutputBase *me);                   // 打开 (断言 ops->on 必须存在)
void output_off(OutputBase *me);                  // 关闭 (断言 ops->off 必须存在)
void output_set(OutputBase *me, uint32_t level);  // 设置电平 (ops->set 为 NULL 则跳过)
void output_toggle(OutputBase *me);               // 翻转 (无则退化为 on)

#endif  // COMP_OUTPUT_H
