// BSP 层步进电机硬件抽象
//
// 设计原则:
//   - 引脚 + 定时器通过 ctx 结构体传入, 函数 CHECK ctx 后使用
//   - 速度控制: 写定时器 LOAD 寄存器 (非固定 ISR)
//   - 占空比 50%: period/2 → CMP 寄存器
//   - 平台相关: STM32 TIM / TI GPTimer / MSPM0 TIMG 各自实现
//
// ctx 模式: 每个函数接收 void *ctx → 强转为平台相关结构体 → 访问端口/定时器
// 允许多实例: 不同轴用不同的 ctx, 同一个函数指针可以服务多个电机

#ifndef BSP_STEP_MOTOR_H
#define BSP_STEP_MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"

// ======== 平台抽象: DIR 引脚 ctx ========
typedef struct {
  GPIO_TypeDef *port;
  uint16_t      pin;
} BspStepDir;

// ======== 平台抽象: PUL 定时器 ctx ========
typedef struct {
  TIM_TypeDef  *tim;       // 定时器基地址 (如 TIM2)
  uint8_t       cc_ch;     // 比较通道索引 (0-3 → CC1-CC4)
} BspStepPul;

// ======== BSP 函数声明 ========

// 设置方向引脚 — HIGH=正转, LOW=反转
// ctx 指向 BspStepDir {port, pin}, CHECK 后使用 (Bug #4: 不能 (void)ctx)
void bsp_step_dir_set(void *ctx, bool high);

// 设置脉冲周期 (速度控制)
// period = 定时器 ARR 值, 频率 = 定时器时钟 / (prescaler * period)
// ctx 指向 BspStepPul {tim, cc_ch}, CHECK 后写入 tim->CCRx (50% 占空比)
// 关键: 直接操作硬件寄存器 → Speed control works!
void bsp_step_pul_set_period(void *ctx, uint16_t period);

// 使能/禁用定时器输出 (启动/停止脉冲)
void bsp_step_pul_enable(void *ctx, bool enable);

// 写当前相位到 GPIO (4 相: A/B/C/D)
// pins[4] = {BspStepDir for A, B, C, D}, phase_data = 相序表当前拍
void bsp_step_write_phase(const BspStepDir pins[4], uint8_t phase_data);

#endif
