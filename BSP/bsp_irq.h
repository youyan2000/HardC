// BSP 中断优先级分层抽象 — 确保 FAST 绝对优先, 不被 HMI/通信/SLOW 干扰
//
// 硬约束: FAST(控制定时器 ISR) 抢占优先级最高。任何 HMI/通信/外设中断都
//   不能抢占 FAST。Cortex-M 保证"抢优先级数字最小的不可被任何同级/低优先
//   数打断", 因此只要 FAST=0, 其余全 >=1, 硬件即保证 FAST 独占。
//
// 平台差异:
//   STM32 (Cortex-M, NVIC): 抢占优先级数字越小越优先。FAST 设最小即可。
//   C2000 (PIE, 中断不嵌套): 进 ISR 自动屏蔽其余中断, FAST ISR 运行时天然
//     不受任何中断干扰; 本层提供语义占位与断言钩子。
//
// 用法(board_init 开头调用):
//   bsp_irq_set_fast_preempt(HRTIM1_IRQn, 0);   // 显式把 FAST IRQ 抢优先设最小
//   bool ok = bsp_irq_assert_order(HRTIM1_IRQn); // debug: 断言真的最小

#ifndef BSP_IRQ_H
#define BSP_IRQ_H

#include <stdint.h>
#include <stdbool.h>

// 把 FAST 控制定时器中断的抢占优先级设为给定数字 (STM32: 0=最高)。
//   IRQn 平台相关: STM32 传 HAL IRQn (如 HRTIM1_IRQn); C2000 传 PIE 组号(占位)。
//   返回 false = 平台不支持显式设置(如 C2000 不嵌套, 走 PIE 掩码)。
bool bsp_irq_set_fast_preempt(int irqn, uint8_t preempt);

// 断言 FAST 中断抢占优先级确实为期望的最小值 (debug 防误配)。
//   STM32: 读 NVIC_GetPriority 比较; C2000: 恒 true (硬件不嵌套即保证)。
//   返回 false = 配置被改坏。
bool bsp_irq_assert_order(int irqn, uint8_t expect_preempt);

// ======== 库级强制入口 (默认调用) ========

// 三档中断配置 — 工程在 board_init 填三个中断号 (STM32: IRQn; C2000: PIE 组, 占位)
typedef struct {
  int fast_irqn;  // 控制定时器 ISR (FAST) 中断号 → 强制抢优先 0 (最高)
  int slow_irqn;  // 监控定时器 ISR (SLOW) 中断号 → 强制抢优先 1
  int hmi_irqn;   // HMI/通信中断号 → 强制抢优先 2
} BspIrqCfg;

// 库级强制: 把三档中断优先级钉死 (FAST=0 / SLOW=1 / HMI=2), 不靠工程自觉。
//   - 逐个强制写 NVIC 优先级 (STM32) / PIE 语义 (C2000)
//   - 全部写完后读回校验: 任一不满足 (FAST!=0 或 SLOW<=FAST 或 HMI<=SLOW) → 返回 false
//   - 返回 false = 配置被改坏/不支持 → App 必须停机, 不允许带病运行
// board_init 默认调用; 失败应进入停机处理 (强制语义, 不静默)。
bool bsp_irq_apply(const BspIrqCfg *cfg);

// ======== 采样类中断的附加约定 (A4) ========
// ADC/DMA 完成中断 (HAL_ADC_ConvCpltCallback 的驱动源) 抢占优先级必须低于 FAST:
//   - 生产侧 on_dma_complete 在"标 pending → 重装 DMA"之间若被 FAST 抢占,
//     FAST fetch 会先切快照 → 重装目标必须已在标 pending 前捕获 (见 adc_dc_sampler.c);
//     即便如此, 仍要求 ADC/DMA 完成中断优先级 ≥1 (低于 FAST=0), 否则其 ISR 本体
//     会抢占控制环 — 直接破坏"FAST 不被任何东西干扰"的总约定.
//   - 建议: ADC/DMA 完成中断与 SLOW 同级 (1) 或更低; 禁止设到 0 与 FAST 同级.
//   - C2000 PIE 不嵌套, 该约定天然满足 (无抢占窗口).

// 全局中断临界区 (MAIN 侧保护共享数据; ISR 侧天然原子, 无需调用)
//   STM32: __disable_irq/__enable_irq; C2000: DINT/EINT
void bsp_irq_lock(void);
void bsp_irq_unlock(void);

#endif  // BSP_IRQ_H
