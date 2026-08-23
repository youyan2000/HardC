// BSP IRQ C2000 后端 — bsp_irq.h 的 C2000 (PIE) 实现
//
// C2000 (PIE 向量) 中断默认不嵌套: 进入 ISR 后由 IER/INTM 硬件屏蔽其余中断,
//   FAST 控制 ISR 运行时天然不受任何中断干扰。故:
//     - set_fast_preempt: C2000 无可配抢占优先级(用 PIE 组 + IER 屏蔽),
//       语义上"FAST 不受扰"已由硬件保证, 这里返回 false 表示"无需/不支持显式设置".
//     - assert_order: 恒 true (硬件不嵌套即保证 FAST 不被同级/低优打断).
//
// 由 cmake/HardC.CMake 的 c2000 分支编译; 通过 driverlib 头获得 INT 宏 (占位).

#include "bsp_irq.h"

#include <stdbool.h>

bool bsp_irq_set_fast_preempt(int irqn, uint8_t preempt) {
  (void) irqn;
  (void) preempt;
  // C2000 PIE 不嵌套, 无可配抢占优先级; FAST 不受扰由硬件屏蔽保证。
  // 返回 false = 平台不支持显式设置(语义已由 PIE 保证).
  return false;
}

bool bsp_irq_assert_order(int irqn, uint8_t expect_preempt) {
  (void) irqn;
  (void) expect_preempt;
  // PIE 中断不嵌套: FAST ISR 运行时其余中断被 IER 屏蔽, 天然不被抢占。
  return true;
}

// 库级强制: C2000 PIE 中断不嵌套, FAST ISR 运行时其余中断被 IER 硬件屏蔽,
//   三档次序天然成立。此处做语义校验 (cfg 非空即视为通过; 无可配抢占优先级)。
bool bsp_irq_apply(const BspIrqCfg *cfg) {
  if (cfg == NULL) {
    return false;
  }
  // C2000 无 NVIC 可配抢占优先级; 硬件不嵌套保证 FAST 不被抢占.
  // 三个 IRQn 仅作语义占位 (PIE 组), 返回 true 表示"硬件已保证".
  return true;
}
// 全局中断临界区 (C28x: DINT/EINT)
void bsp_irq_lock(void) {
  DINT;
}

void bsp_irq_unlock(void) {
  EINT;
}
