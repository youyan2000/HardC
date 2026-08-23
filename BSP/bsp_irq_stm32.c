// BSP IRQ STM32 后端 — bsp_irq.h 的 STM32 (Cortex-M/NVIC) 实现
//
// 硬约束: FAST 控制定时器中断抢占优先级设为 0(最小=最高)。
//   Cortex-M NVIC 保证"抢优先级为 0 的中断不能被任何同级/低优先打断",
//   因此只要其余中断(SLOW/HMI/通信)抢占优先级 >=1, FAST 即绝对独占。
//
// 由 cmake/HardC.CMake 的 st 分支编译; 系列由 bsp_stm32_hal.h 选择.

#include "bsp_irq.h"
#include "bsp_stm32_hal.h"

bool bsp_irq_set_fast_preempt(int irqn, uint8_t preempt) {
  if (irqn < 0) {
    return false;
  }
  // 抢占优先级: preempt; 子优先级固定 0.
  // 保持 NVIC 分组不变 (CubeMX 默认 4 位抢占/0 位子优先), 用 preempt 直写.
  HAL_NVIC_SetPriority((IRQn_Type) irqn, preempt, 0u);
  return true;
}

bool bsp_irq_assert_order(int irqn, uint8_t expect_preempt) {
  if (irqn < 0) {
    return false;
  }
  // 读回当前优先级 (preempt 位 = (val & NVIC_PRIORITY_MASK) 的高 4 位)
  uint32_t prio = NVIC_GetPriority((IRQn_Type) irqn);
  uint8_t preempt_bits = (uint8_t) (prio >> (8u - 4u));  // 4 位抢占
  uint8_t sub_bits = (uint8_t) (prio & 0x0Fu);
  return preempt_bits == expect_preempt && sub_bits == 0u;
}

// 库级强制: 把三档里每一档列出的**所有**中断优先级钉死 + 逐个读回校验.
//   空档(!n)跳过(不写不校验). 任一失败 → false (App 须停机).
// 对一档内所有中断写/读回优先级; n 转 unsigned 避免 -Wsign-compare.
static bool _irq_set_all(const int *irqn, int n, uint8_t preempt) {
  for (unsigned i = 0; i < (unsigned) n; ++i)
    if (!bsp_irq_set_fast_preempt(irqn[i], preempt))
      return false;
  return true;
}
static bool _irq_assert_all(const int *irqn, int n, uint8_t expect) {
  for (unsigned i = 0; i < (unsigned) n; ++i)
    if (!bsp_irq_assert_order(irqn[i], expect))
      return false;
  return true;
}
bool bsp_irq_apply(const BspIrqCfg *cfg) {
  if (cfg == NULL) {
    return false;
  }
  // 1. 所有 FAST=0 / SLOW=1 / HMI=2 (抢占优先, 子优先 0)
  if (!_irq_set_all(cfg->fast_irqn, cfg->fast_n, 0u)) return false;
  if (!_irq_set_all(cfg->slow_irqn, cfg->slow_n, 1u)) return false;
  if (!_irq_set_all(cfg->hmi_irqn,  cfg->hmi_n,  2u)) return false;

  // 2. 逐个读回校验: 同一档内所有中断都必须是预期优先级
  if (!_irq_assert_all(cfg->fast_irqn, cfg->fast_n, 0u)) return false;
  if (!_irq_assert_all(cfg->slow_irqn, cfg->slow_n, 1u)) return false;
  if (!_irq_assert_all(cfg->hmi_irqn,  cfg->hmi_n,  2u)) return false;

  return true;
}
// 全局中断临界区 (Cortex-M)
void bsp_irq_lock(void) {
  __disable_irq();
}

void bsp_irq_unlock(void) {
  __enable_irq();
}
