// BSP 跳转 STM32 后端 — bsp_jump.h 的 Cortex-M 实现
//
// Cortex-M 跳转 App:
//   1. App 向量表起始 (app_entry) 前 8 字节:
//      [0] = 初始 MSP (栈顶地址, 在 RAM 内)
//      [4] = Reset_Handler (复位向量, 在 Flash 内)
//   2. 关全部中断 (__disable_irq + 清 pending; 跳转前彻底停中断)
//   3. 重映射 VTOR = app_entry (中断向量表指向 App 的向量表)
//   4. 设 MSP = App 初始栈顶
//   5. 跳转到 Reset_Handler
//
// 关键: 跳转前必须确保没有中断残留 (否则 App 的向量表未初始化前可能触发).
//   __disable_irq 只关 PRIMASK, 已挂起的 PendSV/SysTick 仍会在使能后触发 —
//   这里在跳转前停在 __disable_irq 不返回, 由 App 的 SystemInit 重新初始化.

#include "bsp_jump.h"

// 平台能力: Cortex-M 直接操作 SCB/NVIC
#include "bsp_stm32_hal.h"

// 阈值: App 入口在 Flash 区, 栈顶在 RAM 区 (具体范围由工程 flash_map 定义)
// 这里用默认 (F334/G4: Flash 0x08000000-0x0807FFFF, RAM 0x20000000-0x2001FFFF)
// 可在编译期由外部覆盖
#ifndef BSP_JUMP_FLASH_LO
#define BSP_JUMP_FLASH_LO 0x08000000u
#endif
#ifndef BSP_JUMP_FLASH_HI
#define BSP_JUMP_FLASH_HI 0x08080000u
#endif
#ifndef BSP_JUMP_RAM_LO
#define BSP_JUMP_RAM_LO 0x20000000u
#endif
#ifndef BSP_JUMP_RAM_HI
#define BSP_JUMP_RAM_HI 0x20020000u
#endif

void bsp_jump_to_app(uint32_t app_entry) {
  // 读 App 初始 MSP 和 Reset_Handler
  uint32_t app_sp = *(volatile uint32_t *) app_entry;
  uint32_t app_pc = *(volatile uint32_t *) (app_entry + 4u);

  // 关全局中断 (PRIMASK), 防止跳转过程被中断打断
  __disable_irq();

  // 清 NVIC 挂起 (防 PendSV/SysTick 残留)
  SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk | SCB_ICSR_PENDSTCLR_Msk;

  // 重映射中断向量表到 App
  SCB->VTOR = app_entry;

  // 设 App 栈顶
  __set_MSP(app_sp);
  __set_CONTROL(0u);  // 线程模式, MSP, 特权级 (App 从特权态启动)

  // 跳转到 App Reset_Handler
  typedef void (*ResetHandler)(void);
  ResetHandler reset = (ResetHandler) app_pc;
  reset();

  // 不应到达
  for (;;) {
  }
}

int bsp_jump_validate_app(uint32_t app_entry) {
  if ((app_entry & 0x3u) != 0u) {
    return -1;  // 未对齐
  }
  if (app_entry < BSP_JUMP_FLASH_LO || app_entry >= BSP_JUMP_FLASH_HI) {
    return -1;  // 不在 Flash 区
  }
  uint32_t app_sp = *(volatile uint32_t *) app_entry;
  uint32_t app_pc = *(volatile uint32_t *) (app_entry + 4u);
  // 栈顶必须在 RAM 区, 复位向量必须在 Flash 区
  if (app_sp < BSP_JUMP_RAM_LO || app_sp >= BSP_JUMP_RAM_HI) {
    return -1;
  }
  if (app_pc < BSP_JUMP_FLASH_LO || app_pc >= BSP_JUMP_FLASH_HI) {
    return -1;
  }
  return 0;
}
