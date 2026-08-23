// BSP 跳转抽象 — 跳转到 App 起始地址 (平台差异归 BSP)
//
// 定位: Bootloader 最终把控制权交给 App 的入口.
//   STM32 (Cortex-M): 读 App 栈顶 + Reset_Handler → 重映射 VTOR → 设 MSP → 跳转
//   C2000 (C28x):      [待验证] 跳 App 的 _c_int00 (codestartbranch.asm), 需真机 CCS 验证
//
// 上层 (mod_bootloader) 只调 bsp_jump_to_app / bsp_jump_validate_app,
// 不关心平台差异. 跳转前需关闭全部中断, 确保干净启动.

#ifndef BSP_JUMP_H
#define BSP_JUMP_H

#include <stdint.h>

// 跳转到 App 入口
//   app_entry: App 入口地址
//     STM32: App 基址 (向量表起始, 前 8 字节 = MSP + Reset_Handler)
//     C2000: [待验证] App 的 _c_int00 入口地址
// 不返回 (跳转后控制权交给 App)
void bsp_jump_to_app(uint32_t app_entry);

// 校验 App 入口是否有效 (防止跳到损坏/空 Flash)
//   返回 0=有效, 非0=无效
int bsp_jump_validate_app(uint32_t app_entry);

#endif  // BSP_JUMP_H
