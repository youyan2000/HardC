// BSP Flash 硬件抽象接口 — 平台无关擦/写/读 + 分区管理 (不透明句柄)
//
// 定位: 供 Bootloader (boot_loader) 和 Components/database 经 CompFlashOps 适配访问.
//   BSP 之上零 HAL: 上层只持 BspFlash 不透明句柄 + 调 bsp_flash_* 抽象.
//   平台内部 (bsp_flash_stm32.c 用 HAL, bsp_flash_c2000.c 用 driverlib) 自决.
//
// 设计 (对标 LibXR Flash/STM32Flash 的纯C 不透明句柄版):
//   - bsp_flash_set_region 绑定一段连续 Flash 区域 (对标 LibXR start_sector),
//     之后所有 offset 都是相对该区域基址的偏移 — 上层不关心绝对地址.
//   - 擦除按扇区/页粒度 (min_erase_size); 写入按最小写单元 (min_write_size) 分块,
//     相同数据跳过 (省 Flash 寿命, 学 LibXR FastCmp 跳过).
//   - 擦写前关 I/D Cache, 后恢复 (F4/G4/H7 有 Cache 时; F334 无 Cache 忽略).
//
// 平台差异:
//   STM32 F334: 16KB 页, HAL_FLASHEx_Erase(PAGES), HAL_FLASH_Program(HALFWORD=16bit)
//   STM32 G474: 2KB 页,  HAL_FLASHEx_Erase(PAGES), HAL_FLASH_Program(DOUBLEWORD=64bit)
//   C2000 F280049: 8KB sector × 32 (2 Bank), TI F021 Flash API (bsp_flash_c2000_fixed.c)
//
// 由 cmake/HardC.CMake 的 st/c2000 分支编译.

#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include <stdint.h>

// ======== 不透明句柄 ========
//   STM32: NULL (用全局 FLASH) 或 FLASH_HandleTypeDef*;
//   C2000: NULL (用全局 Flash 控制器)
typedef void BspFlash;

// ======== Flash 物理参数 POD ========
typedef struct {
  uint32_t base_addr;       // 绑定区域基址 (如 STM32 0x08008000)
  uint32_t size;            // 绑定区域大小 (字节)
  uint32_t min_erase_size;  // 最小擦除单元 (STM32: 页大小; C2000: 4KB sector)
  uint32_t min_write_size;  // 最小写单元 (STM32: FLASH_TYPEPROGRAM 位宽; C2000: 编程粒度)
} BspFlashInfo;

// ======== BSP 接口 ========

// 绑定平台 Flash (STM32/C2000 均传 NULL, 用全局 Flash; 返回非 NULL=成功)
BspFlash *bsp_flash_bind(void *h);

// 绑定一段连续 Flash 区域 — 之后所有 offset 相对此基址
//   info: 区域基址/大小/最小单元; 由 App 从 flash_map (YAML 生成) 传入
// 返回 0=成功, 非0=参数错误
int bsp_flash_set_region(BspFlash *me, const BspFlashInfo *info);

// 擦除 [offset, offset+size) — 相对区域基址, 按 min_erase_size 对齐
// 返回 0=成功, 非0=错误
int bsp_flash_erase(BspFlash *me, uint32_t offset, uint32_t size);

// 写入 [offset, offset+len) — 按 min_write_size 分块, 相同数据跳过
// 返回 0=成功, 非0=错误
int bsp_flash_write(BspFlash *me, uint32_t offset, const uint8_t *data, uint32_t len);

// 读取 [offset, offset+len) — 直接从 Flash 地址拷贝
// 返回 0=成功, 非0=错误
int bsp_flash_read(BspFlash *me, uint32_t offset, uint8_t *buf, uint32_t len);

// 容量信息 (绑定后可用)
uint32_t bsp_flash_min_erase_size(BspFlash *me);
uint32_t bsp_flash_min_write_size(BspFlash *me);
uint32_t bsp_flash_size(BspFlash *me);

#endif  // BSP_FLASH_H
