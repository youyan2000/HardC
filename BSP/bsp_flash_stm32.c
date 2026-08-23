// BSP Flash STM32 后端 — bsp_flash.h 的 STM32 (HAL) 实现
//
// 目标系列: F334 (HRTIM 直流) + G474 (HRTIM 直流/交流) — 均用 PAGES 擦除模式.
//   F334: FLASH_PAGE_SIZE = 16KB, FLASH_TYPEPROGRAM_HALFWORD (16-bit 编程)
//   G474: FLASH_PAGE_SIZE = 2KB,  FLASH_TYPEPROGRAM_DOUBLEWORD (64-bit 编程)
//
// 实现要点 (对标 LibXR stm32_flash.cpp):
//   - bsp_flash_set_region 绑定一段区域 (基址/大小/最小单元)
//   - Erase: 遍历区域内的页, 逐个 HAL_FLASHEx_Erase (支持跨多页一段)
//   - Write: 按 min_write_size 分块, memcmp 相同则跳过 (省寿命), HAL_FLASH_Program
//   - 解锁/上锁: 擦写前后 HAL_FLASH_Unlock/Lock
//   - Cache: F334 无 Cache 忽略; G4/F4/H7 擦写前关后恢复
//
// 由 cmake/HardC.CMake 的 st 分支编译; 系列由 bsp_stm32_hal.h 选择.

#include "bsp_flash.h"
#include "bsp_stm32_hal.h"
#include <string.h>

// ======== 模块级状态 (单例, 同 bsp_uart 约定) ========

static uint32_t s_base = 0u;        // 绑定区域基址
static uint32_t s_size = 0u;        // 绑定区域大小
static uint32_t s_min_erase = 0u;   // 页大小
static uint32_t s_min_write = 0u;   // 编程单元
static uint32_t s_prog_type = 0u;   // FLASH_TYPEPROGRAM_*

// 探测页大小 (PAGES 模式)
static uint32_t detect_page_size(void) {
#if defined(FLASH_PAGE_SIZE)
  return (uint32_t) FLASH_PAGE_SIZE;
#elif defined(FLASH_SECTOR_SIZE)
  return (uint32_t) FLASH_SECTOR_SIZE;
#else
  return 0u;
#endif
}

// 探测编程类型/最小单元
static uint32_t detect_prog_type(void) {
#if defined(FLASH_TYPEPROGRAM_DOUBLEWORD)
  return (uint32_t) FLASH_TYPEPROGRAM_DOUBLEWORD;
#elif defined(FLASH_TYPEPROGRAM_HALFWORD)
  return (uint32_t) FLASH_TYPEPROGRAM_HALFWORD;
#elif defined(FLASH_TYPEPROGRAM_WORD)
  return (uint32_t) FLASH_TYPEPROGRAM_WORD;
#else
  return 0u;
#endif
}

static uint32_t prog_min_write(void) {
#if defined(FLASH_TYPEPROGRAM_DOUBLEWORD)
  return 8u;
#elif defined(FLASH_TYPEPROGRAM_HALFWORD)
  return 2u;
#elif defined(FLASH_TYPEPROGRAM_WORD)
  return 4u;
#else
  return 1u;
#endif
}

// ======== 接口 ========

BspFlash *bsp_flash_bind(void *h) {
  (void) h;  // STM32 用全局 FLASH, 忽略句柄
  return (BspFlash *) &s_base;  // 非 NULL = 已绑定
}

int bsp_flash_set_region(BspFlash *me, const BspFlashInfo *info) {
  (void) me;
  if (info == NULL || info->size == 0u || info->base_addr == 0u) {
    return -1;
  }
  s_base = info->base_addr;
  s_size = info->size;
  s_min_erase = (info->min_erase_size != 0u) ? info->min_erase_size : detect_page_size();
  s_min_write = (info->min_write_size != 0u) ? info->min_write_size : prog_min_write();
  s_prog_type = detect_prog_type();
  if (s_min_erase == 0u || s_min_write == 0u) {
    return -1;
  }
  return 0;
}

int bsp_flash_erase(BspFlash *me, uint32_t offset, uint32_t size) {
  (void) me;
  if (size == 0u) {
    return 0;
  }
  if (offset + size > s_size) {
    return -1;
  }
  uint32_t start_addr = s_base + offset;
  uint32_t end_addr = start_addr + size;

  // 关 I/D Cache (有 Cache 时)
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  bool dcache_en = (SCB->CCR & SCB_CCR_DC_Msk) != 0u;
  if (dcache_en) SCB_DisableDCache();
#endif
#if defined(__ICACHE_PRESENT) && (__ICACHE_PRESENT == 1U)
  bool icache_en = (SCB->CCR & SCB_CCR_IC_Msk) != 0u;
  if (icache_en) SCB_DisableICache();
#endif

  HAL_FLASH_Unlock();

  uint32_t page = start_addr / s_min_erase;
  uint32_t last_page = (end_addr - 1u) / s_min_erase;
  for (; page <= last_page; page++) {
    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Page = (uint32_t) page,
        .NbPages = 1u,
    };
    uint32_t error = 0u;
    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &error);
    if (st != HAL_OK || error != 0xFFFFFFFFu) {
      HAL_FLASH_Lock();
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
      if (dcache_en) SCB_EnableDCache();
#endif
#if defined(__ICACHE_PRESENT) && (__ICACHE_PRESENT == 1U)
      if (icache_en) SCB_EnableICache();
#endif
      return -1;
    }
  }

  HAL_FLASH_Lock();

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  if (dcache_en) SCB_EnableDCache();
#endif
#if defined(__ICACHE_PRESENT) && (__ICACHE_PRESENT == 1U)
  if (icache_en) SCB_EnableICache();
#endif
  return 0;
}

int bsp_flash_write(BspFlash *me, uint32_t offset, const uint8_t *data, uint32_t len) {
  (void) me;
  if (data == NULL || len == 0u) {
    return 0;
  }
  if (offset + len > s_size) {
    return -1;
  }
  uint32_t addr = s_base + offset;

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  bool dcache_en = (SCB->CCR & SCB_CCR_DC_Msk) != 0u;
  if (dcache_en) SCB_DisableDCache();
#endif
#if defined(__ICACHE_PRESENT) && (__ICACHE_PRESENT == 1U)
  bool icache_en = (SCB->CCR & SCB_CCR_IC_Msk) != 0u;
  if (icache_en) SCB_DisableICache();
#endif

  HAL_FLASH_Unlock();

  const uint8_t *src = data;
  uint32_t written = 0u;
  while (written < len) {
    // 每次写一个完整单元 (s_min_write); 最后不足时用 0xFF 补齐 (擦除态, 写入无影响)
    uint32_t unit = s_min_write;
    uint32_t actual = len - written;
    if (actual > unit) actual = unit;  // 本单元实际数据字节

    // 跳过相同数据 (只比较本单元实际数据)
    if (memcmp((const void *)(addr + written), src + written, actual) == 0) {
      written += unit;  // 前进整个单元 (Flash 已擦, 补齐部分 0xFF 相同)
      continue;
    }

    // 组装一个完整编程单元, 高位 0xFF
    uint64_t word = 0xFFFFFFFFFFFFFFFFu;
    memcpy(&word, src + written, actual);

#if defined(FLASH_TYPEPROGRAM_DOUBLEWORD)
    if (s_prog_type == (uint32_t) FLASH_TYPEPROGRAM_DOUBLEWORD) {
      // DOUBLEWORD (G474): HAL_FLASH_Program 的 Data 参数是 32-bit,
      // 需拆两次 WORD 写 (低 32 位 + 高 32 位), 地址 +4
      uint32_t low32 = (uint32_t)(word & 0xFFFFFFFFu);
      uint32_t high32 = (uint32_t)(word >> 32u);
      if (HAL_FLASH_Program((uint32_t) FLASH_TYPEPROGRAM_WORD,
                            addr + written, low32) != HAL_OK ||
          HAL_FLASH_Program((uint32_t) FLASH_TYPEPROGRAM_WORD,
                            addr + written + 4u, high32) != HAL_OK) {
        HAL_FLASH_Lock();
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
        if (dcache_en) SCB_EnableDCache();
#endif
#if defined(__ICACHE_PRESENT) && (__ICACHE_PRESENT == 1U)
        if (icache_en) SCB_EnableICache();
#endif
        return -1;
      }
    } else
#endif
    {
      if (HAL_FLASH_Program(s_prog_type, addr + written, (uint32_t) word) != HAL_OK) {
        HAL_FLASH_Lock();
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
        if (dcache_en) SCB_EnableDCache();
#endif
#if defined(__ICACHE_PRESENT) && (__ICACHE_PRESENT == 1U)
        if (icache_en) SCB_EnableICache();
#endif
        return -1;
      }
    }
    written += unit;
  }

  HAL_FLASH_Lock();

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  if (dcache_en) SCB_EnableDCache();
#endif
#if defined(__ICACHE_PRESENT) && (__ICACHE_PRESENT == 1U)
  if (icache_en) SCB_EnableICache();
#endif
  return 0;
}

int bsp_flash_read(BspFlash *me, uint32_t offset, uint8_t *buf, uint32_t len) {
  (void) me;
  if (buf == NULL || len == 0u) {
    return 0;
  }
  if (offset + len > s_size) {
    return -1;
  }
  memcpy(buf, (const void *)(s_base + offset), len);
  return 0;
}

uint32_t bsp_flash_min_erase_size(BspFlash *me) {
  (void) me;
  return s_min_erase;
}

uint32_t bsp_flash_min_write_size(BspFlash *me) {
  (void) me;
  return s_min_write;
}

uint32_t bsp_flash_size(BspFlash *me) {
  (void) me;
  return s_size;
}
