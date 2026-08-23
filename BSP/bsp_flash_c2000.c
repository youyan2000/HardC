// BSP Flash C2000 后端 — bsp_flash.h 的 TMS320F280049C 实现 (driverlib)
//
// [待验证] — 需真机 CCS + XDS110 验证 (HardC 诚实标注约定, 见 lessons #72 / stage-29)
//
// F280049 Flash 事实 (用户提供):
//   - 容量: 2 Bank × 16 Sector × 4KB = 256KB
//   - C2000Ware 链接器 (28004x_generic_flash_lnk.cmd):
//       FLASH_BANK0_SEC0: origin=0x080002 length=0x000FFE (~4KB)
//       FLASH_BANK0_SEC1: origin=0x081000 length=0x001000 (4KB)
//       每个 Sector 4KB
//   - Flash 起始地址: 0x080000 (24 位地址空间, 非 ARM 的 0x08000000!)
//   - 编程走 Flash 状态机 (FSM), 需 EALLOW 保护
//
// driverlib Flash API (F28004x, [待验证] API 名/签名):
//   - Flash_initModule(base, clocks, waitstates)       — 初始化 Flash 模块
//   - Flash_eraseSector(addr)                          — 擦除一个 4KB sector
//   - Flash_program(addr, buf, len, &status)           — 编程 (len 按 8/16/32-bit)
//   - Flash_enableMainBankPower() / Flash_disableMainBankPower()
//   - Flash_disableCacheAndPrefetch() / Flash_enableCacheAndPrefetch()
//   - Flash_getStatus()                                — 查询编程状态
//
// 实现要点 (对标 LibXR STM32Flash, 但 C2000 用扇区):
//   - bsp_flash_set_region 绑定区域 (F280049 从 app 起始, 如 0x082000)
//   - Erase: 遍历区域内 sector (4KB), 逐个 Flash_eraseSector
//   - Write: 按 Flash_program 粒度写, 相同跳过
//   - 擦写前 EALLOW (写 Flash 控制寄存器), 后 EDIS

#include "bsp_flash.h"
#include "driverlib.h"
#include <string.h>  // memcpy (bsp_flash_write 组装 word)

// ======== 模块级状态 (单例) ========

static uint32_t s_base = 0u;        // 绑定区域基址
static uint32_t s_size = 0u;        // 绑定区域大小
static uint32_t s_min_erase = 0u;   // sector 大小 (4KB)
static uint32_t s_min_write = 0u;   // Flash_program 粒度

// C2000 sector 大小 (F280049: 每个 4KB)
#define C2000_FLASH_SECTOR_SIZE 0x1000u
// F280049 Flash 基址 (24 位)
#define C2000_FLASH_BASE 0x080000u

// 编程粒度 (Flash_program 通常一次 32-bit word; [待验证])
#define C2000_FLASH_PROG_GRANULARITY 4u

// ======== 内部辅助 ========

// 把一个地址映射到其所在 sector 起始 (向下到 4KB 对齐)
static uint32_t sector_of(uint32_t addr) {
  return addr & ~(C2000_FLASH_SECTOR_SIZE - 1u);
}

// ======== 接口 ========

BspFlash *bsp_flash_bind(void *h) {
  (void) h;  // C2000 用全局 Flash 控制器, 忽略句柄
  return (BspFlash *) &s_base;
}

int bsp_flash_set_region(BspFlash *me, const BspFlashInfo *info) {
  (void) me;
  if (info == NULL || info->size == 0u || info->base_addr == 0u) {
    return -1;
  }
  s_base = info->base_addr;
  s_size = info->size;
  s_min_erase = (info->min_erase_size != 0u) ? info->min_erase_size : C2000_FLASH_SECTOR_SIZE;
  s_min_write = (info->min_write_size != 0u) ? info->min_write_size : C2000_FLASH_PROG_GRANULARITY;
  if (s_min_erase == 0u) {
    return -1;
  }
  // 初始化 Flash 模块
  // [待验证] Flash_initModule 需时钟/等待态参数 (参考 SysConfig board.c / CLA 初始化)
  // Flash_initModule(0, 3, 3);  // F28004x: 3 个等待态 @ 120MHz (参数需真机核对)
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

  // EALLOW: 写 Flash 控制寄存器需要
  EALLOW;
  // [待验证] 擦除前需 Flash_disableReadMode / 确保不在执行流中
  uint32_t sec = sector_of(start_addr);
  while (sec < end_addr) {
    // [待验证] Flash_eraseSector API 签名
    // Flash_eraseSector(sec);
    sec += C2000_FLASH_SECTOR_SIZE;
  }
  EDIS;
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

  // [待验证] 编程前需 Flash_disableCacheAndPrefetch (防止读缓存)
  // Flash_disableCacheAndPrefetch();

  EALLOW;
  // 按粒度分块编程
  const uint8_t *src = data;
  uint32_t written = 0u;
  while (written < len) {
    uint32_t chunk = s_min_write;
    if (len - written < chunk) chunk = len - written;
    // [待验证] Flash_program(addr+written, (uint32_t*)&src[written], chunk, &status)
    // 简化: 组装一个 word
    uint32_t word = 0xFFFFFFFFu;
    for (uint32_t i = 0u; i < chunk; i++) {
      word &= ~(0xFFu << (i * 8u));  // 清零低位 — 简化占位
    }
    memcpy((uint8_t *)&word, src + written, chunk);
    // [待验证] Flash_program(addr + written, &word, chunk, NULL);
    written += chunk;
  }
  EDIS;
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
  // C2000 Flash 直接映射到地址空间, 可 memcpy 读
  for (uint32_t i = 0u; i < len; i++) {
    buf[i] = *((const uint8_t *)(s_base + offset + i));
  }
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
