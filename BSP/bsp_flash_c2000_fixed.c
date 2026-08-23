// BSP Flash C2000 后端 (修正版) — bsp_flash.h 的 TMS320F280049C 实现
//
// 本文件基于 TI 官方 SCI Bootloader 示例 (C2000Ware flashapi_ex2) 的真实 F021 API
// 重写, 替换旧 bsp_flash_c2000.c (那版 API 是凭记忆猜测的).
//
// [需要 CCS + XDS110 真机验证] — 链接 F021 Flash API 库 + RAM 执行约束需真机确认
//
// F280049 Flash 事实 (来自 C2000Ware flash_programming_f28004x.h):
//   - 2 Bank × 16 Sector × 8KB = 256KB
//   - Bank0: 0x80000 ~ 0x8FFFF (Bzero_Sector0_start=0x80000 ~ Bzero_Sector15_end=0x8FFFF)
//   - Bank1: 0x90000 ~ 0x9FFFF (Bone_Sector0_start=0x90000 ~ Bone_Sector15_end=0x9FFFF)
//   - 每个 sector 8KB (B_8KSector_u32length = 0x800 = 2048 个 32-bit word)
//   - C2000 地址是 16-bit word 编址, 1 sector = 0x1000 words = 8KB 字节
//
// F021 Flash API (libraries/flash_api/f28004x, v1.56):
//   - Fapi_initializeAPI(F021_CPU0_BASE_ADDRESS, sysclk_mhz)  // 必须在 RAM 执行
//   - Fapi_setActiveFlashBank(Fapi_FlashBank0)                 // F28004x 两 Bank 一次初始化
//   - Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector, addr) // 擦除 sector
//   - Fapi_checkFsmForReady() / Fapi_getFsmStatus()             // 等 FSM 完成/查状态
//   - Fapi_doBlankCheck(addr, length32, &status)                // 空检查
//   - Fapi_issueProgrammingCommand(addr, data16, size_words, 0, 0, Fapi_AutoEccGeneration)
//   - Fapi_doVerify(addr32, len32, &value, &status)             // 校验
//
// 关键约束 (C2000):
//   1. Flash API 函数必须从 RAM 执行 (.TI.ramfunc) — 不能从被擦/写的 Bank 取指
//   2. 编程必须 64-bit 对齐, 且第 3 参 (字数) 只能为 4 或 8 (官方 F28004x 示例);
//      本实现用 C2000_PROGRAM_WORDS=4, Fapi_AutoEccGeneration 自动生成 ECC
//   3. 擦除/编程时不能访问被操作 Bank
//   4. 需要 EALLOW 保护

#include "bsp_flash.h"

// C2000 driverlib — EALLOW/EDIS/NULL/uint8_t 显式包含 (不依赖 --preinclude=driverlib.h)
#include "driverlib.h"

// C2000 F021 Flash API
#include "F021_F28004x_C28x.h"

// SYSCLK MHz — 由工程定义以适配实际时钟 (F280049 @120MHz → 120).
// Falls back to 100MHz if not defined so the file compiles statically.
#ifndef C2000_SYSCLK_MHZ
#define C2000_SYSCLK_MHZ 100u
#endif

// F021 Flash API 是否已链接.
// cmake/HardC.CMake 的 c2000 分支在链接 F021_API_F28004x_FPU32_EABI.lib 时定义
// C2000_FLASH_HAS_F021=1. 本文件无条件调用 Fapi_* 符号, 必须随 F021 库一起链接;
// 未链接库时链接期报 Fapi_* 未定义符号. 下方 0 默认值仅为宏预定义兜底.
#ifndef C2000_FLASH_HAS_F021
#define C2000_FLASH_HAS_F021 0
#endif

// ======== 模块级状态 (单例) ========

static uint32_t s_base = 0u;   // 绑定区域基址 (C2000 16-bit word 地址)
static uint32_t s_size = 0u;   // 绑定区域大小 (字节)

// F280049 sector 大小 (8KB = 0x1000 words; C2000 word=16bit)
#define C2000_SECTOR_SIZE_WORDS 0x1000u  // = 8KB 字节
#define C2000_SECTOR_SIZE_BYTES 0x2000u  // 8KB = 8192 字节
#define C2000_SECTOR_WORDS_32   0x0800u  // 2048 个 32-bit word = 8KB

// F280049 Flash 基址 (24 位, word 寻址)
#define C2000_FLASH_BASE 0x080000u

// 编程 buffer: 4 words (64-bit, ECC 对齐) — 数据必须 4-word 对齐
#define C2000_PROGRAM_WORDS 4u

// ======== 内部辅助 ========

// 绝对地址 → 所在 sector 起始 (向下 8KB 对齐, word 寻址)
static uint32_t sector_of(uint32_t addr) {
  return addr & ~(C2000_SECTOR_SIZE_WORDS - 1u);
}

// ======== 接口 (注意: 擦/写需从 RAM 执行, 见头注释) ========

BspFlash *bsp_flash_bind(void *h) {
  (void) h;
  return (BspFlash *) &s_base;
}

int bsp_flash_set_region(BspFlash *me, const BspFlashInfo *info) {
  (void) me;
  if (info == NULL || info->size == 0u || info->base_addr == 0u) {
    return -1;
  }
  s_base = info->base_addr;
  s_size = info->size;
  // 初始化 F021 Flash API (必须在 RAM 执行). SYSCLK MHz 由调用方经 BspFlashInfo
  // (见上方 C2000_SYSCLK_MHZ 机制) 或 App 宏注入. [真机验证 SYSCLK 值]
  // C2000_FLASH_HAS_F021=1 (CMake c2000 分支链接 F021 库后定义) 时执行初始化;
  // 本文件要求 F021 库参与链接, Fapi_* 路径始终编译.
#if C2000_FLASH_HAS_F021
  Fapi_initializeAPI(F021_CPU0_BASE_ADDRESS, C2000_SYSCLK_MHZ);
  Fapi_setActiveFlashBank(Fapi_FlashBank0);
#endif
  return 0;
}

#pragma CODE_SECTION(bsp_flash_set_region, ".TI.ramfunc")

int bsp_flash_erase(BspFlash *me, uint32_t offset, uint32_t size) {
  (void) me;
  if (size == 0u) return 0;
  if (offset + size > s_size) return -1;
  // 接口的 offset/size 单位是字节, s_base 是 C2000 word 地址.
  // 擦除只能按 8KB sector 进行 (min_erase_size): 要求 offset 与 size 都是 8KB 的
  // 整数倍, 否则拒绝 — 非对齐请求按 floor 折算会静默漏擦尾部 sector.
  if ((offset % C2000_SECTOR_SIZE_BYTES) != 0u) return -1;
  if ((size % C2000_SECTOR_SIZE_BYTES) != 0u) return -1;

  // 字节 offset → word 地址 (offset/2), 向下对齐到 sector 边界
  uint32_t num_sectors = size / C2000_SECTOR_SIZE_BYTES;
  uint32_t sec = sector_of(s_base + offset / 2u);

  EALLOW;
  for (uint32_t s = 0u; s < num_sectors; s++) {
    Fapi_StatusType st;
    Fapi_FlashStatusType fst;
    Fapi_FlashStatusWordType fsw;
    // 擦除 sector
    st = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector, (uint32_t *) sec);
    // 等待 FSM 完成
    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady) {}
    // TI 示例顺序: 先查命令状态, 再查 FMSTAT, 最后空检查
    // (擦除命令的 st 必须在 doBlankCheck 覆盖它之前检查)
    if (st != Fapi_Status_Success) {
      EDIS;
      return -1;
    }
    fst = Fapi_getFsmStatus();
    if (fst != 0u) {
      EDIS;
      return -1;
    }
    // 空检查 (擦除本身带 verify, 此为可选加固)
    st = Fapi_doBlankCheck((uint32_t *) sec, C2000_SECTOR_WORDS_32, &fsw);
    if (st != Fapi_Status_Success) {
      EDIS;
      return -1;
    }
    sec += C2000_SECTOR_SIZE_WORDS;
  }
  EDIS;
  return 0;
}

#pragma CODE_SECTION(bsp_flash_erase, ".TI.ramfunc")

int bsp_flash_write(BspFlash *me, uint32_t offset, const uint8_t *data, uint32_t len) {
  (void) me;
  if (data == NULL || len == 0u) return 0;
  if (offset + len > s_size) return -1;

  EALLOW;
  // C2000 是 16-bit word 寻址; data 是字节流, 需转成 word 缓冲.
  // 编程最小 4 words (64-bit, ECC) — data 每 8 字节一组; pos 是字节偏移,
  // 目标 word 地址 = s_base + (offset + pos) / 2 (须 64-bit 对齐: offset+pos 为 8 的
  // 倍数, 由 min_write_size=8 字节契约保证).
  uint32_t pos = 0u;
  while (pos < len) {
    // 组装 4 个 16-bit word (8 字节)
    uint16_t buf[C2000_PROGRAM_WORDS] = {0xFFFFu, 0xFFFFu, 0xFFFFu, 0xFFFFu};
    uint32_t chunk = len - pos;
    if (chunk > C2000_PROGRAM_WORDS * 2u) chunk = C2000_PROGRAM_WORDS * 2u;
    // 字节 → word (little-endian: byte[2i] = low, byte[2i+1] = high)
    for (uint32_t i = 0u; i < chunk; i++) {
      if ((pos + i) & 1u) {
        buf[(pos + i) / 2u] |= ((uint16_t) data[pos + i]) << 8u;
      } else {
        buf[(pos + i) / 2u] = (uint16_t) data[pos + i];
      }
    }

    Fapi_StatusType st;
    Fapi_FlashStatusType fst;
    Fapi_FlashStatusWordType fsw;
    // F021 C28x 版 Fapi_issueProgrammingCommand 第 3 参 = 16-bit 字数 (u16DataBufferSizeInWords),
    // 官方 F28004x 示例: 长度只能为 4 或 8 个字. buf 是 C2000_PROGRAM_WORDS=4 个字 → 传 4.
    st = Fapi_issueProgrammingCommand((uint32_t *) (s_base + (offset + pos) / 2u), buf,
                                      C2000_PROGRAM_WORDS, 0, 0, Fapi_AutoEccGeneration);
    while (Fapi_checkFsmForReady() == Fapi_Status_FsmBusy) {}
    fst = Fapi_getFsmStatus();
    if (st != Fapi_Status_Success || fst != 0u) {
      EDIS;
      return -1;
    }
    // 编程后校验: Fapi_doVerify 的 u32Length 是 32-bit 字数.
    // 本次编程 4 个 16-bit 字 = 8 字节 = 2 个 32-bit 字 → 传 C2000_PROGRAM_WORDS/2 = 2.
    // 校验缓冲 = 实际写入的 2 个 32-bit word (TI 官方示例传数据缓冲, 不传 NULL).
    uint32_t check[C2000_PROGRAM_WORDS / 2u];
    check[0] = (uint32_t) buf[0] | ((uint32_t) buf[1] << 16u);
    check[1] = (uint32_t) buf[2] | ((uint32_t) buf[3] << 16u);
    st = Fapi_doVerify((uint32_t *) (s_base + (offset + pos) / 2u),
                       C2000_PROGRAM_WORDS / 2u, check, &fsw);
    if (st != Fapi_Status_Success) {
      EDIS;
      return -1;
    }
    pos += chunk;
  }
  EDIS;
  return 0;
}

#pragma CODE_SECTION(bsp_flash_write, ".TI.ramfunc")

int bsp_flash_read(BspFlash *me, uint32_t offset, uint8_t *buf, uint32_t len) {
  (void) me;
  if (buf == NULL || len == 0u) return 0;
  if (offset + len > s_size) return -1;
  // C2000 Flash 直接映射, 可按 word 读; 接口 offset 是字节偏移, s_base 是 word 地址.
  // 目标 word = s_base + (offset + i) / 2, 高低字节由 (offset + i) & 1 决定.
  for (uint32_t i = 0u; i < len; i++) {
    uint32_t target_word = s_base + (offset + i) / 2u;
    uint16_t w = *((volatile uint16_t *) target_word);
    if ((offset + i) & 1u) {
      buf[i] = (uint8_t) (w >> 8u);
    } else {
      buf[i] = (uint8_t) (w & 0xFFu);
    }
  }
  return 0;
}

uint32_t bsp_flash_min_erase_size(BspFlash *me) {
  (void) me;
  return C2000_SECTOR_SIZE_BYTES;  // 8KB
}

uint32_t bsp_flash_min_write_size(BspFlash *me) {
  (void) me;
  return C2000_PROGRAM_WORDS * 2u;  // 4 words = 8 字节
}

uint32_t bsp_flash_size(BspFlash *me) {
  (void) me;
  return s_size;
}
