/*
 * Bootloader 固件链接脚本模板 (cmake/bootloader_flash.ld)
 *
 * 用法: Bootloader 固件用本脚本 — Flash 起始 = MCU Flash 基址 (0x08000000),
 *       App 固件用 app_flash.ld — Flash 起始 = 基址 + BOOTLOADER_SIZE.
 *
 * 两个独立固件 (共享 HardC 库, 不同入口 + 不同链接脚本):
 *   - Bootloader: 入口 bootloader_main, Flash 起始 = 0x08000000
 *   - App:        入口 app_main, Flash 起始 = 0x08000000 + BOOTLOADER_SIZE
 *
 * 目标 MCU 差异 (由 YmaC 从 flash_map 生成具体值替代 __FLASH_ORIGIN/__LENGTH):
 *   - STM32F334:  Flash 0x08000000,  64KB (F334C8/R8), 页 16KB
 *   - STM32G474:  Flash 0x08000000, 512KB (G474RE),   页 2KB
 *
 * 建议 BOOTLOADER_SIZE (Bootloader 占用):
 *   - F334 (64KB):  Bootloader 8KB,  App 从 0x08002000 起 (56KB)
 *   - G474 (512KB): Bootloader 32KB, App 从 0x08008000 起 (480KB)
 *   (具体值由 YmaC flash_map 注入)
 */

/* Bootloader 占用的 Flash (起始 = 基址, 长度 = BOOTLOADER_SIZE) */
__FLASH_ORIGIN = 0x08000000;
__FLASH_LENGTH = 8K;    /* F334 建议 8KB; G474 建议 32KB — 由 YmaC 注入 */

/* RAM (Bootloader 极简, 用少量 RAM 即可) */
__RAM_ORIGIN = 0x20000000;
__RAM_LENGTH = 8K;      /* F334: 12KB; G474: 128KB — 由 YmaC 注入 */

MEMORY
{
  FLASH (rx) : ORIGIN = __FLASH_ORIGIN, LENGTH = __FLASH_LENGTH
  RAM (xrw)  : ORIGIN = __RAM_ORIGIN,  LENGTH = __RAM_LENGTH
}

/* 最高 RAM 地址 = 栈顶 */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* 最小堆/栈大小 (链接错误检查用) */
__min_heap_size = 0;
__min_stack_size = 0x400;   /* 1KB 栈 (Bootloader 极简) */

/* 定义输出段 */
SECTIONS
{
  /* 中断向量表 (放最前, Bootloader 的向量表) */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  /* 程序代码 */
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  /* 常量数据 */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab (READONLY) :
  {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >FLASH

  .ARM (READONLY) :
  {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >FLASH

  .preinit_array (READONLY) :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .init_array (READONLY) :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .fini_array (READONLY) :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  /* 已初始化数据 (LMA 在 Flash, VMA 在 RAM, 启动时拷贝) */
  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    *(.RamFunc)
    *(.RamFunc*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  /* 未初始化数据 */
  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  /* 堆栈检查区 (确保 heap+stack 不超出 RAM) */
  ._user_heap_stack (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + __min_heap_size;
    . = . + __min_stack_size;
    . = ALIGN(8);
  } >RAM

  /* 丢弃标准库信息 */
  /DISCARD/ :
  {
    libc.a:* ( * )
    libm.a:* ( * )
    libgcc.a:* ( * )
  }
}
