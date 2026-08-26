/*
 * App 固件链接脚本模板 (cmake/app_flash.ld)
 *
 * 用法: App 固件用本脚本 — Flash 起始 = MCU Flash 基址 + BOOTLOADER_SIZE,
 *       与 Bootloader (bootloader_flash.ld) 共享同一 MCU Flash, 分区不重叠.
 *
 * 两个独立固件 (共享 HardC 库, 不同入口 + 不同链接脚本):
 *   - Bootloader: 入口 bootloader_main, Flash 起始 = 0x08000000
 *   - App:        入口 app_main, Flash 起始 = 页对齐地址 (P1: 页对齐而非 Bootloader 大小简单相加)
 *
 * 关键: App 链接脚本的 Flash 起始必须与 Bootloader 中 bsp_jump_to_app 的
 *       app_addr 一致 (boot_loader.cfg.app_addr).
 *
 * 目标 MCU 差异 (由 YmaC 从 flash_map 生成具体值, 见 flash_map_gen.py 页对齐校验 P1):
 *   - STM32F334:  Flash 0x08000000,  64KB, 页 16KB — App 起始 0x08004000 (Bootloader 16K 整页)
 *   - STM32G474:  Flash 0x08000000, 512KB, 页 2KB  — App 起始 0x08008000
 */

/* App 占用的 Flash (起始 = 页对齐, 由 flash_map_gen 按 page_size 注入; 此值仅模板默认) */
__FLASH_ORIGIN = 0x08004000;  /* F334: App 页对齐起始 (16K 页); G474: 0x08008000 (2K 页) */
__FLASH_LENGTH = 40K;         /* F334: 64K-16K(boot整页)-8K(param); G474: 480K — 由 YmaC 注入 */

/* RAM (App 完整, 用全部 RAM) */
__RAM_ORIGIN = 0x20000000;
__RAM_LENGTH = 12K;           /* F334: 12KB; G474: 128KB — 由 YmaC 注入 */

MEMORY
{
  FLASH (rx) : ORIGIN = __FLASH_ORIGIN, LENGTH = __FLASH_LENGTH
  RAM (xrw)  : ORIGIN = __RAM_ORIGIN,  LENGTH = __RAM_LENGTH
}

/* 最高 RAM 地址 = 栈顶 */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* 最小堆/栈大小 (App 控制环用较大栈) */
__min_heap_size = 0;
__min_stack_size = 0x800;     /* 2KB 栈 (控制环+通信) */

/* 定义输出段 */
SECTIONS
{
  /* 中断向量表 (放最前, App 的向量表; bsp_jump_to_app 已设 VTOR 指向此) */
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
