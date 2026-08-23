// BSP 跳转 C2000 后端 — bsp_jump.h 的 TMS320F280049C 实现
//
// [待验证] — 需真机 CCS + XDS110 验证跳转 + PIE 向量表重定位 (HardC 诚实标注约定)
//
// C2000 App 入口约定 (TI F280049):
//   - App 侧 codestartbranch.asm 定义 code_start 标签, 在 App Flash 起始处
//     放置跳转指令 "LB _c_int00" (长分支到 C 运行时入口 _c_int00)
//   - Bootloader 获取 App 入口地址 (code_start 标签值 或 _c_int00 起始地址)
//     后跳转, 合法方式二选一:
//       a) 函数指针直接调用 _c_int00 (本实现采用)
//       b) TI _ExitBoot 风格: 把 EntryAddr 装入 ACC 后执行 LRETR 返回跳转
//
// C2000 与 Cortex-M 的差异:
//   - 无 VTOR / 无"前 8 字节 = MSP+Reset"约定 (那是 ARM 向量表)
//   - C2000 App 入口由链接器 (cmd 文件) 决定, _c_int00 是 C 运行时入口
//   - PIE 向量表在 RAM (可重定位), 跳转前需 DINT 关中断,
//     新 App 必须自行重新配置 PIE 向量表
//
// 本实现: 采用"函数指针调用 _c_int00" (路线 a), 与 codestartbranch.asm 的
//   LB _c_int00 约定一致; 入口地址由调用方 (Bootloader) 提供.

#include "bsp_jump.h"

// 平台能力: C2000 中断控制
#include "driverlib.h"

// C2000 Flash 起始 (24 位地址)
#define C2000_FLASH_BASE 0x080000u

void bsp_jump_to_app(uint32_t app_entry) {
  // 关全局中断 (C28x DINT)
  DINT;

  // [待验证] C2000 跳转方式 (两条合法路线, 本实现用路线 a):
  //   a) 函数指针调用 _c_int00 (当前实现): app_entry 直接作为函数指针调用
  //   b) TI _ExitBoot 风格: 把 EntryAddr 装入 ACC 后 LRETR 返回跳转
  // DINT 已在上方完成; PIE 向量表位于 RAM (跳转后旧向量失效),
  //   新 App 必须自行重装 PIE 向量表
  typedef void (*AppEntryFn)(void);
  AppEntryFn entry = (AppEntryFn) app_entry;
  entry();

  // 不应到达
  for (;;) {
  }
}

int bsp_jump_validate_app(uint32_t app_entry) {
  // 校验 App 入口在 Flash 区 (F280049: 0x080000 ~ 0x09FFFF, 2 Bank × 16 Sector × 8KB = 256KB)
  if (app_entry < C2000_FLASH_BASE) {
    return -1;
  }
  if (app_entry >= C2000_FLASH_BASE + 0x20000u) {  // 上界: 256KB = 0x20000 words (C28x 字寻址; 0x080000..0x09FFFF)
    return -1;
  }
  // [待验证] 可加: 读 App 入口处的跳转指令 (codestartbranch) 校验
  //   但 C2000 指令编码校验复杂, 先用"地址在 Flash 区"的粗略校验
  return 0;
}
