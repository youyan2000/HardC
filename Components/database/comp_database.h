// 闪存键值数据库 — 主备双块 + 顺序写入 (纯C, 零 malloc)
//
// 来源: LibXR src/middleware/database/{interface,raw_sequential}.{hpp,cpp}
//       (DatabaseRawSequential — 适用于不支持逆序写入的 Flash)
// 译为 HardC 纯C 版本: 零 malloc (缓冲调用者提供), ops 函数指针表访问 Flash (可 host 单测)
//
// 核心机制 (对齐 LibXR DatabaseRawSequential):
//   1. 主备双块 (防断电损坏): block_size = 总容量一半; Save 先写备份块再写主块;
//      Init 校验: 任一损坏从另一块恢复, 都损坏则重新初始化.
//   2. 顺序写入: 新键追加到末尾 (不覆盖旧键); 更新键要求数据长度一致;
//      数据相同则跳过 (省 Flash 寿命).
//   3. KeyInfo 编码: 4 字节 = 1bit 后继键 + 7bit 键名长度 + 24bit 数据长度.
//   4. Flash 头 (FLASH_HEADER) + 块末尾校验字节: 检测块损坏.
//   5. 缓冲区保存整块内容, 所有操作先改 buffer 再 Save 整体写 Flash
//      (依赖底层 skip-same 省写次数).
//
// Flash 依赖: 通过 CompFlashOps 函数指针表, 不直接依赖具体 BSP 后端.
//   绑定 bsp_flash (STM32/C2000) 时只需写一个适配器 (见头注释示例).
//   可 host 单测: 传 mock FlashOps (RAM 模拟) 即可.

#ifndef COMP_DATABASE_H
#define COMP_DATABASE_H

#include <stdint.h>
#include <stddef.h>
#include "comp_error_code.h"

#ifdef __cplusplus
extern "C" {
#endif

// ======== Flash 操作接口 (函数指针表, HardC ops 风格) ========
// 对标 LibXR `Flash` 抽象; 由调用者绑定具体后端 (bsp_flash 适配器 / 模拟 Flash).
typedef struct {
  void *ctx;  // 后端上下文 (如 BspFlash*; mock 时传模拟对象)
  // 擦除 [offset, offset+size) — 相对绑定区域基址, 按扇区/页粒度
  int (*erase)(void *ctx, uint32_t offset, uint32_t size);
  // 写入 [offset, offset+len)
  int (*write)(void *ctx, uint32_t offset, const uint8_t *data, uint32_t len);
  // 读取 [offset, offset+len)
  int (*read)(void *ctx, uint32_t offset, uint8_t *buf, uint32_t len);
  // 最小擦除单元 (STM32 扇区/页; C2000 sector) — 决定 block_size 粒度
  uint32_t (*min_erase_size)(void *ctx);
  // 绑定区域总大小 (字节)
  uint32_t (*size)(void *ctx);
} CompFlashOps;

// ======== 数据库实例 (值包含, 零 malloc) ========
typedef struct {
  const CompFlashOps *flash;  // Flash 操作接口
  uint8_t *buffer;            // 数据缓冲 (调用者提供, 长度 >= buffer_size)
  uint32_t buffer_size;       // 缓冲 / 单块大小 (主块或备份块)
  uint32_t block_size;        // 单块大小 = 总容量/2 (按 min_erase_size 对齐)
} CompDatabase;

// ======== API ========

// 初始化: 绑定 Flash + 缓冲, 校验/恢复主备块, 加载数据到 buffer.
//   buffer:    调用者提供 (长度 >= buffer_size), 零 malloc
//   buffer_size: 数据缓冲大小 (须 <= 总容量/2; 决定单块存储容量)
// 返回 ERR_OK / ERR_ARG / ERR_NO_MEM 等
int comp_database_init(CompDatabase *me, const CompFlashOps *flash,
                       uint8_t *buffer, uint32_t buffer_size);

// 获取键值: 按 name 查找, 读到 data (size 必须等于存储长度, 否则 ERR_SIZE)
// 返回 ERR_OK / ERR_NOT_FOUND / ERR_SIZE / ERR_FAILED
int comp_database_get(CompDatabase *me, const char *name,
                      void *data, uint32_t size);

// 设置键值: 键存在则更新 (长度须一致, 数据相同跳过); 不存在则失败
// 返回 ERR_OK / ERR_NOT_FOUND / ERR_SIZE / ERR_FULL
int comp_database_set(CompDatabase *me, const char *name,
                      const void *data, uint32_t size);

// 添加新键: 键不存在则追加到末尾; 已存在则等效 set
// 返回 ERR_OK / ERR_FULL / ERR_SIZE
int comp_database_add(CompDatabase *me, const char *name,
                      const void *data, uint32_t size);

// 保存当前 buffer 到 Flash (先写备份块再写主块, 防断电)
int comp_database_save(CompDatabase *me);

// 从 Flash 主块加载数据到 buffer
int comp_database_load(CompDatabase *me);

// 还原: 清空全部数据 (擦除两块 + 写初始头), 恢复出厂
int comp_database_restore(CompDatabase *me);

#ifdef __cplusplus
}
#endif

#endif  // COMP_DATABASE_H
