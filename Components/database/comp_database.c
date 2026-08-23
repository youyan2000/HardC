// 闪存键值数据库实现 — CompDatabase 纯C (主备双块 + 顺序写入)
//
// 本文件是 comp_database.c 的修正版 (交付报告建议的 comp_database_fixed.c):
//   修复: init 主块损坏从备份恢复时, while 循环内每块都擦除会覆盖前块 — 
//         先整体擦除一次, 再逐块从备份拷贝.
//   统一: 2 空格缩进 (符合 HardC 规范).
//   其余逻辑与 comp_database.c 等价.
//
// 翻译自 LibXR DatabaseRawSequential (src/middleware/database/raw_sequential.cpp)
// 全部经 CompFlashOps 访问 Flash, 零 malloc, 可 host 单测.
//
// 存储布局 (每个块):
//   [FlashInfo][KeyInfo+name+data][KeyInfo+name+data]...[校验字节]
//   FlashInfo = { header(4B), KeyInfo(4B) }  — 块头
//   KeyInfo   = 4B: 1bit next + 7bit name-len + 24bit data-size
//   校验字节  在 buffer 末尾 (buffer[buffer_size-1])
//
// 主备双块 (防断电):
//   主块 offset [0, block_size), 备份块 offset [block_size, 2*block_size)
//   Save: 先写备份 (擦+写), 再写主块 (擦+写) — 任何时候断电至少一块完整
//   Init: 备份未初始化/损坏 → 重置备份; 主块损坏 → 从备份恢复或重置

#include "comp_database.h"

#include <string.h>
#include <stddef.h>

// ======== 常量 (对齐 LibXR) ========

#define DB_VERSION 3u
#define DB_FLASH_HEADER (0x12345678u + DB_VERSION)  // 块头标识
#define DB_CHECKSUM_BYTE 0x56u                       // 块末尾校验字节

// OFFSETOF 宏 (避免依赖 stddef offsetof 对非标准布局的坑; KeyInfo 是 4 字节原生)
#define DB_OFFSETOF(type, member) ((size_t)(&(((type *)0)->member)))

// ======== KeyInfo 编码/解码 (4 字节 uint32_t, 对齐 LibXR) ========

// raw_data 布局: bit31=next_key, bit30-24=name_len(7bit), bit23-0=data_size(24bit)
static uint32_t db_keyinfo_make(uint8_t next_key, uint8_t name_len, uint32_t data_size) {
  uint32_t raw = 0u;
  if (next_key) raw |= (1u << 31);
  raw |= ((uint32_t)(name_len & 0x7Fu) << 24);
  raw |= (data_size & 0x00FFFFFFu);
  return raw;
}

static uint8_t db_keyinfo_next(uint32_t ki) { return (uint8_t)((ki >> 31) & 0x1u); }
static uint8_t db_keyinfo_name_len(uint32_t ki) { return (uint8_t)((ki >> 24) & 0x7Fu); }
static uint32_t db_keyinfo_data_size(uint32_t ki) { return ki & 0x00FFFFFFu; }

// ======== Flash 封装 (失败返回错误; 对标 LibXR ReadFlashOrExit 但返回错误而非 REQUIRE) ========

static int db_flash_read(CompDatabase *me, uint32_t offset, void *buf, uint32_t len) {
  if (!me->flash || !me->flash->read) return ERR_FAILED;
  return me->flash->read(me->flash->ctx, offset, (uint8_t *)buf, len);
}

static int db_flash_write(CompDatabase *me, uint32_t offset, const void *data, uint32_t len) {
  if (!me->flash || !me->flash->write) return ERR_FAILED;
  return me->flash->write(me->flash->ctx, offset, (const uint8_t *)data, len);
}

static int db_flash_erase(CompDatabase *me, uint32_t offset, uint32_t size) {
  if (!me->flash || !me->flash->erase) return ERR_FAILED;
  return me->flash->erase(me->flash->ctx, offset, size);
}

// ======== 块操作 (offset 0=MAIN, block_size=BACKUP) ========

// 块是否已初始化: 块头 header == FLASH_HEADER
static int db_is_block_inited(CompDatabase *me, uint32_t block_offset) {
  uint32_t header = 0u;
  if (db_flash_read(me, block_offset, &header, sizeof(header)) != ERR_OK) {
    return 0;
  }
  return header == DB_FLASH_HEADER;
}

// 块是否为空: 块头 key 的 name_len == 0
static int db_is_block_empty(CompDatabase *me, uint32_t block_offset) {
  uint32_t header = 0u, keyinfo = 0u;
  if (db_flash_read(me, block_offset, &header, sizeof(header)) != ERR_OK) return 0;
  if (db_flash_read(me, block_offset + sizeof(header), &keyinfo, sizeof(keyinfo)) != ERR_OK) {
    return 0;
  }
  return db_keyinfo_name_len(keyinfo) == 0u;
}

// 块是否损坏: 末尾校验字节不对
static int db_is_block_error(CompDatabase *me, uint32_t block_offset) {
  uint8_t chk = 0u;
  if (db_flash_read(me, block_offset + me->buffer_size - 1u, &chk, 1u) != ERR_OK) {
    return 1;
  }
  return chk != DB_CHECKSUM_BYTE;
}

// 初始化一个块 (擦除 + 写当前 buffer 内容)
static int db_init_block(CompDatabase *me, uint32_t block_offset) {
  int ret = db_flash_erase(me, block_offset, me->block_size);
  if (ret != ERR_OK) return ret;
  return db_flash_write(me, block_offset, me->buffer, me->buffer_size);
}

// 填充初始 buffer (FlashInfo 头 + 空 key + 校验字节)
static void db_make_initial_buffer(CompDatabase *me) {
  uint32_t header = DB_FLASH_HEADER;
  uint32_t keyinfo = db_keyinfo_make(0, 0, 0);  // 空键 (name_len=0)
  // 先全部置 0xFF (Flash 擦除态)
  memset(me->buffer, 0xFF, me->buffer_size);
  if (me->buffer_size >= 8u) {
    memcpy(me->buffer, &header, sizeof(header));
    memcpy(me->buffer + 4u, &keyinfo, sizeof(keyinfo));
    me->buffer[me->buffer_size - 1u] = DB_CHECKSUM_BYTE;
  }
}

// ======== 键扫描 ========

// 块内是否有后继键
static int db_has_next_key(CompDatabase *me, uint32_t offset) {
  uint32_t ki = 0u;
  if (db_flash_read(me, offset, &ki, sizeof(ki)) != ERR_OK) return 0;
  return db_keyinfo_next(ki) != 0u;
}

// 键总大小: KeyInfo + name + data
static uint32_t db_get_key_size(CompDatabase *me, uint32_t offset) {
  uint32_t ki = 0u;
  if (db_flash_read(me, offset, &ki, sizeof(ki)) != ERR_OK) return 0u;
  return 4u + (uint32_t)db_keyinfo_name_len(ki) + db_keyinfo_data_size(ki);
}

// 下一个键偏移
static uint32_t db_get_next_key(CompDatabase *me, uint32_t offset) {
  return offset + db_get_key_size(me, offset);
}

// 比较键名: 返回 0=匹配 (对齐 LibXR 语义: 返回 0 表示相同)
static int db_key_name_compare(CompDatabase *me, uint32_t offset, const char *name) {
  uint32_t ki = 0u;
  if (db_flash_read(me, offset, &ki, sizeof(ki)) != ERR_OK) return -1;
  uint32_t name_len = db_keyinfo_name_len(ki);
  for (uint32_t i = 0u; i < name_len; i++) {
    uint8_t ch = 0u;
    if (db_flash_read(me, offset + 4u + i, &ch, 1u) != ERR_OK) return -1;
    if (ch != (uint8_t)name[i]) {
      return 1;  // 不同
    }
  }
  return 0;  // 相同
}

// 比较数据: 返回 0=相同 (对齐 LibXR 语义)
static int db_key_data_compare(CompDatabase *me, uint32_t offset,
                               const void *data, uint32_t size) {
  uint32_t ki = 0u;
  if (db_flash_read(me, offset, &ki, sizeof(ki)) != ERR_OK) return -1;
  uint32_t data_off = offset + 4u + (uint32_t)db_keyinfo_name_len(ki);
  const uint8_t *src = (const uint8_t *)data;
  for (uint32_t i = 0u; i < size; i++) {
    uint8_t ch = 0u;
    if (db_flash_read(me, data_off + i, &ch, 1u) != ERR_OK) return -1;
    if (ch != src[i]) {
      return 1;  // 不同
    }
  }
  return 0;  // 相同
}

// 修改某键的 next 标志 (写入当前 buffer 对应位置, 之后 Save 生效)
static int db_set_next_key_exist(CompDatabase *me, uint32_t offset, uint8_t exist) {
  uint32_t ki = 0u;
  if (db_flash_read(me, offset, &ki, sizeof(ki)) != ERR_OK) return ERR_FAILED;
  uint32_t new_ki = db_keyinfo_make(exist, db_keyinfo_name_len(ki), db_keyinfo_data_size(ki));
  if (offset + 4u <= me->buffer_size) {
    memcpy(me->buffer + offset, &new_ki, sizeof(new_ki));
  }
  return ERR_OK;
}

// 找主块最后一个键的偏移 (0=空)
static uint32_t db_get_last_key(CompDatabase *me) {
  if (db_is_block_empty(me, 0u)) {
    return 0u;
  }
  uint32_t offset = DB_OFFSETOF(struct { uint32_t header; uint32_t key; }, key);
  while (db_has_next_key(me, offset)) {
    offset = db_get_next_key(me, offset);
  }
  return offset;
}

// 搜索键: 返回键偏移 (0=未找到)
static uint32_t db_search_key(CompDatabase *me, const char *name) {
  if (db_is_block_empty(me, 0u)) {
    return 0u;
  }
  uint32_t offset = DB_OFFSETOF(struct { uint32_t header; uint32_t key; }, key);
  while (1) {
    if (db_key_name_compare(me, offset, name) == 0) {
      return offset;
    }
    if (!db_has_next_key(me, offset)) {
      break;
    }
    offset = db_get_next_key(me, offset);
  }
  return 0u;
}

// ======== 核心: Add / Set / Get ========

static int db_set_key_offset(CompDatabase *me, uint32_t offset,
                             const void *data, uint32_t size) {
  if (offset == 0u) return ERR_FAILED;
  uint32_t ki = 0u;
  if (db_flash_read(me, offset, &ki, sizeof(ki)) != ERR_OK) return ERR_FAILED;
  if (db_keyinfo_data_size(ki) == size) {
    // 长度一致: 若数据不同才更新 buffer 并 Save
    if (db_key_data_compare(me, offset, data, size) != 0) {
      uint32_t data_off = offset + 4u + (uint32_t)db_keyinfo_name_len(ki);
      if (data_off + size <= me->buffer_size) {
        memcpy(me->buffer + data_off, data, size);
        return comp_database_save(me);
      }
      return ERR_OUT_OF_RANGE;
    }
    return ERR_OK;  // 数据相同, 跳过
  }
  return ERR_SIZE;  // 长度不一致, 不支持原位改长度
}

static int db_add_key(CompDatabase *me, const char *name,
                      const void *data, uint32_t size) {
  uint32_t exist = db_search_key(me, name);
  if (exist != 0u) {
    return db_set_key_offset(me, exist, data, size);
  }

  uint32_t name_len = (uint32_t)strlen(name) + 1u;  // 含 '\0'
  uint32_t last_off = db_get_last_key(me);
  uint32_t key_off = last_off ? db_get_next_key(me, last_off)
                              : DB_OFFSETOF(struct { uint32_t header; uint32_t key; }, key);
  uint32_t end_off = key_off + 4u + name_len + size;
  if (end_off > me->buffer_size - 1u) {  // 预留校验字节
    return ERR_FULL;
  }

  // 写入 buffer: name + data
  memcpy(me->buffer + key_off + 4u, name, name_len);
  memcpy(me->buffer + key_off + 4u + name_len, data, size);
  // 写 KeyInfo
  uint32_t ki = db_keyinfo_make(0, (uint8_t)name_len, size);
  memcpy(me->buffer + key_off, &ki, sizeof(ki));
  // 若有前一个键, 置其 next 标志
  if (last_off != 0u) {
    int ret = db_set_next_key_exist(me, last_off, 1u);
    if (ret != ERR_OK) return ret;
  }
  return comp_database_save(me);
}

// ======== 公开 API ========

int comp_database_init(CompDatabase *me, const CompFlashOps *flash,
                       uint8_t *buffer, uint32_t buffer_size) {
  if (!me || !flash || !buffer || buffer_size < 2u) return ERR_ARG;
  // 校验容量: 两倍 block_size 必须 <= Flash 总容量, buffer <= 半容量
  uint32_t flash_size = flash->size ? flash->size(flash->ctx) : 0u;
  uint32_t min_erase = flash->min_erase_size ? flash->min_erase_size(flash->ctx) : 1u;

  me->flash = flash;
  me->buffer = buffer;
  me->buffer_size = buffer_size;  // 单块数据容量 (含头+校验)

  // block_size = 总容量/2, 向下对齐到 min_erase_size
  uint32_t half = flash_size / 2u;
  me->block_size = (half / min_erase) * min_erase;
  if (me->block_size == 0u || me->block_size < buffer_size) {
    return ERR_SIZE;
  }

  // 构造初始 buffer (擦除态 + FlashInfo + 校验字节)
  db_make_initial_buffer(me);

  // 1. 备份块: 未初始化或损坏 → 重置 (防主备都坏的极端)
  if (!db_is_block_inited(me, me->block_size) ||
      db_is_block_error(me, me->block_size)) {
    int ret = db_init_block(me, me->block_size);
    if (ret != ERR_OK) return ret;
  }
  // 2. 主块: 损坏 → 从备份恢复或重置
  if (db_is_block_error(me, 0u)) {
    if (db_is_block_empty(me, me->block_size)) {
      // 备份也是空 → 重置主块为初始
      int ret = db_init_block(me, 0u);
      if (ret != ERR_OK) return ret;
    } else {
      // 主块损坏: 从备份恢复 — 先整体擦除主块一次, 再逐块从备份拷贝
      // (原 comp_database.c 的 bug: while 循环内每块都擦除会覆盖前块, 这里已修正)
      int ret = db_flash_erase(me, 0u, me->block_size);
      if (ret != ERR_OK) return ret;
      uint8_t tmp[32];  // 分块拷贝, 避免大栈
      uint32_t off = 0u;
      while (off < me->buffer_size) {
        uint32_t chunk = me->buffer_size - off;
        if (chunk > sizeof(tmp)) chunk = sizeof(tmp);
        ret = db_flash_read(me, me->block_size + off, tmp, chunk);
        if (ret != ERR_OK) return ret;
        ret = db_flash_write(me, off, tmp, chunk);  // 不再循环内擦除
        if (ret != ERR_OK) return ret;
        off += chunk;
      }
    }
  }
  // 3. 加载主块到 buffer
  return comp_database_load(me);
}

int comp_database_get(CompDatabase *me, const char *name,
                      void *data, uint32_t size) {
  uint32_t off = db_search_key(me, name);
  if (off == 0u) return ERR_NOT_FOUND;
  uint32_t ki = 0u;
  if (db_flash_read(me, off, &ki, sizeof(ki)) != ERR_OK) return ERR_FAILED;
  if (db_keyinfo_data_size(ki) != size) return ERR_SIZE;
  uint32_t data_off = off + 4u + (uint32_t)db_keyinfo_name_len(ki);
  return db_flash_read(me, data_off, data, size);
}

int comp_database_set(CompDatabase *me, const char *name,
                      const void *data, uint32_t size) {
  uint32_t off = db_search_key(me, name);
  if (off == 0u) return ERR_NOT_FOUND;
  return db_set_key_offset(me, off, data, size);
}

int comp_database_add(CompDatabase *me, const char *name,
                      const void *data, uint32_t size) {
  return db_add_key(me, name, data, size);
}

int comp_database_save(CompDatabase *me) {
  if (!me->flash || !me->flash->erase || !me->flash->write) return ERR_FAILED;
  // 先写备份块
  int ret = db_flash_erase(me, me->block_size, me->block_size);
  if (ret != ERR_OK) return ret;
  ret = db_flash_write(me, me->block_size, me->buffer, me->buffer_size);
  if (ret != ERR_OK) return ret;
  // 再写主块
  ret = db_flash_erase(me, 0u, me->block_size);
  if (ret != ERR_OK) return ret;
  return db_flash_write(me, 0u, me->buffer, me->buffer_size);
}

int comp_database_load(CompDatabase *me) {
  return db_flash_read(me, 0u, me->buffer, me->buffer_size);
}

int comp_database_restore(CompDatabase *me) {
  db_make_initial_buffer(me);
  int ret = db_flash_erase(me, 0u, me->block_size);
  if (ret != ERR_OK) return ret;
  ret = db_flash_erase(me, me->block_size, me->block_size);
  if (ret != ERR_OK) return ret;
  ret = db_flash_write(me, 0u, me->buffer, me->buffer_size);
  if (ret != ERR_OK) return ret;
  return db_flash_write(me, me->block_size, me->buffer, me->buffer_size);
}
