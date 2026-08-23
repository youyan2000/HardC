// 引导加载模块实现 (完善版) — ModBootloader
//
// 本文件是 mod_bootloader 的完整实现 (对标 LibXR Database 持久化 + STM32Flash):
//   - 字节流组帧状态机 (UART 必需, 处理跨帧重组): SYNC→HEADER→DATA→CRC
//   - 边收边写: 每块校验 CRC → 写 Flash (offset = block * MOD_BL_MAX_BLOCK_SIZE)
//   - 首块触发 App 区整区擦除
//   - 全部收完 → VERIFY → 清除升级标志 → 跳转 App
//   - 升级标志用 comp_database (主备双块防断电)
//
// 三上下文 (严格遵守 HardC):
//   CTX_HMI  : mod_bootloader_rx — 字节/帧入 rx_ring, 非阻塞
//   CTX_MAIN : mod_bootloader_poll — 组帧 + CRC + Flash 写 (IO_SYNC), 跳转
//   board_init: mod_bootloader_should_jump — 读 Database 升级标志
//
// 升级协议帧:
//   [0xAA][len_hi][len_lo][block_hi][block_lo][data...][crc32(4B)]
//   len    = data 长度 (0..MOD_BL_MAX_BLOCK_SIZE)
//   block  = 块号; 首块 (block==0) 的 data 前 4 字节 = 固件总长度 (大端)
//   crc32  = CRC32(len_hi+len_lo+block_hi+block_lo+data), 覆盖不含帧头

#include "mod_bootloader.h"

#include <string.h>

// ======== 协议常量 ========

#define MOD_BL_FRAME_HEADER_BYTE 0xAAu
#define MOD_BL_FRAME_FIXED (2u + 2u + 4u)  // len(2) + block(2) + crc(4), 不含帧头/data

// ======== Database FlashOps 适配 (bsp_flash → CompFlashOps) ========

static int db_erase(void *ctx, uint32_t offset, uint32_t size) {
  return bsp_flash_erase((BspFlash *) ctx, offset, size);
}
static int db_write(void *ctx, uint32_t offset, const uint8_t *data, uint32_t len) {
  return bsp_flash_write((BspFlash *) ctx, offset, data, len);
}
static int db_read(void *ctx, uint32_t offset, uint8_t *buf, uint32_t len) {
  return bsp_flash_read((BspFlash *) ctx, offset, buf, len);
}
static uint32_t db_min_erase(void *ctx) {
  return bsp_flash_min_erase_size((BspFlash *) ctx);
}
static uint32_t db_size(void *ctx) {
  return bsp_flash_size((BspFlash *) ctx);
}

// ======== 内部工具 ========

// CRC32 (标准: init 0xFFFFFFFF, 结果 ^ 0xFFFFFFFF)
static uint32_t crc32_buf(const uint8_t *data, uint32_t len) {
  uint32_t crc = crc32_calc(data, len, 0xFFFFFFFFu);
  return crc ^ 0xFFFFFFFFu;
}

// 写固件数据到 Flash — 连续偏移, 基于当前已写长度 (fw_received),
// 而非 block*BLOCK_SIZE (固件数据紧凑连续).
//   block==0 时 data 前 4 字节是固件总长度元数据, 不写入 Flash (跳过).
// 返回: 0=成功, 非0=错误; 不推进 fw_received (由 process_frame 统一推进).
static int write_block_data(ModBootloader *me, uint16_t block_id,
                            const uint8_t *data, uint16_t len) {
  // 首块跳过 4 字节固件长度字段 (元数据不写入固件区)
  uint32_t data_off = (block_id == 0u) ? 4u : 0u;
  if (len <= data_off) {
    return -1;  // 首块至少 4 字节长度 + 数据
  }
  uint32_t payload_len = len - data_off;
  const uint8_t *payload = &data[data_off];

  // 连续偏移 = 当前已写固件长度
  uint32_t offset = me->fw_received;
  if (offset + payload_len > me->cfg.app_max_size) {
    return -1;
  }
  return bsp_flash_write(me->flash, offset, payload, payload_len);
}

// 组帧状态机: 喂一个字节, 累积到 frame_buf, 返回
//   1 = 已收满一个完整帧 (frame_buf 含 data+crc, frame_expect 已定, 可处理)
//   0 = 尚未收满
//   -1 = 帧头错误需重同步 (SYNC 态)
static int frame_feed(ModBootloader *me, uint8_t byte) {
  switch (me->frame_st) {
    case MOD_BL_FRAME_SYNC:
      if (byte == MOD_BL_FRAME_HEADER_BYTE) {
        me->frame_len = 0u;
        me->frame_st = MOD_BL_FRAME_HEADER;
      }
      return 0;

    case MOD_BL_FRAME_HEADER:
      me->frame_buf[me->frame_len++] = byte;
      if (me->frame_len >= 4u) {  // len_hi, len_lo, block_hi, block_lo
        uint16_t datalen = ((uint16_t) me->frame_buf[0] << 8) | me->frame_buf[1];
        if (datalen > MOD_BL_MAX_BLOCK_SIZE) {
          // 长度非法 → 回到 SYNC 重同步
          me->frame_st = MOD_BL_FRAME_SYNC;
          return -1;
        }
        me->frame_expect = datalen + 4u;  // 还需 data + crc
        me->frame_st = MOD_BL_FRAME_DATA;
      }
      return 0;

    case MOD_BL_FRAME_DATA:
      me->frame_buf[me->frame_len++] = byte;
      // frame_buf[0..3] = len/block, [4..] = data
      if (me->frame_len - 4u >= (me->frame_expect - 4u)) {
        me->frame_st = MOD_BL_FRAME_CRC;
      }
      return 0;

    case MOD_BL_FRAME_CRC:
      me->frame_buf[me->frame_len++] = byte;
      if (me->frame_len - 4u >= me->frame_expect) {
        // 完整帧: frame_len = 4(header) + datalen + 4(crc)
        me->frame_st = MOD_BL_FRAME_SYNC;
        return 1;
      }
      return 0;
  }
  return 0;
}

// 处理一个完整帧: 校验 CRC + 写 Flash + 更新状态
static int process_frame(ModBootloader *me) {
  // frame_buf: [0..3]=len/block, [4..4+datalen)=data, [末尾4]=crc
  uint16_t datalen = ((uint16_t) me->frame_buf[0] << 8) | me->frame_buf[1];
  uint16_t block = ((uint16_t) me->frame_buf[2] << 8) | me->frame_buf[3];
  uint8_t *data = &me->frame_buf[4];
  uint8_t *crc_field = &me->frame_buf[4u + datalen];

  // 校验 CRC (覆盖 len+block+data)
  // 重拼 len_hi..data (frame_buf[0..3] + data)
  uint32_t crc_calc = crc32_buf(me->frame_buf, 4u + datalen);
  uint32_t crc_recv = ((uint32_t) crc_field[0] << 24) | ((uint32_t) crc_field[1] << 16) |
                      ((uint32_t) crc_field[2] << 8) | crc_field[3];
  if (crc_calc != crc_recv) {
    return -1;  // 帧 CRC 错误
  }

  // 顺序检测 (块号须严格递增)
  if (block != me->next_block_id) {
    return -2;  // 乱序/丢块 — 升级失败 (简化: 要求严格顺序)
  }
  me->next_block_id++;

  // 首块: 解析固件总长度 + 触发 App 区整区擦除
  if (block == 0u) {
    if (datalen < 4u) {
      return -3;
    }
    me->fw_len = ((uint32_t) data[0] << 24) | ((uint32_t) data[1] << 16) |
                 ((uint32_t) data[2] << 8) | data[3];
    if (me->fw_len == 0u || me->fw_len > me->cfg.app_max_size) {
      return -4;
    }
    // 先整区擦除 App 区 (写之前必须擦)
    if (bsp_flash_erase(me->flash, 0u, me->fw_len) != 0) {
      return -5;
    }
    me->app_erased = true;
  }

  // 写块到 Flash (offset = block * MOD_BL_MAX_BLOCK_SIZE)
  if (write_block_data(me, block, data, datalen) != 0) {
    return -6;
  }
  // 连续推进已写固件长度 (首个 block 跳过 4 字节长度元数据)
  me->fw_received += (uint32_t) ((block == 0u) ? (datalen - 4u) : datalen);

  return 0;
}

// ======== 公开 API ========

int mod_bootloader_init(ModBootloader *me, const ModBootloaderTransport *transport,
                        const ModBootloaderCfg *cfg, BspFlash *flash,
                        uint8_t *db_buf, uint32_t db_buf_size,
                        uint8_t *rx_ring_buf) {
  if (!me || !cfg || !flash || !db_buf || !rx_ring_buf) {
    return ERR_ARG;
  }
  memset(me, 0, sizeof(*me));
  me->cfg = *cfg;
  me->flash = flash;
  if (transport) {
    me->transport = *transport;
  }
  // 收包环
  ring_init(&me->rx_ring, rx_ring_buf, cfg->rx_ring_size > 0u ? cfg->rx_ring_size : 128u);
  // 组帧状态机初始
  me->frame_st = MOD_BL_FRAME_SYNC;
  // Database (升级标志持久化)
  static const CompFlashOps flash_ops = {
      .ctx = flash, .erase = db_erase, .write = db_write, .read = db_read,
      .min_erase_size = db_min_erase, .size = db_size,
  };
  int ret = comp_database_init(&me->db, &flash_ops, db_buf, db_buf_size);
  if (ret != ERR_OK && ret != ERR_NOT_FOUND) {
    return ret;
  }
  // 读取升级标志 (键不存在 = 未请求升级)
  uint32_t flag = 0u;
  if (me->cfg.db_upgrade_key) {
    if (comp_database_get(&me->db, me->cfg.db_upgrade_key, &flag, sizeof(flag)) == ERR_OK) {
      me->upgrade_requested = (flag != 0u);
    }
  }
  me->st = MOD_BL_IDLE;
  return ERR_OK;
}

int mod_bootloader_rx(ModBootloader *me, const uint8_t *data, uint16_t len) {
  if (!me || !data || len == 0u) {
    return ERR_ARG;
  }
  // CTX_HMI: 非阻塞入环. 环满丢弃 (升级重发兜底).
  (void) ring_write(&me->rx_ring, data, len);
  return ERR_OK;
}

void mod_bootloader_poll(ModBootloader *me) {
  if (!me) return;

  // IDLE: 若请求升级则进入 RECV
  if (me->st == MOD_BL_IDLE && me->upgrade_requested) {
    me->st = MOD_BL_RECV;
  }

  // RECV: 从 rx_ring 逐字节喂组帧状态机, 收满一帧则处理
  if (me->st == MOD_BL_RECV) {
    while (me->st == MOD_BL_RECV) {
      // 读一字节
      uint8_t byte;
      if (!ring_pop(&me->rx_ring, &byte)) {
        break;  // 环空
      }
      int r = frame_feed(me, byte);
      if (r == 1) {
        // 收满一帧, 处理
        if (process_frame(me) != 0) {
          me->st = MOD_BL_ERROR;
          break;
        }
        // 收完判断: fw_received 是已写入 Flash 的固件数据长度 (首块已跳过 4 字节长度
        // 元数据), fw_len 是固件总长度 (首块 data[0..3] 大端). 收满 fw_len 即进入校验.
        if (me->fw_received >= me->fw_len) {
          me->st = MOD_BL_VERIFY;
          break;
        }
      } else if (r == -1) {
        // 帧头重同步 (长度非法) — 不改变整体状态, 继续等
      }
    }
  }

  // VERIFY: 校验 App 区 (读回前 fw_len 字节算 CRC, 与 cfg.app_crc 比)
  else if (me->st == MOD_BL_VERIFY) {
    if (me->cfg.app_crc != 0u) {
      // 读回 App 区前 fw_len 字节算 CRC
      // 用分块读避免大栈
      uint8_t tmp[64];
      uint32_t crc = 0xFFFFFFFFu;
      uint32_t off = 0u;
      while (off < me->fw_len) {
        uint32_t chunk = me->fw_len - off;
        if (chunk > sizeof(tmp)) chunk = sizeof(tmp);
        if (bsp_flash_read(me->flash, off, tmp, chunk) != 0) {
          me->st = MOD_BL_ERROR;
          return;
        }
        crc = crc32_calc(tmp, chunk, crc);
        off += chunk;
      }
      crc ^= 0xFFFFFFFFu;
      if (crc != me->cfg.app_crc) {
        me->st = MOD_BL_ERROR;
        return;
      }
    }
    // 校验通过 → 清除升级标志 → 跳转
    if (me->cfg.db_upgrade_key) {
      uint32_t zero = 0u;
      (void) comp_database_set(&me->db, me->cfg.db_upgrade_key, &zero, sizeof(zero));
    }
    me->st = MOD_BL_JUMP;
  }

  // JUMP: 跳转 App
  else if (me->st == MOD_BL_JUMP) {
    mod_bootloader_jump(me);
  }
}

bool mod_bootloader_should_jump(ModBootloader *me) {
  if (!me) return false;
  if (me->upgrade_requested) {
    return true;  // 应进入升级 (poll 接管), 不跳 App
  }
  return bsp_jump_validate_app(me->cfg.app_addr) == 0;
}

void mod_bootloader_jump(ModBootloader *me) {
  if (!me) return;
  if (bsp_jump_validate_app(me->cfg.app_addr) == 0) {
    bsp_jump_to_app(me->cfg.app_addr);
  }
  me->st = MOD_BL_ERROR;  // 跳转不返回
}

int mod_bootloader_request_upgrade(ModBootloader *me) {
  if (!me || !me->cfg.db_upgrade_key) return ERR_ARG;
  uint32_t one = 1u;
  int ret = comp_database_set(&me->db, me->cfg.db_upgrade_key, &one, sizeof(one));
  if (ret == ERR_NOT_FOUND) {
    ret = comp_database_add(&me->db, me->cfg.db_upgrade_key, &one, sizeof(one));
  }
  if (ret == ERR_OK) {
    me->upgrade_requested = true;
    me->st = MOD_BL_RECV;
  }
  return ret;
}

ModBootloaderState mod_bootloader_state(const ModBootloader *me) {
  return me ? me->st : MOD_BL_ERROR;
}
