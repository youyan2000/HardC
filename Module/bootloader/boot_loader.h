// 引导加载模块 — Bootloader (Module 层)
//
// 职责: 固件升级 + 跳转 App.
//   - 升级传输: 支持 UART (com_uart DMA 收包) + CAN (com_can 帧订阅) — 复用 Devices/comm
//   - 校验:     comp_crc (CRC-32)
//   - 持久化:   comp_database (主备双块, 存升级标志/版本) — 修正版 comp_database_fixed.c
//   - Flash:    bsp_flash (BSP 不透明句柄)
//   - 跳转:     bsp_jump (BSP 平台差异跳转)
//
// 三上下文分工 (严格遵守 HardC):
//   CTX_HMI  : boot_loader_rx() — 收包 (UART RX / CAN 订阅回调 → 入 rx_ring), 非阻塞
//   CTX_MAIN : boot_loader_poll() — BackgroundTask 调, 解析协议 + CRC 校验 + Flash 写 (IO_SYNC 允许)
//   board_init: boot_loader_should_jump() — 若升级标志有效且 App 有效 → 清标志 → bsp_jump_to_app()
//
// 升级协议帧 (UART 字节流 / CAN 帧 统一):
//   [0xAA 帧头][len_hi][len_lo][block_id_hi][block_id_lo][data...][crc32(4B)]
//   len    = data 长度 (0..BOOT_BL_MAX_BLOCK_SIZE)
//   block  = 块号 (0..65535); 首块 (block==0) 的 data 前 4 字节 = 固件总长度 (大端)
//   crc32  = CRC32(len_hi + len_lo + block_hi + block_lo + data), 覆盖不含帧头
// 接收策略: 边收边写 (不缓存整个固件, 零 malloc)
//   - 每收到一个完整块: 校验块 CRC → 写 Flash (offset = block_id * BOOT_BL_MAX_BLOCK_SIZE)
//   - 首块触发整区擦除 (App 区)
//   - 全部块收完 (fw_received >= fw_len) → BOOT_BL_VERIFY → 校验 App 区 CRC → 跳转
//
// 组帧状态机 (UART 字节流必需, 处理跨帧重组):
//   BOOT_BL_FRAME_SYNC   — 找帧头 0xAA
//   BOOT_BL_FRAME_HEADER — 收 len(2) + block(2)
//   BOOT_BL_FRAME_DATA   — 收 data
//   BOOT_BL_FRAME_CRC    — 收 crc32(4)
//   CAN 每帧天然是完整块, 直接按一帧喂入 (rx_push 整帧入环, 组帧状态机对 CAN 退化为直通)

#ifndef BOOT_LOADER_H
#define BOOT_LOADER_H

#include <stdint.h>
#include <stdbool.h>
#include "comp_comm.h"
#include "comp_ring.h"
#include "comp_crc.h"
#include "comp_database.h"
#include "bsp_flash.h"
#include "bsp_jump.h"

// 升级协议帧头字节
//   注: _BYTE 后缀规避与组帧状态枚举 BOOT_BL_FRAME_HEADER 同名冲突 (同名#define 使枚举失效)
#define BOOT_BL_FRAME_HEADER_BYTE 0xAAu

// 升级传输接口 (支持 UART + CAN)
typedef struct {
  // 收包回调 (CTX_HMI): 收到完整帧/数据 → 入 rx_ring. 非阻塞.
  //   ctx, data, len
  // 返回 0=成功, 非0=错误
  int (*rx_push)(void *ctx, const uint8_t *data, uint16_t len);
  // 应答回调 (CTX_MAIN): 升级进度/结果 → UART/CAN 发. 非阻塞.
  //   ctx, data, len
  // 返回 0=成功, 非0=错误
  int (*tx_report)(void *ctx, const uint8_t *data, uint16_t len);
  void *ctx;  // 传输实例 (Uart* / Can*)
} BootLoaderTransport;

// 配置 POD (YAML 注入)
typedef struct {
  uint32_t app_addr;      // App 起始地址
  uint32_t app_max_size;  // App 最大大小 (字节)
  uint32_t app_crc;       // 目标固件 CRC-32 (升级前由上位机提供, 0=不校验)
  CrcPoly crc_type;       // CRC 类型 (默认 CrcPoly_32)
  uint16_t rx_ring_size;  // 收包环大小
  // Database 升级标志键
  const char *db_upgrade_key;  // 升级标志键名 (如 "upgrade")
} BootLoaderCfg;

// 状态机 (整体)
typedef enum {
  BOOT_BL_IDLE,    // 空闲
  BOOT_BL_RECV,    // 接收固件
  BOOT_BL_VERIFY,  // 校验 CRC
  BOOT_BL_JUMP,    // 跳转 App
  BOOT_BL_ERROR,   // 错误
} BootLoaderState;

// 组帧子状态 (UART 字节流)
typedef enum {
  BOOT_BL_FRAME_SYNC,    // 找帧头 0xAA
  BOOT_BL_FRAME_HEADER,  // 收 len(2) + block(2)
  BOOT_BL_FRAME_DATA,    // 收 data
  BOOT_BL_FRAME_CRC,     // 收 crc32(4)
} BootLoaderFrameState;

// 最大单块数据长度 (含首块前 4 字节固件长度字段)
#define BOOT_BL_MAX_BLOCK_SIZE 256u

// 运行时实例
typedef struct {
  BootLoaderTransport transport;  // 传输 (UART/CAN)
  BootLoaderCfg cfg;
  BspFlash *flash;  // Flash 管理器
  CompDatabase db;  // 升级标志持久化
  uint8_t *db_buf;  // Database 缓冲 (调用者提供)
  uint32_t db_buf_size;
  Ring rx_ring;          // 收包环 (HMI→MAIN)
  uint8_t *rx_ring_buf;  // 收包环缓冲
  BootLoaderState st;    // 整体状态

  // 组帧状态机 (UART 字节流)
  BootLoaderFrameState frame_st;
  uint8_t frame_buf[BOOT_BL_MAX_BLOCK_SIZE + 4u + 4u];  // data + crc32 (最大帧体)
  uint16_t frame_len;                                   // 当前帧已收字节数
  uint16_t frame_expect;                                // 当前帧期望总字节数 (header+data+crc)

  // 运行态 (升级)
  uint32_t fw_len;          // 目标固件长度 (首块前 4 字节)
  uint32_t fw_received;     // 已接收固件数据长度
  uint32_t app_crc_target;  // 目标 App CRC (cfg.app_crc)
  uint8_t next_block_id;    // 期望下一个块号 (顺序接收)
  bool app_erased;          // App 区是否已擦除
  bool upgrade_requested;   // 是否进入升级模式 (Database 标志)
} BootLoader;

// ======== API ========

// 初始化: 绑定传输 + Flash + Database + 收包环
//   transport: UART/CAN 收/答回调
//   cfg: 配置
//   flash: BSP Flash 句柄 (bsp_flash_bind + set_region 后)
//   db_buf/db_buf_size: Database 数据缓冲 (零 malloc)
//   rx_ring_buf/rx_ring_size: 收包环缓冲
// 返回 ERR_OK / ERR_ARG
int boot_loader_init(BootLoader *me, const BootLoaderTransport *transport, const BootLoaderCfg *cfg, BspFlash *flash,
                     uint8_t *db_buf, uint32_t db_buf_size, uint8_t *rx_ring_buf);

// CTX_HMI: 收包 (UART rx / CAN 订阅回调转发). 非阻塞.
//   data/len: 收到的字节/帧数据
// 返回 0=成功, 非0=错误
int boot_loader_rx(BootLoader *me, const uint8_t *data, uint16_t len);

// CTX_MAIN (BackgroundTask 调): 升级协议处理 + Flash 写 + 跳转.
//   IO_SYNC 允许 (MAIN 上下文), 会阻塞 Flash 擦写.
void boot_loader_poll(BootLoader *me);

// board_init: 判断是否进入升级 (读 Database 升级标志)
//   返回 true=应进入升级, false=应跳转 App
bool boot_loader_should_jump(BootLoader *me);

// 跳转 App (校验有效后)
void boot_loader_jump(BootLoader *me);

// 请求进入升级模式 (App 收到升级命令时调, 写 Database 标志)
int boot_loader_request_upgrade(BootLoader *me);

// 当前状态
BootLoaderState boot_loader_state(const BootLoader *me);

#endif  // BOOT_LOADER_H
