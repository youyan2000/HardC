// Module 层帧协议解析器 —— 基于 UART 的变长帧协议
// 参考 SmCar Rx_FSM 三帧协议模式 (0xFE/0xFD/0xEF 头)
// 帧格式: [0xAA][CMD][LEN][DATA 0~255B][CHK] — XOR 逐字节累加校验
//
// 状态机: R_IDLE → R_CMD → R_LEN → R_DAT → R_CHK → (回调) → R_IDLE
// 用法: 在 HAL UART RX 回调中调用 mod_comm_feed(), 收到完整帧时自动回调

#include "mod_comm.h"
#include <stddef.h>

// -------- 帧协议常量 --------
#define FRAME_HEAD  0xAA  // 帧头魔数

// -------- 初始化 --------

// 绑定 UART → 状态机归零 → 回调表清零
void mod_comm_init(ModComm *me, CommBase *uart) {
  me->uart = uart;
  me->st   = R_IDLE;
  me->cmd  = 0;
  me->len  = 0;
  me->idx  = 0;
  me->chk  = 0;
  for (int i = 0; i < 256; i++) { me->data[i] = 0; }
  for (int i = 0; i < 256; i++) { me->handlers[i] = NULL; }
}

// -------- 逐字节喂入状态机 --------

// 在 HAL UART RX 回调中调用, 每收到 1 字节喂入一次
// 当一帧完整且校验通过时, 自动查找并调用已注册的命令回调
void mod_comm_feed(ModComm *me, uint8_t byte) {

  switch (me->st) {

  // --- R_IDLE: 等待帧头 0xAA ---
  case R_IDLE:
    if (byte == FRAME_HEAD) {
      me->st  = R_CMD;
      me->chk = byte;  // 校验从帧头开始累加
    }
    // 非帧头字节: 静默丢弃
    break;

  // --- R_CMD: 读取命令字 ---
  case R_CMD:
    me->cmd  = byte;
    me->chk ^= byte;
    me->st   = R_LEN;
    break;

  // --- R_LEN: 读取数据长度 ---
  case R_LEN:
    me->len  = byte;
    me->chk ^= byte;
    if (me->len == 0) {
      me->st = R_CHK;  // 无数据帧 → 直接等校验
    } else {
      me->st  = R_DAT;
      me->idx = 0;
    }
    break;

  // --- R_DAT: 逐字节接收数据 ---
  case R_DAT:
    me->data[me->idx] = byte;
    me->chk ^= byte;
    me->idx++;
    if (me->idx >= me->len) {
      me->st = R_CHK;  // 数据收齐 → 校验
    }
    break;

  // --- R_CHK: 校验帧完整性 ---
  case R_CHK:
    if (byte == me->chk) {
      // 校验通过: 查找并调用命令回调
      mod_cmd_fn fn = me->handlers[me->cmd];
      if (fn) {
        fn(me->cmd, me->data, me->len);
      }
    }
    // 校验失败或回调完成: 回到空闲, 等下一帧
    me->st = R_IDLE;
    break;
  }
}

// -------- 组帧发送 --------

// 将 cmd + data 封装为帧, 通过绑定 UART 发送
void mod_comm_send(ModComm *me, uint8_t cmd, const uint8_t *dat, uint8_t len) {
  uint8_t buf[258];  // 最大帧: 1头+1cmd+1len+255data+1chk = 259, 栈上 256 安全
  uint8_t chk = FRAME_HEAD;

  buf[0] = FRAME_HEAD;

  buf[1] = cmd;
  chk   ^= cmd;

  buf[2] = len;
  chk   ^= len;

  for (uint8_t i = 0; i < len; i++) {
    buf[3 + i] = dat[i];
    chk       ^= dat[i];
  }

  buf[3 + len] = chk;

  comm_send(me->uart, buf, (uint16_t)(4 + len));
}

// -------- 命令回调注册 --------

// 注册 cmd → fn 映射, 收到该 cmd 时自动回调
void mod_comm_on(ModComm *me, uint8_t cmd, mod_cmd_fn fn) {
  me->handlers[cmd] = fn;
}
