// 调试串口协议框架 — COM-OOP Module 层
// 串口协议设计 (hmi_ui.c car_cmd_rx / car_cmd_ef_rx / cfg_rx)
//
// 5 种帧类型, 接收状态机按帧头路由:
//   0xFA: 控制命令 (2 字节) → CarCmd → CmdDispatcher
//   0xFB: PID 批量调参 (33 字节) → pi_check 校验 → on_pid_tune 回调
//   0xFC: 传感器查询 (变长) → CRC-8 校验 → on_sensor_cmd 回调
//   0xEE: 校准流程 (2 字节) → on_cal_step 回调
//   0xEF: 底层直控 (2 字节) → CarCmd → CmdDispatcher
//
// ISR 安全: serial_proto_feed 仅写 buf + idx + frame_ready,
//           不调用任何用户回调, 不 printf
// 主循环:   serial_proto_process 校验 + 回调 + 应答, 可 printf

#include "hmi_serial_proto.h"
#include <stddef.h>
#include <string.h>

// ======== CRC 校验 ========

// CRC-16/XMODEM (poly=0x1021, init=0x0000)
// 用于 0xFB PID 帧完整性校验
static uint16_t crc16_xmodem(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0x0000;
  while (len--) {
    crc ^= (uint16_t) (*data++) << 8;
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// CRC-8 (poly=0x07, init=0x00)
// 用于 0xFC 传感器帧尾校验
static uint8_t crc8(const uint8_t *data, uint16_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// ======== 帧长度常量 ========

// 各帧类型期望长度
#define LEN_CTRL 2    // [0xFA][CarCmd byte]
#define LEN_PID 33    // sizeof(SerialPidFrame)
#define LEN_CAL 2     // [0xEE][CalStep]
#define LEN_DIRECT 2  // [0xEF][cmd byte]
// 0xFC 传感器帧为变长, exp_len 在接收到长度字节后动态更新

// ======== 初始化 ========

void serial_proto_init(SerialProto *me) {
  me->state = RX_IDLE;
  me->header = 0;
  me->idx = 0;
  me->exp_len = 0;
  me->frame_ready = false;
  me->frame_count = 0;
  me->crc_err_count = 0;
  me->on_pid_tune = NULL;
  me->on_sensor_cmd = NULL;
  me->on_cal_step = NULL;
  me->send_fn = NULL;
  memset(me->buf, 0, sizeof(me->buf));
}

// ======== 逐字节喂入状态机 (ISR 安全) ========

// ISR 中逐字节调用。非帧头字节返回 false (可继续传递给其他协议解析器)。
// 状态机路径:
//   RX_IDLE → 匹配帧头 → 切换子状态 + 设 exp_len
//   子状态  → 累积字节 → idx >= exp_len → frame_ready = true → 回到 RX_IDLE
bool serial_proto_feed(SerialProto *me, uint8_t byte) {
  // --- RX_IDLE: 等待帧头 ---
  if (me->state == RX_IDLE) {
    switch (byte) {
    case FRAME_CTRL:
      me->state = RX_CTRL;
      me->header = FRAME_CTRL;
      me->buf[0] = byte;
      me->idx = 1;
      me->exp_len = LEN_CTRL;
      return true;

    case FRAME_PID_TUNE:
      me->state = RX_PID;
      me->header = FRAME_PID_TUNE;
      me->buf[0] = byte;
      me->idx = 1;
      me->exp_len = LEN_PID;
      return true;

    case FRAME_SENSOR:
      me->state = RX_SENSOR;
      me->header = FRAME_SENSOR;
      me->buf[0] = byte;
      me->idx = 1;
      // 变长帧: 先收齐 [header][cmd][len] = 3 字节,
      // 再根据 data_len 更新 exp_len
      me->exp_len = 3;  // 临时值, 收到长度字节后更新为 4 + data_len
      return true;

    case FRAME_CAL:
      me->state = RX_CAL;
      me->header = FRAME_CAL;
      me->buf[0] = byte;
      me->idx = 1;
      me->exp_len = LEN_CAL;
      return true;

    case FRAME_DIRECT:
      me->state = RX_DIRECT;
      me->header = FRAME_DIRECT;
      me->buf[0] = byte;
      me->idx = 1;
      me->exp_len = LEN_DIRECT;
      return true;

    default:
      // 非帧头字节, 未消费
      return false;
    }
  }

  // --- 子状态: 累积字节 ---

  // 缓冲溢出保护
  if (me->idx >= sizeof(me->buf)) {
    // 帧过长, 丢弃并回到空闲
    me->state = RX_IDLE;
    me->idx = 0;
    return false;
  }

  me->buf[me->idx++] = byte;

  // 变长帧特殊处理: 收到 data_len 字节后更新 exp_len
  if (me->state == RX_SENSOR && me->idx == 3) {
    // buf[0]=0xFC, buf[1]=SensorCmd, buf[2]=data_len
    uint8_t data_len = me->buf[2];
    // 帧总长 = header(1) + cmd(1) + len(1) + data(data_len) + crc8(1)
    me->exp_len = 4 + data_len;
  }

  // 帧收齐?
  if (me->idx >= me->exp_len) {
    me->frame_ready = true;  // 通知主循环
    me->state = RX_IDLE;     // 回到空闲, 准备下一帧
  }

  return true;
}

// ======== 帧就绪检查 ========

bool serial_proto_is_frame_ready(const SerialProto *me) {
  return me->frame_ready;
}

// ======== 帧处理 + 应答 (主循环调用) ========

void serial_proto_process(SerialProto *me, CmdDispatcher *cmd) {
  if (!me->frame_ready)
    return;

  // 先复制帧信息, 再清标志 (防止 ISR 覆盖 buf 期间数据竞争)
  uint8_t hdr = me->header;
  uint16_t len = me->idx;  // 实际接收字节数
  uint8_t frame_buf[64];
  memcpy(frame_buf, me->buf, len);
  me->frame_ready = false;

  switch (hdr) {
  // ======== 0xFA: 控制命令 → CarCmd → CmdDispatcher ========
  case FRAME_CTRL: {
    CarCmd car_cmd = cmd_from_serial_byte(frame_buf[1]);
    if (car_cmd != CMD_NONE && cmd) {
      cmd_dispatch_execute(cmd, car_cmd, NULL, 0);
    }
    me->frame_count++;
    break;
  }

  // ======== 0xFB: PID 批量调参 ========
  case FRAME_PID_TUNE: {
    if (len < sizeof(SerialPidFrame)) {
      me->crc_err_count++;
      break;
    }

    const SerialPidFrame *f = (const SerialPidFrame *) frame_buf;

    // 校验 1: pi_check 魔数 (π = 0x40490FDA)
    if (f->pi_check != PID_TUNE_PI_MAGIC) {
      me->crc_err_count++;
      break;
    }

    // 校验 2: 帧尾 0xFE
    if (f->tail != 0xFE) {
      me->crc_err_count++;
      break;
    }

    // 校验 3: CRC-16/XMODEM (覆盖 [0..29], 即排除 crc16 和 tail)
    uint16_t calc_crc = crc16_xmodem(frame_buf, 30);
    if (calc_crc != f->crc16) {
      me->crc_err_count++;
      break;
    }

    // 全部校验通过 → 回调
    if (me->on_pid_tune) {
      me->on_pid_tune(f);
    }
    me->frame_count++;
    break;
  }

  // ======== 0xFC: 传感器查询 ========
  case FRAME_SENSOR: {
    // 最小帧长: [0xFC][SensorCmd][data_len=0][crc8] = 4 字节
    if (len < 4) {
      me->crc_err_count++;
      break;
    }

    // CRC-8 校验 (覆盖除 crc8 外的所有字节)
    uint8_t calc_crc = crc8(frame_buf, len - 1);
    if (calc_crc != frame_buf[len - 1]) {
      me->crc_err_count++;
      break;
    }

    SensorCmd sub = (SensorCmd) frame_buf[1];
    uint8_t dlen = frame_buf[2];
    const uint8_t *dptr = (dlen > 0) ? &frame_buf[3] : NULL;

    if (me->on_sensor_cmd) {
      me->on_sensor_cmd(sub, dptr, dlen);
    }
    me->frame_count++;
    break;
  }

  // ======== 0xEE: 校准流程控制 ========
  case FRAME_CAL: {
    CalStep step = (CalStep) frame_buf[1];
    if (me->on_cal_step) {
      me->on_cal_step(step);
    }
    me->frame_count++;
    break;
  }

  // ======== 0xEF: 底层直控 → CarCmd → CmdDispatcher ========
  case FRAME_DIRECT: {
    CarCmd car_cmd = cmd_from_direct_byte(frame_buf[1]);
    if (car_cmd != CMD_NONE && cmd) {
      cmd_dispatch_execute(cmd, car_cmd, NULL, 0);
    }
    me->frame_count++;
    break;
  }

  default:
    // 未知帧头 (不应到达)
    break;
  }
}

// ======== 发送应答帧 ========

// 通过注册的 send_fn 发送数据, 返回 true = 发送成功
bool serial_proto_send_response(SerialProto *me, const uint8_t *data, uint16_t len) {
  if (!me->send_fn || !data || len == 0) {
    return false;
  }
  me->send_fn(data, len);
  return true;
}
