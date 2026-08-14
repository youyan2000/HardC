// 超级电容 CAN 协议模块 — COM-OOP Module 层 (ctx main)
// 组帧/解析 0x051/0x061, 通过回调接缝收发, 零 HAL/Components 依赖 (host 单 TU 可测)
// 收发全在主循环 (ctx main): 5ms 周期 tx_telemetry + 主循环 poll 排空 RX
// 帧布局/功率编码见 mod_can_proto.h 头注释

#include "mod_can_proto.h"
#include <string.h>

// ======== 内部工具 ========

// 浮点 clamp: v 限幅到 [lo, hi]
static float clamp_f(float v, float lo, float hi) {
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

// 功率编码 + 边界饱和: 输入 clamp [-256,768] 后编码, 返回 u16
// 768.00W 精确边界编码 65536.0f, (uint16_t)65536.0f 属越界转换 (C11 6.3.1.4 UB) —
// 此处饱和到 0xFFFF, 防 clamp 目的被精确边界击穿
static uint16_t sc_encode_power(float p) {
  float enc = clamp_f(p, MOD_CAN_POWER_MIN_W, MOD_CAN_POWER_MAX_W) * 64.0f + 16384.0f;
  if (enc >= 65536.0f) {
    return 65535u;
  }
  return (uint16_t) enc;
}

// ======== 初始化 ========

void mod_can_init(ModCanProto *me) {
  memset(me, 0, sizeof(*me));
}

// ======== 绑定 I/O 接缝 ========

void mod_can_bind(ModCanProto *me, mod_can_send_fn send, mod_can_poll_fn poll, mod_can_on_referee_fn on_referee) {
  me->send = send;
  me->poll = poll;
  me->on_referee = on_referee;
}

// ======== 组帧发送 0x051 ========

void mod_can_tx_telemetry(ModCanProto *me, const ModCanTelemetry *t) {
  if (me == NULL || t == NULL) {
    return;
  }

  // buf[0]: 裁判功率上限 (W, 直接存 u8, clamp [0,255])
  me->tx_buf[0] = (uint8_t) clamp_f(t->referee_power_limit_w, 0.0f, 255.0f);

  // 三个功率字段: 饱和编码 (内部 clamp [-256,768] + 边界饱和, 防编码回绕)
  uint16_t chassis = sc_encode_power(t->chassis_power_w);
  uint16_t referee = sc_encode_power(t->referee_power_w);
  uint16_t scmx = sc_encode_power(t->supercap_output_mx_w);

  // 全小端打包 (STM32/C2000 均 LE): buf[i]=lo, buf[i+1]=hi
  me->tx_buf[1] = (uint8_t) (chassis & 0xFFu);
  me->tx_buf[2] = (uint8_t) (chassis >> 8);
  me->tx_buf[3] = (uint8_t) (referee & 0xFFu);
  me->tx_buf[4] = (uint8_t) (referee >> 8);
  me->tx_buf[5] = (uint8_t) (scmx & 0xFFu);
  me->tx_buf[6] = (uint8_t) (scmx >> 8);

  // buf[7]: 输出能力百分比, clamp [0,100]
  me->tx_buf[7] = (uint8_t) clamp_f(t->output_capability_pct, 0.0f, 100.0f);

  if (me->send) {
    me->send(MOD_CAN_TX_ID, me->tx_buf, 8);
    me->tx_count++;
  }
}

// ======== 接收解析入口 ========

void mod_can_on_frame(ModCanProto *me, uint32_t id, const uint8_t *data, uint8_t len) {
  if (me == NULL || id != MOD_CAN_RX_ID || data == NULL) {
    return;
  }

  // 0x061 最小帧长: [0] 使能/保留 + [1-2] 裁判功率上限 = 3 字节
  if (len < 3) {
    me->rx_drop_count++;
    return;
  }

  // [0] bit0: 变换器使能位
  me->referee.enable_conv = (data[0] & 0x01u) != 0;
  // [1-2] u16 LE 解码 → W
  uint16_t raw = (uint16_t) ((uint16_t) data[1] | ((uint16_t) data[2] << 8));
  me->referee.power_limit_w = POWER_U16_TO_WATT(raw);

  me->rx_count++;
  if (me->on_referee) {
    me->on_referee(&me->referee);
  }
}

// ======== 轮询收帧 ========

bool mod_can_poll(ModCanProto *me) {
  if (me == NULL || me->poll == NULL) {
    return false;
  }

  uint32_t id = 0;
  uint8_t dlc = 0;
  uint8_t data[8];
  bool any = false;

  // 排空 RX 队列: poll 返回 true 即有帧, 交给解析入口 (dlc 用实际值, 可能 < 8)
  while (me->poll(&id, &dlc, data)) {
    mod_can_on_frame(me, id, data, dlc);
    any = true;
  }
  return any;
}
