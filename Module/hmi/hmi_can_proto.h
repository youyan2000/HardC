// 超级电容 CAN 协议模块 — COM-OOP Module 层 (ctx main)
// 来源: WEILAI 未来战队 2026 三相超级电容 FDCAN 协议
//   (对照 docs/learning/SuperCap_Projects_Study_Report.md 八、通信协议对比)
//
// 帧布局 (标准帧, 8 字节, 全小端 LE):
//   0x051 发送遥测 (200Hz, 5ms 周期):
//     [0]      refereePowerLimit   uint8_t   裁判功率上限 (W, 直接存, clamp 0..255)
//     [1-2]    chassisPower        uint16_t  底盘功率 (编码 p*64+16384)
//     [3-4]    refereePower        uint16_t  裁判功率 (同上编码)
//     [5-6]    SuperCapOutputMx    uint16_t  超电最大输出功率 (同上编码)
//     [7]      OutPutCapability    uint8_t   输出能力百分比 (clamp 0..100)
//   0x061 接收 (裁判/主控下发):
//     [0] bit0 enableCONV          1 bit     变换器使能位
//     [0] bit1..7 resv             7 bit     保留
//     [1-2]    refereePowerLimit  uint16_t  裁判功率上限 (W, 解码 (u-16384)/64)
//     [3-7]    resv               5 byte    保留
//
// 功率编码 (与 WEILAI mod_conn.h 一致):
//   u16 编码 = p*64 + 16384, 量程 -256W ~ +768W, 分辨率 0.015625W
//   负功率 (放电) 也编码进 uint16_t; 编码前须 clamp 到量程内防回绕
//
// 用法:
//   1. HmiCanProto can;
//   2. hmi_can_init(&can);
//   3. App board_init:
//        hmi_can_bind(&can, my_can_send, my_can_poll, my_on_referee);
//      on_referee 回调 → gen_supercap_set_referee_power(&sc, ref->power_limit_w)
//   4. 5ms 周期 (1kHz ISR ÷ 5): hmi_can_tx_telemetry(&can, &tel);
//   5. 主循环: hmi_can_poll(&can);  // 排空 RX, 逐帧走 hmi_can_on_frame
//
// ctx: main — 协议收发全部在主循环/低速上下文, 不进 28kHz 控制 ISR

#ifndef HMI_CAN_PROTO_H
#define HMI_CAN_PROTO_H

#include <stdint.h>
#include <stdbool.h>

// ======== CAN 帧 ID ========
#define HMI_CAN_TX_ID 0x051u  // 发送遥测帧
#define HMI_CAN_RX_ID 0x061u  // 接收裁判帧

// ======== 功率编码 (WEILAI mod_conn.h 一致) ========
#define POWER_WATT_TO_U16(p) ((uint16_t) ((p) * 64.0f + 16384.0f))
#define POWER_U16_TO_WATT(u) (((float) (u) - 16384.0f) / 64.0f)
#define HMI_CAN_POWER_MIN_W (-256.0f)
#define HMI_CAN_POWER_MAX_W (768.0f)

// 0x051 发送遥测 (float W → 编码)
typedef struct {
  float referee_power_limit_w;  // → buf[0] u8 直接存 (clamp 0..255)
  float chassis_power_w;        // → u16 编码
  float referee_power_w;        // → u16 编码
  float supercap_output_mx_w;   // → u16 编码
  float output_capability_pct;  // → buf[7] u8 (clamp 0..100)
} HmiCanTelemetry;

// 0x061 解析结果
typedef struct {
  bool enable_conv;     // [0] bit0
  float power_limit_w;  // [1-2] u16 解码
} HmiCanReferee;

// 传输 I/O 接缝 (回调绑定, host 可测 / 可绑任意 CAN 设备)
typedef bool (*hmi_can_poll_fn)(uint32_t *id, uint8_t *dlc, uint8_t data[8]);    // 轮询收帧, 有帧填并返回 true
typedef void (*hmi_can_send_fn)(uint32_t id, const uint8_t *data, uint8_t dlc);  // 发送一帧
typedef void (*hmi_can_on_referee_fn)(const HmiCanReferee *ref);                 // 收到有效 0x061 后回调

// 协议上下文
typedef struct {
  hmi_can_send_fn send;              // TX 接缝
  hmi_can_poll_fn poll;              // RX 轮询接缝
  hmi_can_on_referee_fn on_referee;  // 0x061 回调 (App 绑定 → gen_supercap_set_referee_power)
  HmiCanReferee referee;             // 最近解析的 0x061 (公开, 只读)
  uint32_t rx_count;                 // 收到 0x061 有效帧计数
  uint32_t rx_drop_count;            // 长度不足丢弃计数
  uint32_t tx_count;                 // 发送帧计数
  uint8_t tx_buf[8];                 // 0x051 组帧缓冲
} HmiCanProto;

// ======== API ========

// 初始化协议上下文: memset 归零
void hmi_can_init(HmiCanProto *me);

// 绑定 I/O 接缝 (App board_init 调用)
void hmi_can_bind(HmiCanProto *me, hmi_can_send_fn send, hmi_can_poll_fn poll, hmi_can_on_referee_fn on_referee);

// 组 0x051 → me->send (未绑定则忽略)
void hmi_can_tx_telemetry(HmiCanProto *me, const HmiCanTelemetry *t);

// 轮询收帧: 调 me->poll 逐帧转发给 hmi_can_on_frame; 返回是否有任一帧
bool hmi_can_poll(HmiCanProto *me);

// 解析入口 (0x061 处理, 其它忽略)
void hmi_can_on_frame(HmiCanProto *me, uint32_t id, const uint8_t *data, uint8_t len);

#endif  // HMI_CAN_PROTO_H
