// 位置编码器驱动 — motor 域转子位置反馈 (Devices 层)
//
// 来源: TI controlSUITE position_manager
// 翻译为 HardC 纯C 版本
//
// 定位: 转子位置反馈 — 功率控制采样的一种非 ADC 采样, 服务 CTX_FAST 闭环.
//   ctx: fast  — ISR 读位置在 CTX_FAST 热路径 (encoder_request / encoder_read_position)
//   ctx: slow/main — 速度估计 / 诊断 (encoder_update)
//
// 协议 → 总线映射:
//   BiSS-C   — RS485 双向 (Uart), MA脉冲 + SLO数据, CRC6 校验
//   Endat22  — RS485 双向 (Uart), 命令帧 + 位置帧, MRS 码
//   SinCos   — 模拟 1Vpp 差分 (ADC), 正/余弦插值
//   T-Format — 串行单向 (Uart), 纯接收, 无需发送
//   PTO      — 脉冲序列 ABZ + UVW (Gpio), 正交计数
//
// 数据面: 原始帧经 rx_buf 注入 (总线 IRQ/App 填帧), 本类只解析. 协议解析
//   算法完整保留 (BiSS-C/Endat22/SinCos/T-Format/PTO + CRC6/5/8 + 速度估计).
//
// 典型用法:
//   ISR (CTX_FAST): encoder_request() → encoder_read_position() → 获取位置
//   主循环 (CTX_MAIN): encoder_update(dt) → 速度估计 + 诊断

#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include "com_uart.h"
#include "com_gpio.h"
#include "bsp_adc.h"
#include "comp_error_code.h"
#include <stdint.h>
#include <stdbool.h>

// 编码器协议类型
typedef enum {
  EncProto_BiSS_C,   // BiSS-C (RS485 双向)
  EncProto_Endat22,  // Endat 2.2 (RS485 双向)
  EncProto_SinCos,   // Sin/Cos 模拟 (1Vpp 差分)
  EncProto_TFormat,  // T-Format (串行单向)
  EncProto_PTO,      // 脉冲序列 (ABZ + UVW)
} EncoderProto;

// 编码器配置 POD
typedef struct {
  EncoderProto proto;  // 协议类型
  int bits_single;     // 单圈分辨率 (bits, 典型 17~25)
  int bits_multi;      // 多圈分辨率 (bits, 典型 0~16, 0=单圈编码器)
  float freq_hz;       // 通信时钟频率 (Hz, 典型 1~10 MHz for RS485)
  float timeout_us;    // 通信超时 (us)
  bool use_crc;        // 是否启用 CRC 校验
} EncoderCfg;

// 编码器运行时 Instance — 独立结构体 (去 CommBase, 总线指针组合)
typedef struct {
  EncoderCfg cfg;     // 配置
  Uart *uart;         // BiSS/Endat/TFormat 串行总线 (NULL=未绑定, 走 rx_buf 注入)
  Gpio *gpio;         // PTO ABZ/UVW 电平总线 (NULL=未绑定)
  BspAdcHandle *adc;  // SinCos 模拟采样 (NULL=未绑定)

  // 位置数据
  uint64_t raw_position;  // 原始位置 (单圈 + 多圈)
  uint32_t single_turn;   // 单圈位置 (0 ~ 2^bits_single-1)
  int32_t multi_turn;     // 多圈计数
  float angle_rad;        // 角度 (rad, 0~2π)
  float velocity;         // 速度 (rad/s) — 差分估计

  // 状态
  bool connected;          // 编码器在线
  bool crc_error;          // 最后一次 CRC 校验失败
  bool timeout_error;      // 通信超时
  uint32_t error_count;    // 累计错误次数
  uint64_t last_position;  // 上一次位置 (速度估计用)
  float last_read_time;    // 上一次读取时间 (s)

  // 协议特有数据 (union, 按需访问)
  union {
    struct {
      bool warning;  // /WARN 信号
      bool error;    // /ERR 信号
      uint8_t crc6;  // CRC6 校验值
      uint8_t cds;   // CDS 位 (0=位置, 1=温度/辅助数据)
    } biss;
    struct {
      uint8_t mrs_code;  // MRS 码 (编码器类型/状态)
      bool param_ready;  // 参数读回完成
      uint8_t crc5;      // CRC5 校验值
    } endat;
    struct {
      float offset_x;    // 正弦通道偏移校准
      float offset_y;    // 余弦通道偏移校准
      float amplitude;   // 幅值比 (Y/X)
      float phase_corr;  // 正交相位修正 (rad, 典型 ±0.05)
    } sincos;
    struct {
      int32_t abz_count;   // ABZ 正交计数
      uint8_t uvw_state;   // UVW 初始扇区 (0~5)
      bool index_found;    // Z 相索引已检测
      uint8_t quad_state;  // AB 正交状态机 (0~3)
    } pto;
  } proto_data;

  // 接收缓冲 (协议解析用)
  uint8_t rx_buf[64];  // 原始帧数据 (最大 64 字节, 涵盖最长协议帧)
  uint16_t rx_len;     // 接收字节数
} Encoder;

// ======== 公共 API ========

// 初始化 — 绑协议配置 + 总线指针 (NULL=未绑定该总线, 帧经 rx_buf 注入)
void encoder_init(Encoder *me, const EncoderCfg *cfg, Uart *uart, Gpio *gpio, BspAdcHandle *adc);

// 反初始化
void encoder_deinit(Encoder *me);

// ISR 热路径: 发起一次位置读取 — Endat22 发读位置命令帧 (经 uart), 其余协议无需主机请求
// 返回: ERR_OK=已发起 / ERR_STATE=串行总线未绑定 / ERR_NOT_SUPPORT=协议需主机能力或未知
ErrorCode encoder_request(Encoder *me);

// ISR 热路径: 解析 rx_buf → 位置
// 返回: ERR_OK=成功 / ERR_TIMEOUT=帧短未对齐 (原-1) / ERR_CHECK=CRC失败 (原-2) /
//       ERR_STATE=未知协议 (原-3)
ErrorCode encoder_read_position(Encoder *me);

// 主循环: 速度估计 + 诊断 (dt: 距上次调用秒数)
void encoder_update(Encoder *me, float dt);

// 获取角度 (rad, 0~2π)
float encoder_get_angle(const Encoder *me);

// 获取速度 (rad/s)
float encoder_get_velocity(const Encoder *me);

// CRC 错误计数
uint32_t encoder_get_error_count(const Encoder *me);

// 编码器是否正常 (已连接 + 无 CRC 错误)
bool encoder_is_ok(const Encoder *me);

// 重置错误状态 (error_count 不归零, 仅清除 crc_error + timeout_error)
void encoder_reset(Encoder *me);

#endif  // MOTOR_ENCODER_H
