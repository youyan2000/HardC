// 位置编码器驱动 — CommBase 子类实现
//
// 来源: TI controlSUITE position_manager
// 翻译为 HardC 纯C 版本
//
// 架构:
//   CommBase (comp_comm.h)
//     ↑ container_of
//   Encoder (本文件) — 协议分发 + 位置解算 + 速度估计
//
// 协议分发: 所有 ops 根据 cfg.proto 做 switch, 具体协议实现在各自分支
//   当前版本提供完整的帧解析算法, 硬件 I/O 由 BSP/App 层通过 rx_buf 注入
//
// 速度估计: (position - last_position) / dt, 处理单圈回绕
//   回绕检测: |diff| > half_range → 修正 (如 17-bit: 0→131071 是正向,
//             131071→0 是反向, diff > 65536 判断回绕方向)

#include "com_encoder.h"
#include "container_of.h"
#include <math.h>
#include <string.h>
#include "comp_math.h"

// ======== CRC 计算 (协议通用) ========

// CRC6 — BiSS-C: 多项式 x^6 + x + 1 (0x43), 初始值 0x00
// 输入: bit 流按 MSB-first, 覆盖 Start→Warn 的所有位
// 输出: 6-bit CRC 放置在低 6 位
static uint8_t crc6_compute(const uint8_t *data, int nbits) {
  uint8_t crc = 0;
  for (int i = 0; i < nbits; i++) {
    int byte_idx = i / 8;
    int bit_idx  = 7 - (i % 8);  // MSB first
    uint8_t bit_in = (data[byte_idx] >> bit_idx) & 1;
    uint8_t bit_out = (crc >> 5) & 1;  // x^6 反馈
    crc = (uint8_t)((crc << 1) | bit_in);
    if (bit_out) {
      crc ^= 0x43;  // 多项式 0x43 = x^6 + x + 1
    }
  }
  return crc & 0x3F;  // 低 6 位
}

// CRC5 — Endat 2.2: 多项式 x^5 + x^2 + 1 (0x25), 初始值 0x00
static uint8_t crc5_compute(const uint8_t *data, int nbits) {
  uint8_t crc = 0;
  for (int i = 0; i < nbits; i++) {
    int byte_idx = i / 8;
    int bit_idx  = 7 - (i % 8);
    uint8_t bit_in = (data[byte_idx] >> bit_idx) & 1;
    uint8_t bit_out = (crc >> 4) & 1;  // x^5 反馈
    crc = (uint8_t)((crc << 1) | bit_in);
    if (bit_out) {
      crc ^= 0x25;  // 多项式 0x25 = x^5 + x^2 + 1
    }
  }
  return crc & 0x1F;  // 低 5 位
}

// CRC8 — T-Format: 多项式 x^8 + x^2 + x + 1 (0x07), 初始值 0x00
static uint8_t crc8_compute(const uint8_t *data, int nbytes) {
  uint8_t crc = 0;
  for (int i = 0; i < nbytes; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 0x80) {
        crc = (uint8_t)((crc << 1) ^ 0x07);
      } else {
        crc = (uint8_t)(crc << 1);
      }
    }
  }
  return crc;
}

// ======== 位提取辅助 ========

// 从字节数组提取 N-bit 字段 (MSB first)
// offset_bits: 起始位偏移 (从 0 开始, 相对于 data 起始)
// nbits: 要提取的位数 (1~64)
static uint64_t bits_extract(const uint8_t *data, int offset_bits, int nbits) {
  uint64_t val = 0;
  for (int i = 0; i < nbits; i++) {
    int total_bit = offset_bits + i;
    int byte_idx = total_bit / 8;
    int bit_idx  = 7 - (total_bit % 8);  // MSB first
    val = (val << 1) | ((data[byte_idx] >> bit_idx) & 1);
  }
  return val;
}

// ======== BiSS-C 帧解析 ========
//
// SLO 帧格式 (从编码器接收):
//   Start(1) + CDS(1) + Position(single+multi bits) + Error(1) + Warn(1) + CRC6(6)
//   总位数 = 1 + 1 + bits_single + bits_multi + 1 + 1 + 6
//
// Start bit 总是 1. 如果 Start bit 为 0, 帧未对齐.
// CDS: 0=位置数据, 1=温度/辅助传感器数据
// CRC6 覆盖 Start → Warn (不含 CRC 自身)
static int biss_parse_frame(Encoder *me) {
  int pos_bits = me->cfg.bits_single + me->cfg.bits_multi;
  int total_bits = 1 + 1 + pos_bits + 1 + 1 + 6;  // Start + CDS + Pos + Err + Warn + CRC6
  int total_bytes = (total_bits + 7) / 8;

  if (me->rx_len < (uint16_t)total_bytes) {
    return -1;  // 帧太短
  }

  // 检查 Start bit (bit 0)
  uint8_t start_bit = (uint8_t)bits_extract(me->rx_buf, 0, 1);
  if (start_bit != 1) {
    return -1;  // 帧未对齐 — Start bit 应为 1
  }

  // 检查 CRC6 (最后 6 bits)
  int crc_data_bits = total_bits - 6;  // Start→Warn 的位数
  uint8_t crc_received = (uint8_t)bits_extract(me->rx_buf, crc_data_bits, 6);
  uint8_t crc_computed = crc6_compute(me->rx_buf, crc_data_bits);

  if (crc_received != crc_computed && me->cfg.use_crc) {
    me->proto_data.biss.crc6 = crc_received;
    return -2;  // CRC 错误
  }
  me->proto_data.biss.crc6 = crc_received;

  // 提取 CDS (bit 1)
  me->proto_data.biss.cds = (uint8_t)bits_extract(me->rx_buf, 1, 1);

  // 提取 Position (bits 2 → 2+pos_bits-1)
  me->raw_position = bits_extract(me->rx_buf, 2, pos_bits);

  // 提取 Error (bit 2+pos_bits)
  int err_bit = 2 + pos_bits;
  me->proto_data.biss.error = (bits_extract(me->rx_buf, err_bit, 1) == 1);

  // 提取 Warn (bit 2+pos_bits+1)
  int warn_bit = err_bit + 1;
  me->proto_data.biss.warning = (bits_extract(me->rx_buf, warn_bit, 1) == 1);

  return 0;
}

// ======== Endat 2.2 帧解析 ========
//
// 命令帧 (主→编码器):
//   Start(1) + Cmd(6) + Param(取决于命令) + CRC5(5)
//   命令: 0x00=读位置, 0x07=读参数, 0x1C=选择存储区, 等等
//
// 位置帧 (编码器→主):
//   Start(1) + FType(1) + Position(single+multi) + CRC5(5)
//   FType: 0=位置 (带附加数据), 1=参数
//   CRC5 覆盖 Start→Position 所有位
//
// MRS 码 (编码器类型信息):
//   从"读参数"命令返回的 MRS (Manufacturer Specific Register) 中提取
//   典型: 0x37=ECN 系列, 0x3F=EQN 系列

// Endat 模式命令定义
#define ENDAT_CMD_READ_POS   0x00   // 读位置
#define ENDAT_CMD_SEL_MRS    0x1C   // 选择 MRS 存储区
#define ENDAT_CMD_READ_PARAM 0x07   // 读参数

static int endat_parse_position(Encoder *me) {
  int pos_bits = me->cfg.bits_single + me->cfg.bits_multi;
  // 位置帧: Start(1) + FType(1) + Position(N) + CRC5(5)
  int total_bits = 1 + 1 + pos_bits + 5;
  int total_bytes = (total_bits + 7) / 8;

  if (me->rx_len < (uint16_t)total_bytes) {
    return -1;
  }

  // Start bit 检查
  if (bits_extract(me->rx_buf, 0, 1) != 1) {
    return -1;
  }

  // FType (bit 1) — 0=位置数据
  uint8_t ftype = (uint8_t)bits_extract(me->rx_buf, 1, 1);

  // CRC5 验证
  int crc_data_bits = 1 + 1 + pos_bits;  // Start + FType + Position
  uint8_t crc_rx = (uint8_t)bits_extract(me->rx_buf, crc_data_bits, 5);
  uint8_t crc_calc = crc5_compute(me->rx_buf, crc_data_bits);

  if (crc_rx != crc_calc && me->cfg.use_crc) {
    me->proto_data.endat.crc5 = crc_rx;
    return -2;
  }
  me->proto_data.endat.crc5 = crc_rx;

  if (ftype == 0) {
    // 位置数据: 提取 Position (bits 2 → 2+pos_bits-1)
    me->raw_position = bits_extract(me->rx_buf, 2, pos_bits);
  }
  // ftype==1: 参数返回 (如 MRS 码), 不作位置更新
  //   MRS 在 bit 2~15 (14 bits)

  return 0;
}

// 构建 Endat 命令帧到 tx 缓冲 (供 encoder_send 使用)
static int endat_build_cmd(uint8_t *tx_buf, uint8_t cmd, uint16_t param) {
  // 命令帧: Start(0→1 边沿) + Cmd(6) + Param(16) + CRC5(5) = 28 bits = 4 bytes
  // 注: Start bit 由硬件 RS485 方向切换产生, 帧数据从 bit 1 开始
  // 构建: [Cmd:6][Param_hi:8][Param_lo:8][CRC5:5 + pad:3]
  uint32_t frame = 0;
  frame |= ((uint32_t)cmd & 0x3F) << 22;    // Cmd: bits 22-27
  frame |= ((uint32_t)param & 0xFFFF) << 6;  // Param: bits 6-21
  // CRC5 覆盖 bit 1-22 (Cmd + Param), 不含 Start
  // 临时计算: 将 frame bit 1-22 提取为字节序列
  uint8_t crc_data[3];
  uint32_t data_part = (frame >> 6) & 0x3FFFFF;  // 22 bits: Cmd(6)+Param(16)
  crc_data[0] = (uint8_t)(data_part >> 14);
  crc_data[1] = (uint8_t)(data_part >> 6);
  crc_data[2] = (uint8_t)(data_part << 2);
  uint8_t crc5 = crc5_compute(crc_data, 22);
  frame |= crc5 << 1;  // CRC5: bits 1-5
  // bit 0 = 保留 (可为 0)

  tx_buf[0] = (uint8_t)(frame >> 24);
  tx_buf[1] = (uint8_t)(frame >> 16);
  tx_buf[2] = (uint8_t)(frame >> 8);
  tx_buf[3] = (uint8_t)(frame);
  return 4;
}

// ======== T-Format 帧解析 ========
//
// 帧格式 (编码器→主, 单向持续输出):
//   Start(1→0 边沿) + Status(4) + Position(N) + CRC8(8) + Stop(1, 回到高)
//   Start bit: 0 (空闲=高电平, 起始=低电平)
//   Status: 4 bits (bit3=ALMC, bit2=ALMW, bit1=多圈溢出, bit0=位置错误)
//   CRC8 覆盖 Status + Position (不含 Start/Stop)
//
// 帧总长度 = 1 + 4 + N + 8 + 1 = N + 14 bits
static int tformat_parse_frame(Encoder *me) {
  int pos_bits = me->cfg.bits_single + me->cfg.bits_multi;
  int total_bits = 1 + 4 + pos_bits + 8 + 1;  // Start + Status + Pos + CRC8 + Stop
  int total_bytes = (total_bits + 7) / 8;

  if (me->rx_len < (uint16_t)total_bytes) {
    return -1;
  }

  // 字节对齐: T-Format 通常以字节边界开始
  // Start bit 在 bit 0 (值为 0)
  // Status 在 bit 1-4
  uint8_t status_raw = (uint8_t)bits_extract(me->rx_buf, 1, 4);

  // Position 在 bit 5 → 5+pos_bits-1
  me->raw_position = bits_extract(me->rx_buf, 5, pos_bits);

  // CRC8 在 bit 5+pos_bits → 5+pos_bits+7
  int crc_byte_start = 5 + pos_bits;
  uint8_t crc_rx = (uint8_t)bits_extract(me->rx_buf, crc_byte_start, 8);

  // CRC8 覆盖 Status + Position (字节方式)
  int crc_data_bytes = (4 + pos_bits + 7) / 8;  // Status(4bit) + Position
  // 将 Status+Position 打包为字节序列计算 CRC
  int data_bits = 4 + pos_bits;
  uint8_t crc_payload[32];  // 最大容纳 256 bit 帧
  int payload_bytes = (data_bits + 7) / 8;
  for (int i = 0; i < payload_bytes && i < 32; i++) {
    int start_bit = 1 + i * 8;  // Status 从 bit 1 开始
    if (start_bit + 8 <= 1 + data_bits) {
      crc_payload[i] = (uint8_t)bits_extract(me->rx_buf, start_bit, 8);
    } else {
      int remaining = 1 + data_bits - start_bit;
      crc_payload[i] = (uint8_t)(bits_extract(me->rx_buf, start_bit, remaining) << (8 - remaining));
    }
  }
  uint8_t crc_calc = crc8_compute(crc_payload, payload_bytes);

  if (crc_rx != crc_calc && me->cfg.use_crc) {
    return -2;
  }

  // 解码 Status 标志
  // bit3=ALMC (报警清除), bit2=ALMW (报警警告)
  // bit1=多圈溢出, bit0=位置错误
  if (status_raw & 0x01) me->timeout_error = true;  // 位置错误

  return 0;
}

// ======== SinCos 模拟插值 ========
//
// 原理:
//   sin_adc = A * sin(θ) + offset_x
//   cos_adc = A * amplitude * cos(θ + phase_corr) + offset_y
//
// 校准后:
//   sin_norm = (sin_adc - offset_x)
//   cos_norm = (cos_adc - offset_y) / amplitude
//   cos_corr = cos_norm * cos(phase_corr) - sin_norm * sin(phase_corr)  // 正交修正
//   θ = atan2(sin_norm, cos_corr)
//
// 细分插值: 对正余弦信号做 atan2, 分辨率取决于 ADC SNR
//   典型: 12-bit ADC → ~10 bit 角度分辨率, 4096 LSB/rev
//   更高分辨率: 对 ADC 过采样 + 低通滤波

static void sincos_interpolate(Encoder *me, float sin_adc, float cos_adc) {
  float offset_x = me->proto_data.sincos.offset_x;
  float offset_y = me->proto_data.sincos.offset_y;
  float amplitude = me->proto_data.sincos.amplitude;
  float phase_corr = me->proto_data.sincos.phase_corr;

  // 去偏移
  float sn = sin_adc - offset_x;
  float cs = cos_adc - offset_y;

  // 幅值归一化 (保护除零)
  if (amplitude < 0.001f) amplitude = 1.0f;
  cs = cs / amplitude;

  // 正交相位修正 (Heydemann 校正):
  //   cos_corrected = (cos_raw - sin_raw * sin(phase_corr)) / cos(phase_corr)
  if (phase_corr < -0.001f || phase_corr > 0.001f) {
    float sp = sinf(phase_corr);
    float cp = cosf(phase_corr);
    // phase_corr 典型 ±0.05 rad → cos≈0.999, 但加保护防极端配置
    if (cp < 0.001f && cp > -0.001f) {
      me->angle_rad = 0.0f;
      return;
    }
    float cs_corr = (cs - sn * sp) / cp;
    cs = cs_corr;
  }

  // atan2 → 角度 rad [0, 2π)
  me->angle_rad = atan2f(sn, cs);
  if (me->angle_rad < 0.0f) {
    me->angle_rad += M_2PI;
  }

  // 角度 → 单圈位置
  uint64_t max_pos = ((uint64_t)1 << me->cfg.bits_single) - 1;
  me->single_turn = (uint32_t)(me->angle_rad / M_2PI * (float)max_pos);
  me->raw_position = me->single_turn;
}

// ======== PTO 脉冲序列 (ABZ 正交 + UVW 霍尔) ========
//
// ABZ 正交解码状态机 (4 状态):
//   Forward:  00 → 01 → 11 → 10 → 00
//   Reverse:  00 → 10 → 11 → 01 → 00
//
// 每检测到完整 4 步循环, abz_count += 4 (前进) 或 abz_count -= 4 (后退)
//
// Z 相 (Index): 检测到 Z 脉冲时, 如果 AB 在特定状态, 确认找到索引
//
// UVW 霍尔:
//   U V W → Sector
//   1 0 1 → 0
//   1 0 0 → 1
//   1 1 0 → 2
//   0 1 0 → 3
//   0 1 1 → 4
//   0 0 1 → 5

static void pto_quadrature_update(Encoder *me, uint8_t a, uint8_t b, bool z) {
  // 构建当前 AB 状态: A=bit1, B=bit0
  uint8_t ab = ((a & 1) << 1) | (b & 1);
  uint8_t prev = me->proto_data.pto.quad_state;

  // 正交状态转换表: [prev*4 + curr] → 方向 (-1=反向, 0=非法/不变, +1=正向)
  //   Forward:  0→1→3→2→0  (00→01→11→10→00)
  //   Reverse:  0→2→3→1→0  (00→10→11→01→00)
  static const int8_t quadrature_table[16] = {
    // curr=0(00)  curr=1(01)  curr=2(10)  curr=3(11)
     0, +1, -1,  0,   // prev=0 (00): 00→01=+1, 00→10=-1, 00→11=非法
    -1,  0,  0, +1,   // prev=1 (01): 01→00=-1, 01→11=+1
    +1,  0,  0, -1,   // prev=2 (10): 10→00=+1, 10→11=-1
     0, -1, +1,  0,   // prev=3 (11): 11→01=-1, 11→10=+1
  };

  int8_t dir = quadrature_table[prev * 4 + ab];
  me->proto_data.pto.abz_count += dir;
  me->proto_data.pto.quad_state = ab;

  // Z 相索引检测: 在 AB=00 且 Z=1 时确认找到索引
  if (ab == 0 && z && !me->proto_data.pto.index_found) {
    me->proto_data.pto.index_found = true;
    me->proto_data.pto.abz_count = 0;  // 归零计数器
  }

  // 更新 position
  uint32_t single_mask = ((uint64_t)1 << me->cfg.bits_single) - 1;
  me->single_turn = (uint32_t)(me->proto_data.pto.abz_count & (int32_t)single_mask);
  me->raw_position = (uint64_t)me->proto_data.pto.abz_count;
}

// UVW 霍尔扇区解码
static uint8_t uvw_to_sector(uint8_t u, uint8_t v, uint8_t w) {
  uint8_t code = (u & 1) << 2 | (v & 1) << 1 | (w & 1);
  // 标准 6 步霍尔编码: 101→0, 100→1, 110→2, 010→3, 011→4, 001→5
  static const uint8_t hall_to_sector[8] = {
    0xFF,  // 000 非法
    5,     // 001 → sector 5
    3,     // 010 → sector 3
    4,     // 011 → sector 4
    1,     // 100 → sector 1
    0,     // 101 → sector 0
    2,     // 110 → sector 2
    0xFF,  // 111 非法
  };
  return hall_to_sector[code & 0x07];
}

// ======== 协议分发的 ops 实现 ========

// 发送数据到编码器 (命令帧 / MA 脉冲)
static void encoder_send(CommBase *base, const uint8_t *dat, uint16_t len) {
  Encoder *me = container_of(base, Encoder, base);
  (void)dat;
  (void)len;

  switch (me->cfg.proto) {
  case EncProto_BiSS_C:
    // BiSS-C: MA (Master Ack) 脉冲 — RS485 方向切换到发送 → 发时钟+MA → 切回接收
    // 硬件: RS485 DE=1, 发 MA 脉冲 (1 个时钟周期), DE=0
    // BSP 层通过 CommBase.send 接口驱动硬件
    break;
  case EncProto_Endat22: {
    // Endat: 通过 RS485 发命令帧 (6 bit Cmd + 参数 + CRC5)
    uint8_t tx_buf[4];
    int tx_len = endat_build_cmd(tx_buf, ENDAT_CMD_READ_POS, 0);
    // BSP: RS485 DE=1, 发送 tx_len 字节, DE=0
    (void)tx_len;
    break;
  }
  case EncProto_TFormat:
    // T-Format: 编码器持续自主输出, 无需主机发送
    break;
  case EncProto_SinCos:
    // SinCos: 模拟信号, 无需数字发送
    break;
  case EncProto_PTO:
    // PTO: 脉冲由编码器硬件产生, 无需发送
    break;
  default:
    break;
  }
}

// 启动接收 (发起位置读取)
static void encoder_bgn(CommBase *base) {
  Encoder *me = container_of(base, Encoder, base);

  // 清零接收缓冲
  me->rx_len = 0;
  memset(me->rx_buf, 0, sizeof(me->rx_buf));

  switch (me->cfg.proto) {
  case EncProto_BiSS_C:
    // BiSS-C: 触发 RS485 接收
    //   1. RS485 DE=0 (接收模式)
    //   2. 发 MA 脉冲 (通过 encoder_send)
    //   3. 等待 SLO 帧返回 (DMA/中断填入 rx_buf)
    //   4. 帧长度 = (1+1+bits+1+1+6+7)/8 字节
    encoder_send(base, NULL, 0);
    break;
  case EncProto_Endat22:
    // Endat: 发读位置命令 → 等待位置帧
    encoder_send(base, NULL, 0);
    break;
  case EncProto_TFormat:
    // T-Format: 编码器持续输出, SPI/GPIO 捕获电路持续填充
    // 每帧 = (1+4+bits+8+1+7)/8 字节
    break;
  case EncProto_SinCos:
    // SinCos: 触发 ADC 同步采样 Sin/Cos 差分对
    // 典型: ADC 注入组, 由 PWM 触发同步采样
    break;
  case EncProto_PTO:
    // PTO: 脉冲计数由定时器硬件自动完成
    // 读定时器 CNT 寄存器 + AB 状态 (GPIO 输入)
    break;
  default:
    break;
  }
}

// 读取状态字节 — 组合连接/错误状态
static uint8_t encoder_read(CommBase *base) {
  Encoder *me = container_of(base, Encoder, base);

  uint8_t status = 0;
  if (me->connected)     status |= 0x01;
  if (me->crc_error)     status |= 0x02;
  if (me->timeout_error) status |= 0x04;
  return status;
}

// CommBase 虚函数表 (静态常量, 所有实例共享)
static const CommOps encoder_ops = {
  .send = encoder_send,
  .bgn  = encoder_bgn,
  .read = encoder_read,
};

// ======== 构造 / 析构 ========

void encoder_init(Encoder *me, const EncoderCfg *cfg) {
  // 基类初始化
  comm_base_init(&me->base);

  // 绑定配置
  me->cfg = *cfg;

  // 注册虚表
  me->base.ops = &encoder_ops;

  // 清零位置数据
  me->raw_position = 0;
  me->single_turn  = 0;
  me->multi_turn   = 0;
  me->angle_rad    = 0.0f;
  me->velocity     = 0.0f;

  // 清零状态
  me->connected     = false;
  me->crc_error     = false;
  me->timeout_error = false;
  me->error_count   = 0;
  me->last_position = 0;
  me->last_read_time = 0.0f;

  // 清零协议特有数据 + 接收缓冲
  memset(&me->proto_data, 0, sizeof(me->proto_data));
  me->rx_len = 0;
  memset(me->rx_buf, 0, sizeof(me->rx_buf));
}

void encoder_deinit(Encoder *me) {
  me->base.ops = NULL;
  me->connected = false;
}

// ======== 位置读取 (ISR 热路径) ========

int encoder_read_position(Encoder *me) {
  // 未连接则直接返回
  if (!me->connected) {
    return -3;
  }

  int result = 0;

  // 根据协议类型分发到具体解析器
  switch (me->cfg.proto) {
  case EncProto_BiSS_C:
    // BiSS-C: 解析 SLO 帧
    //   帧格式: Start(1) + CDS(1) + Position(N) + Error(1) + Warn(1) + CRC6(6)
    result = biss_parse_frame(me);
    if (result == -2) {
      me->crc_error = true;
      me->error_count++;
    } else if (result == -1) {
      me->timeout_error = true;
      me->error_count++;
    } else {
      me->crc_error = false;
      me->timeout_error = false;
    }
    break;

  case EncProto_Endat22:
    // Endat: 解析位置帧
    //   帧格式: Start(1) + FType(1) + Position + CRC5(5)
    result = endat_parse_position(me);
    if (result == -2) {
      me->crc_error = true;
      me->error_count++;
    } else if (result == -1) {
      me->timeout_error = true;
      me->error_count++;
    } else {
      me->crc_error = false;
      me->timeout_error = false;
    }
    break;

  case EncProto_TFormat:
    // T-Format: 解析串行帧
    //   帧格式: Start(0) + Status(4) + Position + CRC8(8) + Stop(1)
    result = tformat_parse_frame(me);
    if (result == -2) {
      me->crc_error = true;
      me->error_count++;
    } else if (result == -1) {
      me->timeout_error = true;
      me->error_count++;
    } else {
      me->crc_error = false;
      me->timeout_error = false;
    }
    break;

  case EncProto_SinCos: {
    // SinCos: 从 ADC 读 Sin/Cos 值 → atan2 计算角度
    //   rx_buf[0..3] = sin_adc (float32), rx_buf[4..7] = cos_adc (float32)
    //   App 层在调用前填充 rx_buf (从 ADC 结果寄存器)
    if (me->rx_len >= 8) {
      float sin_adc, cos_adc;
      memcpy(&sin_adc, me->rx_buf, 4);
      memcpy(&cos_adc, me->rx_buf + 4, 4);
      sincos_interpolate(me, sin_adc, cos_adc);
      // 直接更新完成, 跳过下面的 raw_position 提取
      return 0;
    }
    result = -1;
    break;
  }

  case EncProto_PTO: {
    // PTO: ABZ 由定时器编码器模式自动计数, UVW 由 GPIO 读取
    //   rx_buf[0] = A(bit1)|B(bit0), rx_buf[1] = Z(bit0)
    //   rx_buf[2] = U(bit2)|V(bit1)|W(bit0)
    if (me->rx_len >= 3) {
      uint8_t ab   = me->rx_buf[0];
      bool    z    = (me->rx_buf[1] & 1) != 0;
      uint8_t uvw  = me->rx_buf[2];
      pto_quadrature_update(me, (ab >> 1) & 1, ab & 1, z);
      me->proto_data.pto.uvw_state = uvw_to_sector((uvw >> 2) & 1, (uvw >> 1) & 1, uvw & 1);
      return 0;
    }
    result = -1;
    break;
  }

  default:
    return -3;  // 未知协议
  }

  if (result != 0) {
    return result;
  }

  // 提取单圈和多圈 (通用后处理; SinCos/PTO 已在上面 return)
  uint32_t single_mask = ((uint64_t)1 << me->cfg.bits_single) - 1;
  me->single_turn = (uint32_t)(me->raw_position & single_mask);

  if (me->cfg.bits_multi > 0) {
    uint32_t multi_mask = ((uint64_t)1 << me->cfg.bits_multi) - 1;
    me->multi_turn = (int32_t)((me->raw_position >> me->cfg.bits_single) & multi_mask);
  } else {
    me->multi_turn = 0;
  }

  // 计算角度 (rad)
  me->angle_rad = (float)me->single_turn
                * M_2PI
                / (float)((uint64_t)1 << me->cfg.bits_single);

  return 0;  // 成功
}

// ======== 速度估计 + 诊断 (主循环) ========

void encoder_update(Encoder *me, float dt) {
  if (dt <= 0.0f) return;

  // 处理单圈回绕 (0 ↔ 2^bits-1)
  uint32_t half_range = 1u << (me->cfg.bits_single - 1);
  int32_t diff = (int32_t)(me->single_turn - (uint32_t)me->last_position);

  if (diff > (int32_t)half_range) {
    // 反向回绕: 如 65535 → 0, 实际正向走了 1 步
    diff -= (int32_t)(1u << me->cfg.bits_single);
  } else if (diff < -(int32_t)half_range) {
    // 正向回绕: 如 0 → 65535, 实际反向走了 1 步
    diff += (int32_t)(1u << me->cfg.bits_single);
  }

  // 速度 = 位置差 / 时间, 转换为 rad/s
  float pos_per_rev = (float)((uint64_t)1 << me->cfg.bits_single);
  me->velocity = (float)diff * M_2PI / pos_per_rev / dt;

  // 保存当前位置供下次速度计算
  me->last_position = me->raw_position;
  me->last_read_time += dt;
}

// ======== 访问器 ========

float encoder_get_angle(const Encoder *me) {
  return me->angle_rad;
}

float encoder_get_velocity(const Encoder *me) {
  return me->velocity;
}

uint32_t encoder_get_error_count(const Encoder *me) {
  return me->error_count;
}

// ======== 状态查询 ========

bool encoder_is_ok(const Encoder *me) {
  return me->connected && !me->crc_error && !me->timeout_error;
}

void encoder_reset(Encoder *me) {
  me->crc_error     = false;
  me->timeout_error = false;
  // error_count 不归零 — 保留累计统计
}
