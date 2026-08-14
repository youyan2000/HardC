// ASK/OOK 无线充电信令编解码 — 包级编码/偶校验 + 2000Hz 包络采样解码状态机
//
// 来源: 无线充电 (WPT) 前端信令 — E 侧 2000Hz ASK 包络调制
//
// 包格式 (12 bit, MSB-first, 存于 uint16_t 低 12 位):
//   bit 11     : req       需求位 (1=请求充电)
//   bit 10..3  : power_w   8-bit 功率值 (0..255)
//   bit 2      : parity    EVEN 偶校验 (覆盖 bit 11..3, 即 req+power_w)
//   bit 1..0   : pad       恒 0
//
// 奇偶校验: EVEN — 包内 '1' 总数 (含校验位) 为偶数; 全 12 位 XOR 归约结果为
//   0 → ask_parity_ok 为 true. 任何单比特翻转都会破坏偶性, 从而被检出.
//
// 解码: 2000Hz 包络采样 → 100Hz 位时钟 (ASK_SAMPLES_PER_BIT=20 采样/位).
//   每 20 采样多数表决 (ones_in_bit ≥ 10 → 位='1'). 位边界自由运行 (无前导码):
//   首个载波存在采样触发同步, 计为位 0 的第 1 采样; 收满 12 位后自动重同步
//   (连续流). 发射端需在包间插入低电平间隙以对齐位边界 — 块级编解码可接受.
//
// 用法: 每 2000Hz 采样调用一次 AskDecode_Update(me, carrier); 返回 true =
//   收满 12-bit 包 (读 me->pkt, ask_parity_ok 校验通过后 ask_decode_pkt 提取).

#ifndef COMP_ASK_H
#define COMP_ASK_H

#include <stdint.h>
#include <stdbool.h>

// ======== 时序常量 ========
#define ASK_CARRIER_HZ 2000u                                    // 载波/包络采样率 (Hz)
#define ASK_BIT_RATE_HZ 100u                                    // 位率 (Hz) → 20 采样/位
#define ASK_PKT_BITS 12u                                        // 包长 (位)
#define ASK_SAMPLES_PER_BIT (ASK_CARRIER_HZ / ASK_BIT_RATE_HZ)  // 20 采样/位

// ======== 编码/校验/解码 ========

// 编码: 1 req + 8 power + 1 偶校验 (over bits 11..3) + 2 pad → 12-bit 包 (低 12 位)
static inline uint16_t ask_encode_pkt(bool req, uint8_t power_w) {
  uint16_t data = (uint16_t) (((req ? 1u : 0u) << 11) | ((uint16_t) power_w << 3));
  uint16_t p = data;
  p ^= (uint16_t) (p >> 8);
  p ^= (uint16_t) (p >> 4);
  p ^= (uint16_t) (p >> 2);
  p ^= (uint16_t) (p >> 1);
  return (uint16_t) (data | ((p & 1u) << 2));
}

// 校验: 全 12 位偶校验通过 = true (任何单比特翻转 → false)
static inline bool ask_parity_ok(uint16_t pkt) {
  uint16_t p = pkt;
  p ^= (uint16_t) (p >> 8);
  p ^= (uint16_t) (p >> 4);
  p ^= (uint16_t) (p >> 2);
  p ^= (uint16_t) (p >> 1);
  return (p & 1u) == 0u;
}

// 解码: 提取 req (bit11) 和 power_w (bits 10..3); 指针可为 NULL (跳过该输出)
static inline void ask_decode_pkt(uint16_t pkt, bool *req, uint8_t *power_w) {
  if (req)
    *req = ((pkt >> 11) & 1u) != 0u;
  if (power_w)
    *power_w = (uint8_t) ((pkt >> 3) & 0xFFu);
}

// ======== 解码状态机 (2000Hz 包络采样 → 100Hz 位时钟) ========
typedef struct {
  uint16_t pkt;         // 组装中的 12-bit 包 (bit11 先收, MSB-first)
  uint8_t bit_pos;      // 已收位数 (0..12)
  uint8_t sample_cnt;   // 当前位内采样计数 (0..19)
  uint8_t ones_in_bit;  // 当前位内载波存在采样计数
  bool synced;          // 已同步到包起始 (首个载波存在采样触发)
} AskDecode;

static inline void AskDecode_Init(AskDecode *me) {
  me->pkt = 0u;
  me->bit_pos = 0u;
  me->sample_cnt = 0u;
  me->ones_in_bit = 0u;
  me->synced = false;
}

// 每 2000Hz 包络采样调用一次. 返回 true = 收满 12-bit 包 (读 me->pkt).
// 语义: 未同步且 carrier=0 → false; 未同步且 carrier=1 → 同步并累积本采样;
//   第 20 采样多数表决判位并移入 pkt; 收满 ASK_PKT_BITS 位 → synced=false 返回 true.
static inline bool AskDecode_Update(AskDecode *me, bool carrier) {
  // 未同步: 首个载波存在采样作为包起始触发
  if (!me->synced) {
    if (!carrier) {
      return false;
    }
    me->synced = true;
    me->bit_pos = 0u;
    me->sample_cnt = 0u;
    me->ones_in_bit = 0u;
    me->pkt = 0u;
  }

  // 累积当前位内载波存在采样
  if (carrier) {
    me->ones_in_bit++;
  }
  me->sample_cnt++;

  // 位边界 (第 20 采样): 多数表决
  if (me->sample_cnt < ASK_SAMPLES_PER_BIT) {
    return false;
  }

  uint8_t bit = (me->ones_in_bit >= (ASK_SAMPLES_PER_BIT / 2)) ? 1u : 0u;
  me->pkt = (uint16_t) ((me->pkt << 1) | bit);
  me->sample_cnt = 0u;
  me->ones_in_bit = 0u;
  me->bit_pos++;

  if (me->bit_pos >= ASK_PKT_BITS) {
    me->synced = false;  // 收满一包 → 自动重同步 (连续流)
    return true;
  }
  return false;
}

#endif  // COMP_ASK_H
