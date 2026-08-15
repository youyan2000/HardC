// 校验和计算库 — 8/16/32 位无符号累加 (自然溢出), ISR 安全

#ifndef COMP_CHECKSUM_H
#define COMP_CHECKSUM_H

#include <stdint.h>

// 8 位无符号校验和: 逐字节累加, 自然溢出
static inline uint8_t math_sum_u8(const uint8_t *addr, uint32_t len) {
  uint8_t sum = 0;
  for (uint32_t i = 0; i < len; i++) sum += addr[i];
  return sum;
}

// 16 位无符号校验和
static inline uint16_t math_sum_u16(const uint16_t *addr, uint32_t len) {
  uint16_t sum = 0;
  for (uint32_t i = 0; i < len; i++) sum += addr[i];
  return sum;
}

// 32 位无符号校验和
static inline uint32_t math_sum_u32(const uint32_t *addr, uint32_t len) {
  uint32_t sum = 0;
  for (uint32_t i = 0; i < len; i++) sum += addr[i];
  return sum;
}

#endif  // COMP_CHECKSUM_H
