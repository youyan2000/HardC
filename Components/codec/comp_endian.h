// 大小端转换库 — 16/32 位原地翻转 + 双缓冲拷贝, 无依赖, ISR 安全

#ifndef COMP_ENDIAN_H
#define COMP_ENDIAN_H

#include <stdint.h>

// 16 位大小端翻转 (原地): 字节 0↔1 交换
static inline void math_endian_reverse_16(void *addr) {
  uint8_t *p = (uint8_t *)addr;
  uint8_t t = p[0];
  p[0] = p[1];
  p[1] = t;
}

// 16 位大小端翻转 (拷贝): 从 src 读, 翻转后写入 dst
static inline void math_endian_reverse_16_copy(const void *src, void *dst) {
  const uint8_t *s = (const uint8_t *)src;
  uint8_t *d = (uint8_t *)dst;
  d[0] = s[1];
  d[1] = s[0];
}

// 32 位大小端翻转 (原地): 字节 0↔3, 1↔2 交换
static inline void math_endian_reverse_32(void *addr) {
  uint8_t *p = (uint8_t *)addr;
  uint8_t t0 = p[0], t1 = p[1];
  p[0] = p[3];
  p[1] = p[2];
  p[2] = t1;
  p[3] = t0;
}

// 32 位大小端翻转 (拷贝): 从 src 读, 翻转后写入 dst
static inline void math_endian_reverse_32_copy(const void *src, void *dst) {
  const uint8_t *s = (const uint8_t *)src;
  uint8_t *d = (uint8_t *)dst;
  d[0] = s[3];
  d[1] = s[2];
  d[2] = s[1];
  d[3] = s[0];
}

#endif  // COMP_ENDIAN_H
