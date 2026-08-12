// 数学工具库

#include "comp_math.h"

/* ========================= 平方根倒数 ========================= */

// 快速平方根倒数 (Quake III 算法)
float inv_sqrtf(float x) {
  float xhalf = 0.5f * x;
  int32_t i = *(int32_t *)&x;
  i = 0x5f3759df - (i >> 1);
  x = *(float *)&i;
  x = x * (1.5f - xhalf * x * x);
  return x;
}

/* ========================= 大小端转换 ========================= */

// 16 位大小端翻转 (原地): 字节 0↔1 交换
void math_endian_reverse_16(void *addr) {
  uint8_t *p = (uint8_t *)addr;
  uint8_t t = p[0];
  p[0] = p[1];
  p[1] = t;
}

// 16 位大小端翻转 (拷贝): 从 src 读, 翻转后写入 dst
void math_endian_reverse_16_copy(const void *src, void *dst) {
  const uint8_t *s = (const uint8_t *)src;
  uint8_t *d = (uint8_t *)dst;
  d[0] = s[1];
  d[1] = s[0];
}

// 32 位大小端翻转 (原地): 字节 0↔3, 1↔2 交换
void math_endian_reverse_32(void *addr) {
  uint8_t *p = (uint8_t *)addr;
  uint8_t t0 = p[0], t1 = p[1];
  p[0] = p[3];
  p[1] = p[2];
  p[2] = t1;
  p[3] = t0;
}

// 32 位大小端翻转 (拷贝): 从 src 读, 翻转后写入 dst
void math_endian_reverse_32_copy(const void *src, void *dst) {
  const uint8_t *s = (const uint8_t *)src;
  uint8_t *d = (uint8_t *)dst;
  d[0] = s[3];
  d[1] = s[2];
  d[2] = s[1];
  d[3] = s[0];
}

/* =========================== 校验和 =========================== */

// 8 位无符号校验和: 逐字节累加, 自然溢出
uint8_t math_sum_u8(const uint8_t *addr, uint32_t len) {
  uint8_t sum = 0;
  for (uint32_t i = 0; i < len; i++) sum += addr[i];
  return sum;
}

// 16 位无符号校验和
uint16_t math_sum_u16(const uint16_t *addr, uint32_t len) {
  uint16_t sum = 0;
  for (uint32_t i = 0; i < len; i++) sum += addr[i];
  return sum;
}

// 32 位无符号校验和
uint32_t math_sum_u32(const uint32_t *addr, uint32_t len) {
  uint32_t sum = 0;
  for (uint32_t i = 0; i < len; i++) sum += addr[i];
  return sum;
}
