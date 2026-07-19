// 平台层数学工具库
// 提供限幅、绝对值、大小端转换、求校验和等通用数学操作
// static inline 函数适合 ISR 热路径, 零调用开销

#ifndef COMP_MATH_H
#define COMP_MATH_H

#include <stdint.h>
#include "../BSP/bsp_dsp.h"    // 硬件加速 sqrt (CMSIS-DSP / C2000 TMU / 纯C回退)

// 2π 常量 (其他模块也会定义, 用 #ifndef 防冲突)
#ifndef M_2PI
#define M_2PI 6.283185f
#endif

/* ================================ 宏定义 ================================ */

// 计算时间戳相差秒数
#define TIME_DIFF(_start, _end)    ((float)(_end - _start) / 1000000.0f)
#define TIME_DIFF_US(_start, _end) ((float)(_end - _start) / 1000000.0f)
#define TIME_DIFF_MS(_start, _end) ((float)(_end - _start) / 1000.0f)

// 角度 (度) → 弧度
#define ANGLE2RADIAN(_angle) (_angle / 360.0f * M_2PI)

// 角速度 (度/秒) → 单位时间变化量 (弧度)
#define SPEED2DELTA(_speed, _dt) (ANGLE2RADIAN(_speed * _dt))

#ifndef MAX
// 返回两个值中的最大值
#define MAX(a, b)               \
  ({                            \
    __typeof__(a) _a = (a);     \
    __typeof__(b) _b = (b);     \
    _a > _b ? _a : _b;          \
  })
#endif

#ifndef MIN
// 返回两个值中的最小值
#define MIN(a, b)               \
  ({                            \
    __typeof__(a) _a = (a);     \
    __typeof__(b) _b = (b);     \
    _a < _b ? _a : _b;          \
  })
#endif

#ifndef CLAMP
// 限幅: 将 a 钳制在 [min, max] 范围内
#define CLAMP(a, min, max) (MIN((max), MAX((min), (a))))
#endif

#ifndef ABS
// 取绝对值
#define ABS(a) ((a) < 0 ? -(a) : (a))
#endif

/* ========================== static inline 函数 ========================== */

// 限幅 (原地修改)
static inline void math_constrain_i8(int8_t *x, int8_t min, int8_t max) {
  if (*x < min) *x = min;
  else if (*x > max) *x = max;
}

static inline void math_constrain_i16(int16_t *x, int16_t min, int16_t max) {
  if (*x < min) *x = min;
  else if (*x > max) *x = max;
}

static inline void math_constrain_f(float *x, float min, float max) {
  if (*x < min) *x = min;
  else if (*x > max) *x = max;
}

// 绝对值 (float / int16)
static inline float math_abs_f(float x) {
  return (x > 0) ? x : -x;
}

static inline int16_t math_abs_i16(int16_t x) {
  return (x > 0) ? x : -x;
}

/* =========================== 函数声明 =========================== */

// 限幅 (返回值版本, 不修改原值)
static inline float math_clamp_f(float val, float min, float max) {
  if (val < min) return min;
  if (val > max) return max;
  return val;
}

static inline int32_t math_clamp_i32(int32_t val, int32_t min, int32_t max) {
  if (val < min) return min;
  if (val > max) return max;
  return val;
}

static inline uint32_t math_clamp_u32(uint32_t val, uint32_t min, uint32_t max) {
  if (val < min) return min;
  if (val > max) return max;
  return val;
}

// 死区: |val| < deadzone → 0, 否则返回原值
static inline float math_deadzone_f(float val, float deadzone) {
  if (val > deadzone)  return val;
  if (val < -deadzone) return val;
  return 0.0f;
}

// 线性映射: x ∈ [in_min, in_max] → [out_min, out_max]
static inline float math_map_f(float x, float in_min, float in_max,
                                        float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) / (in_max - in_min) * (out_max - out_min) + out_min;
}

// 平方根倒数 (快速算法, Legacy — 推荐用 math_sqrt_f32)
float inv_sqrtf(float x);

// 硬件加速平方根 (通过 BSP/bsp_dsp.h 分发)
//   STM32: CMSIS-DSP arm_sqrt_f32 → FPU VSQRT 单周期
//   C2000: TMU __sqrt (后续实现)
//   回退:  牛顿迭代法
static inline float math_sqrt_f32(float x) {
  return bsp_sqrt_f32(x);
}

// 大小端转换 (原地修改)
void math_endian_reverse_16(void *addr);
void math_endian_reverse_32(void *addr);

// 大小端转换 (源 → 目标, 双缓冲)
void math_endian_reverse_16_copy(const void *src, void *dst);
void math_endian_reverse_32_copy(const void *src, void *dst);

// 无符号校验和 (8/16/32 位)
uint8_t  math_sum_u8 (const uint8_t  *addr, uint32_t len);
uint16_t math_sum_u16(const uint16_t *addr, uint32_t len);
uint32_t math_sum_u32(const uint32_t *addr, uint32_t len);

#endif
