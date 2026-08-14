// 平台层数学工具库
// 提供限幅、绝对值、线性映射、快速开方等通用数学操作
// static inline 函数适合 ISR 热路径, 零调用开销

#ifndef COMP_MATH_H
#define COMP_MATH_H

#include <stdint.h>
#include "bsp_dsp.h"    // 硬件加速 sqrt (CMSIS-DSP / C2000 TMU / 纯C回退)

// π / 2π 常量 — 全库唯一 float 源头, 其他模块 #ifndef 卫哨自动让路
// 系统 <math.h> 可能已定义 double 版 M_PI (如 newlib 无条件 #define M_PI 3.14159265358979323846)
// 软浮点 double 在 M4F/C2000 上是性能灾难, 且与 float 的 M_2PI 类型不一致 → 先 #undef 再定义
#undef M_PI
#undef M_2PI
#ifndef M_2PI
#define M_2PI 6.28318530718f  // 离 2π 最近的 float (6.2831855f)
#endif
#ifndef M_PI
#define M_PI 3.14159265358979f  // 离 π 最近的 float (3.1415927f)
#endif

// 硬件加速数学宏 — 默认走 BSP 平台分发, 工程可用 #define 覆盖加速后端
//   sqrt  → bsp_sqrt_f32  (VSQRT / C2000 TMU __sqrt / 纯C牛顿)
//   isqrt → bsp_isqrt_f32 (C2000 CLA CLAisqrt / 纯C 1/sqrtf)
//   abs   → fabsf         (FPU VABS 单指令)
#ifndef MATH_SQRT
#define MATH_SQRT(x)  bsp_sqrt_f32((x))
#endif
#ifndef MATH_ISQRT
#define MATH_ISQRT(x) bsp_isqrt_f32((x))
#endif
#ifndef MATH_ABS
#define MATH_ABS(x)   fabsf((x))
#endif

/* ================================ 宏定义 ================================ */

// 计算时间戳相差秒数
#define TIME_DIFF(_start, _end)    ((float)((_end) - (_start)) / 1000000.0f)
#define TIME_DIFF_US(_start, _end) ((float)((_end) - (_start)) / 1000000.0f)
#define TIME_DIFF_MS(_start, _end) ((float)((_end) - (_start)) / 1000.0f)

// 角度 (度) → 弧度
#define ANGLE2RADIAN(_angle) ((_angle) / 360.0f * M_2PI)

// 角速度 (度/秒) → 单位时间变化量 (弧度)
#define SPEED2DELTA(_speed, _dt) (ANGLE2RADIAN((_speed) * (_dt)))

#ifndef MAX
// 返回两个值中的最大值
// 注意: 参数有副作用时会被双次求值, 仅用于简单表达式
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef MIN
// 返回两个值中的最小值
#define MIN(a, b) ((a) < (b) ? (a) : (b))
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

// 绝对值 (float — 硬件 VABS / int16)
static inline float math_abs_f(float x) {
  return MATH_ABS(x);
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

// 平方根倒数 (Quake III 快速算法, Legacy — 非 FPU/TMU 目标的软回退)
// 有 FPU/TMU 时推荐 math_sqrt_f32 (硬件加速)
static inline float math_inv_sqrtf(float x) {
  float xhalf = 0.5f * x;
  int32_t i = *(int32_t *)&x;
  i = 0x5f3759df - (i >> 1);
  x = *(float *)&i;
  x = x * (1.5f - xhalf * x * x);
  return x;
}

// 硬件加速平方根 (通过 MATH_SQRT 宏分发)
//   STM32: CMSIS-DSP arm_sqrt_f32 → FPU VSQRT 单周期
//   C2000: TMU __sqrt
//   回退:  牛顿迭代法
static inline float math_sqrt_f32(float x) {
  return MATH_SQRT(x);
}

#endif
