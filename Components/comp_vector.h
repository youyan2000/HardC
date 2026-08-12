// 向量/矩阵批处理数学库 — 基本向量运算 + Vector3 三相便捷结构体
//
// 来源: TI controlSUITE dsp/FPU/v1_50 (fpu_vector.h) — C-OOP 纯C float 重实现
//
// 三后端策略:
//   默认 float — 直接 for 循环 (适用于 M4F/M7/FPU C2000)
//   BSP_DSP_ARCH >= 2: 可走 CMSIS-DSP arm_xxx 硬件加速 (SIMD)
//   BSP_DSP_ARCH == 3: C2000Ware CLA 协处理器 (需用户工程提供 DSP.h)
//
// static inline 函数适合 ISR 热路径, 零调用开销
// 所有基本向量运算操作 float* 数组 + int len, 调用者负责边界安全

#ifndef COMP_VECTOR_H
#define COMP_VECTOR_H

#include <math.h>

// ======== 基本向量运算 ========

// dst[i] = a[i] + b[i]
// BSP_DSP_ARCH >= 2: 可使用 CMSIS-DSP arm_add_f32
static inline void vec_add_f32(const float *a, const float *b, float *dst, int len) {
  for (int i = 0; i < len; i++) {
    dst[i] = a[i] + b[i];
  }
}

// dst[i] = a[i] - b[i]
// BSP_DSP_ARCH >= 2: 可使用 CMSIS-DSP arm_sub_f32
static inline void vec_sub_f32(const float *a, const float *b, float *dst, int len) {
  for (int i = 0; i < len; i++) {
    dst[i] = a[i] - b[i];
  }
}

// dst[i] = a[i] * b[i] (逐元素乘法)
// BSP_DSP_ARCH >= 2: 可使用 CMSIS-DSP arm_mult_f32
static inline void vec_mul_f32(const float *a, const float *b, float *dst, int len) {
  for (int i = 0; i < len; i++) {
    dst[i] = a[i] * b[i];
  }
}

// dst[i] = src[i] * scalar
// BSP_DSP_ARCH >= 2: 可使用 CMSIS-DSP arm_scale_f32
static inline void vec_scale_f32(const float *src, float s, float *dst, int len) {
  for (int i = 0; i < len; i++) {
    dst[i] = src[i] * s;
  }
}

// 向量模长 = sqrt(sum(src[i]^2))
// BSP_DSP_ARCH >= 2: 可使用 CMSIS-DSP arm_dot_prod_f32 + arm_sqrt_f32
static inline float vec_mag_f32(const float *src, int len) {
  float sum_sq = 0.0f;
  for (int i = 0; i < len; i++) {
    sum_sq += src[i] * src[i];
  }
  return sqrtf(sum_sq);
}

// 点积 = sum(a[i] * b[i])
// BSP_DSP_ARCH >= 2: 可使用 CMSIS-DSP arm_dot_prod_f32
static inline float vec_dot_f32(const float *a, const float *b, int len) {
  float sum = 0.0f;
  for (int i = 0; i < len; i++) {
    sum += a[i] * b[i];
  }
  return sum;
}

// 最大绝对值 = max(|src[i]|)
// BSP_DSP_ARCH >= 2: 可使用 CMSIS-DSP arm_abs_max_f32
static inline float vec_absmax_f32(const float *src, int len) {
  float max_val = 0.0f;
  for (int i = 0; i < len; i++) {
    float abs_val = fabsf(src[i]);
    if (abs_val > max_val) {
      max_val = abs_val;
    }
  }
  return max_val;
}

// 最大值索引 = argmax(src[i])
// BSP_DSP_ARCH >= 2: 可使用 CMSIS-DSP arm_max_f32 (返回 value + index)
static inline int vec_argmax_f32(const float *src, int len) {
  if (len <= 0) return -1;
  int max_idx = 0;
  float max_val = src[0];
  for (int i = 1; i < len; i++) {
    if (src[i] > max_val) {
      max_val = src[i];
      max_idx = i;
    }
  }
  return max_idx;
}

// 算术平均 = sum(src[i]) / len
// BSP_DSP_ARCH >= 2: 可使用 CMSIS-DSP arm_mean_f32
static inline float vec_mean_f32(const float *src, int len) {
  if (len <= 0) return 0.0f;
  float sum = 0.0f;
  for (int i = 0; i < len; i++) {
    sum += src[i];
  }
  return sum / (float)len;
}

// 填充数组为常数: dst[i] = val
// BSP_DSP_ARCH >= 2: 可使用 CMSIS-DSP arm_fill_f32
static inline void vec_fill_f32(float *dst, float val, int len) {
  for (int i = 0; i < len; i++) {
    dst[i] = val;
  }
}

// 拷贝数组: dst[i] = src[i]
// BSP_DSP_ARCH >= 2: 可使用 CMSIS-DSP arm_copy_f32
static inline void vec_copy_f32(const float *src, float *dst, int len) {
  for (int i = 0; i < len; i++) {
    dst[i] = src[i];
  }
}

// 逐元素钳位: dst[i] = clamp(src[i], lo, hi)
// BSP_DSP_ARCH >= 2: 可使用 CMSIS-DSP arm_clip_f32
static inline void vec_clamp_f32(const float *src, float lo, float hi, float *dst, int len) {
  for (int i = 0; i < len; i++) {
    float val = src[i];
    if (val < lo) {
      dst[i] = lo;
    } else if (val > hi) {
      dst[i] = hi;
    } else {
      dst[i] = val;
    }
  }
}

// ======== Vector3 三相便捷结构体 ========

// 三相向量结构体 — 三元素 (a, b, c), 用于电压/电流/磁链等三相物理量
typedef struct {
  float a, b, c;
} Vector3;

// 默认初始化
#define VECTOR3_DEFAULTS { 0.0f, 0.0f, 0.0f }

// Vector3 加法: a+b, b+b, c+c 逐分量
static inline Vector3 vec3_add(Vector3 x, Vector3 y) {
  Vector3 r;
  r.a = x.a + y.a;
  r.b = x.b + y.b;
  r.c = x.c + y.c;
  return r;
}

// Vector3 减法: a-b, b-b, c-c 逐分量
static inline Vector3 vec3_sub(Vector3 x, Vector3 y) {
  Vector3 r;
  r.a = x.a - y.a;
  r.b = x.b - y.b;
  r.c = x.c - y.c;
  return r;
}

// Vector3 标量乘: 每分量乘 s
static inline Vector3 vec3_scale(Vector3 x, float s) {
  Vector3 r;
  r.a = x.a * s;
  r.b = x.b * s;
  r.c = x.c * s;
  return r;
}

// Vector3 点积: a*a + b*b + c*c
static inline float vec3_dot(Vector3 x, Vector3 y) {
  return x.a * y.a + x.b * y.b + x.c * y.c;
}

// Vector3 模长: sqrt(a*a + b*b + c*c)
static inline float vec3_mag(Vector3 x) {
  return sqrtf(x.a * x.a + x.b * x.b + x.c * x.c);
}

// Vector3 叉积: x × y
static inline Vector3 vec3_cross(Vector3 x, Vector3 y) {
  Vector3 r;
  r.a = x.b * y.c - x.c * y.b;
  r.b = x.c * y.a - x.a * y.c;
  r.c = x.a * y.b - x.b * y.a;
  return r;
}

// Vector3 逐分量钳位: 每分量 clamp 到 [lo, hi]
static inline Vector3 vec3_clamp(Vector3 x, float lo, float hi) {
  Vector3 r;
  r.a = (x.a < lo) ? lo : ((x.a > hi) ? hi : x.a);
  r.b = (x.b < lo) ? lo : ((x.b > hi) ? hi : x.b);
  r.c = (x.c < lo) ? lo : ((x.c > hi) ? hi : x.c);
  return r;
}

#endif  // COMP_VECTOR_H
