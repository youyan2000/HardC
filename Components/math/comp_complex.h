// 复数运算库 — 直角坐标 + 极坐标, 值类型 ISR 友好
//
// 来源: TI controlSUITE vcu/ComplexMath (complex_math.h)
//
// 全部 static inline, ARM EABI 硬浮点 ABI 直接走 VFP 寄存器 s0/s1, 零栈开销
// complex_expj() 是矢量旋转核心: complex_mul(v, complex_expj(theta)) = 旋转角度 theta
// 与 comp_vector.h (Vector3 三相向量) 互补 — 这是真正的复数 (re, im)

#ifndef COMP_COMPLEX_H
#define COMP_COMPLEX_H

#include <math.h>

// Complex32 — 直角坐标 32-bit 浮点复数 (值类型, ISR 友好)
// 按值传递/返回, ARM EABI 硬浮点 ABI 走 VFP 寄存器, 零栈开销
typedef struct {
  float re;    // 实部
  float im;    // 虚部
} Complex32;

// 构造宏
#define COMPLEX32(re_val, im_val) ((Complex32){ (re_val), (im_val) })
#define COMPLEX32_ZERO             ((Complex32){ 0.0f, 0.0f })

// 加法: (a.re + b.re, a.im + b.im)
static inline Complex32 complex_add(Complex32 a, Complex32 b) {
  Complex32 r;
  r.re = a.re + b.re;
  r.im = a.im + b.im;
  return r;
}

// 减法: (a.re - b.re, a.im - b.im)
static inline Complex32 complex_sub(Complex32 a, Complex32 b) {
  Complex32 r;
  r.re = a.re - b.re;
  r.im = a.im - b.im;
  return r;
}

// 乘法: (a.re×b.re - a.im×b.im, a.re×b.im + a.im×b.re)
static inline Complex32 complex_mul(Complex32 a, Complex32 b) {
  Complex32 r;
  r.re = a.re * b.re - a.im * b.im;
  r.im = a.re * b.im + a.im * b.re;
  return r;
}

// 除法: a / b
// 用共轭分母避免复数除法的不稳定性:
//   a / b = (a × conj(b)) / |b|²
static inline Complex32 complex_div(Complex32 a, Complex32 b) {
  Complex32 r;
  float den = b.re * b.re + b.im * b.im;
  if (den < 1e-12f) {
    return COMPLEX32_ZERO;
  }
  r.re = (a.re * b.re + a.im * b.im) / den;
  r.im = (a.im * b.re - a.re * b.im) / den;
  return r;
}

// 模长: sqrt(re² + im²)
static inline float complex_mag(Complex32 a) {
  return sqrtf(a.re * a.re + a.im * a.im);
}

// 模长平方: re² + im² (避免 sqrt, 用于比较)
static inline float complex_mag_sq(Complex32 a) {
  return a.re * a.re + a.im * a.im;
}

// 相位: atan2(im, re), 返回值 ∈ [-π, π]
static inline float complex_phase(Complex32 a) {
  return atan2f(a.im, a.re);
}

// 共轭: (re, -im)
static inline Complex32 complex_conj(Complex32 a) {
  Complex32 r;
  r.re = a.re;
  r.im = -a.im;
  return r;
}

// e^(jθ) = cosθ + j·sinθ — DDS/锁相环/矢量旋转核心
// 用法: rotated = complex_mul(v, complex_expj(theta))
static inline Complex32 complex_expj(float theta) {
  Complex32 r;
  r.re = cosf(theta);
  r.im = sinf(theta);
  return r;
}

// 标量乘: (a.re × s, a.im × s)
static inline Complex32 complex_scale(Complex32 a, float s) {
  Complex32 r;
  r.re = a.re * s;
  r.im = a.im * s;
  return r;
}

// 从极坐标构造: mag × e^(j×phase)
static inline Complex32 complex_polar(float mag, float phase) {
  Complex32 r;
  r.re = mag * cosf(phase);
  r.im = mag * sinf(phase);
  return r;
}

// 取反: (-re, -im) — 等效于 180° 旋转
static inline Complex32 complex_neg(Complex32 a) {
  Complex32 r;
  r.re = -a.re;
  r.im = -a.im;
  return r;
}

// 归一化: a / |a| (单位复数, 模长=1)
// 零向量返回 (1, 0)
static inline Complex32 complex_normalize(Complex32 a) {
  float mag = complex_mag(a);
  if (mag < 1e-12f) {
    return COMPLEX32(1.0f, 0.0f);
  }
  return complex_scale(a, 1.0f / mag);
}

#endif  // COMP_COMPLEX_H
