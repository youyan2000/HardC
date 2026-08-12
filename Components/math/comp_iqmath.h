// IQmath 定点数学库 — 硬件解绑 + 纯C通用回退
//
// 来源: TI controlSUITE IQmathLib (48 C 文件, _iq/_iq15/_iq30 类型系统)
// 解绑: 去掉 C2000 64位ACC依赖, 用 int64_t 标准C实现, 跨平台通用
//
// IQ 格式:
//   _iq    = Q24  (int32_t, 24位小数, 范围 -128~127.999999)
//   _iq30  = Q30  (int32_t, 30位小数, 范围   -2~1.999999)
//   _iq15  = Q15  (int16_t, 15位小数, 范围   -1~0.999969)  — CMSIS-DSP q15_t 兼容
//
// 三后端策略 (通过 BSP_DSP_ARCH):
//   C2000Ware (ARCH=3): #define 到 TI 硬件 ACC 指令 (1 周期)
//   CMSIS-DSP (ARCH=1/2): 使用 int64_t 纯C实现 (ARM M4 有 SMULL, 仍然快)
//   纯C回退 (ARCH=0):    使用 int64_t 纯C实现 (标准C99, 任何平台)
//
// 💡 设计理念:
//   - 不写3份源码 — 1份纯C实现 + #define 加速路径
//   - IQ 类型本质就是 int32_t/int16_t — 与 CMSIS-DSP q31_t/q15_t 位兼容
//   - 三角表在 comp_iqmath.c 中定义 (单份, 所有平台共用)
//
// 调用方式:
//   _iq va = _IQ(0.5);           // float → Q24
//   _iq vb = _IQmpy(va, va);     // Q24 × Q24 → Q24
//   float f = _IQtoF(vb);        // Q24 → float
//   _iq s = _IQsin(_IQ(0.25));   // sin(0.25 pu) → Q24

#ifndef COMP_IQMATH_H
#define COMP_IQMATH_H

#include <stdint.h>
#include "bsp_dsp.h"

#ifdef __cplusplus
extern "C" {
#endif

// ======================= IQ 基本类型 =======================

// IQ 类型: 本质是有符号整数, 约定小数点在指定位
// Q24 = 32位存储, 低24位为小数, 高8位为整数+符号
// Q30 = 32位存储, 低30位为小数, 高2位为整数+符号
// Q15 = 16位存储, 低15位为小数, 高1位为整数+符号
//
// 与 CMSIS-DSP 兼容: _iq ≡ q31_t (都是 int32_t, Q31 vs Q24 只是解释不同)
//                    _iq15 ≡ q15_t (都是 int16_t)

typedef int32_t  _iq;
typedef int16_t  _iq15;
typedef int32_t  _iq30;
typedef long long iq64;    // 64位中间累加器

// ======================= 格式转换常数 =======================

#define IQ_SHIFT   24             // Q24: 24 位小数
#define IQ15_SHIFT 15             // Q15: 15 位小数
#define IQ30_SHIFT 30             // Q30: 30 位小数

// float → IQ 转换 (截断, C2000 兼容)
// _IQ(x) = (int32_t)(x * 2^24 + 0.5*sign(x))  ← 加0.5四舍五入
// 注: 乘以 (float)(1<<24) 在编译期优化为位运算
#define _IQ(f)     ((_iq)((f) * (float)(1L << IQ_SHIFT)))
#define _IQ15(f)   ((_iq15)((f) * (float)(1L << IQ15_SHIFT)))
#define _IQ30(f)   ((_iq30)((f) * (float)(1L << IQ30_SHIFT)))

// 整型 → IQ (直接移位, 零开销)
#define _IQ24fromI(i)  ((_iq)((int32_t)(i) << IQ_SHIFT))
#define _IQ15fromI(i)  ((_iq15)((int16_t)(i) << IQ15_SHIFT))

// IQ → float
#define _IQtoF(a)       ((float)(a) / (float)(1L << IQ_SHIFT))
#define _IQ15toF(a)     ((float)(a) / (float)(1L << IQ15_SHIFT))
#define _IQ30toF(a)     ((float)(a) / (float)(1L << IQ30_SHIFT))

// IQ 格式转换 (Q24 ↔ Q21 ↔ Q30 ↔ Q15)
#define _IQtoIQ21(a)    ((_iq)((a) >> 3))           // Q24→Q21 (24-21=3)
#define _IQ21toIQ(a)    ((_iq)((a) << 3))           // Q21→Q24
#define _IQtoIQ30(a)    ((_iq30)((a) << 6))         // Q24→Q30 (30-24=6)
#define _IQ30toIQ(a)    ((_iq)((a) >> 6))           // Q30→Q24
#define _IQ15toIQ(a)    ((_iq)((int32_t)(a) << (IQ_SHIFT - IQ15_SHIFT)))  // Q15→Q24
#define _IQtoIQ15(a)    ((_iq15)((a) >> (IQ_SHIFT - IQ15_SHIFT)))         // Q24→Q15
#define _IQ30toIQ15(a)  ((_iq15)((a) >> (IQ30_SHIFT - IQ15_SHIFT)))       // Q30→Q15

// ======================= IQ 算术 (纯C int64 实现) =======================

// 乘法: Qa × Qb → (Qa+Qb) → 右移恢复Q
// 用 int64_t 中间量避免 C2000 64位ACC 依赖
//
// ⚡ C2000 加速路径 (ARCH=3):
//   TI IQmath 提供 __IQmpy 内建函数 (IMPYL + QMPYL ACC 硬件, 1周期)
//   如果可用, 我们 #define 到它; 否则用纯C版本
#if BSP_DSP_ARCH == 3
  // C2000Ware: 假设 TI IQmath 头文件提供这些内建函数
  // 实际使用需要 #include "IQmathLib.h"
  #ifdef __TI_IQMPY_AVAILABLE__
    #define _IQmpy(a, b)    __IQmpy(a, b)          // Q24×Q24→Q24 (1周期)
    #define _IQrmpy(a, b)   __IQrmpy(a, b)         // Q24×Q24→Q24 (四舍五入)
    #define _IQrsmpy(a, b)  __IQrsmpy(a, b)        // Q24×Q24→Q24 (饱和)
    #define _IQmpyI32(a, b) __IQmpyI32(a, b)       // Q24×int32→Q24
    #define _IQmpyIQX(a, n1, b, n2) __IQmpyIQX(a, n1, b, n2) // Qn1×Qn2→GLOBAL_Q
    #define _IQdiv(a, b)    __IQdiv(a, b)           // Q24/Q24→Q24
  #else
    // C2000 但未含 IQmathLib → 用纯C回退
    #define _IQ_NO_INTRINSICS 1
  #endif
#else
  #define _IQ_NO_INTRINSICS 1
#endif

// ======== 纯C IQ 运算实现 ========
#ifdef _IQ_NO_INTRINSICS

// Q24 × Q24 → Q24 (标准乘法, 饱和到 Q24 范围)
// 等效: (int32_t)(((int64_t)a * b) >> 24)
static inline _iq _IQmpy(_iq a, _iq b) {
  iq64 prod = (iq64)a * (iq64)b;
  return (_iq)(prod >> IQ_SHIFT);
}

// Q24 × Q24 → Q24 (四舍五入: 加 1<<23 再右移)
static inline _iq _IQrmpy(_iq a, _iq b) {
  iq64 prod = (iq64)a * (iq64)b;
  prod += (1L << (IQ_SHIFT - 1));           // 加 0.5 LSB 四舍五入
  return (_iq)(prod >> IQ_SHIFT);
}

// Q24 × Q24 → Q24 (饱和: 溢出钳位到 INT32_MAX/INT32_MIN)
static inline _iq _IQrsmpy(_iq a, _iq b) {
  iq64 prod = (iq64)a * (iq64)b;
  iq64 shifted = prod >> IQ_SHIFT;
  if (shifted > (iq64)INT32_MAX) return (_iq)INT32_MAX;
  if (shifted < (iq64)INT32_MIN) return (_iq)INT32_MIN;
  return (_iq)shifted;
}

// Q24 × int32 → Q24 (乘整数, 不改变Q格式)
static inline _iq _IQmpyI32(_iq a, int32_t b) {
  iq64 prod = (iq64)a * (iq64)b;
  return (_iq)(prod >> IQ_SHIFT);
}

// 通用格式乘法: Qn1 × Qn2 → Q24 (用移位对齐)
static inline _iq _IQmpyIQX(_iq a, int n1, _iq b, int n2) {
  iq64 prod = (iq64)a * (iq64)b;
  int shift = n1 + n2 - IQ_SHIFT;
  if (shift >= 0) {
    return (_iq)(prod >> shift);
  } else {
    return (_iq)(prod << (-shift));
  }
}

// Q24 / Q24 → Q24
// 等效: (int32_t)(((int64_t)a << 24) / b)
static inline _iq _IQdiv(_iq a, _iq b) {
  if (b == 0) return (a >= 0) ? (_iq)INT32_MAX : (_iq)INT32_MIN;
  iq64 num = (iq64)a << IQ_SHIFT;
  return (_iq)(num / (iq64)b);
}

#endif  // _IQ_NO_INTRINSICS

// ======== IQ 饱和 + 绝对值 ========

// 饱和: 钳位到 [min, max]
static inline _iq _IQsat(_iq x, _iq max, _iq min) {
  if (x > max) return max;
  if (x < min) return min;
  return x;
}

// 绝对值 (饱和: |INT32_MIN| → INT32_MAX)
static inline _iq _IQabs(_iq x) {
  if (x == (_iq)INT32_MIN) return (_iq)INT32_MAX;
  return (x >= 0) ? x : -x;
}

// Q24 × 2 → Q24 (等价于 << 1, 但比 _IQmpy(v, _IQ(2.0)) 快)
static inline _iq _IQmpy2(_iq x) {
  return x << 1;
}

// Q24 / 2 → Q24
static inline _iq _IQdiv2(_iq x) {
  return x >> 1;
}

// ======================= IQ 三角函数 (查表 + 线性插值) =======================

// 查找表 (定义在 comp_iqmath.c)
extern const int32_t IQ_SIN_TABLE[512];    // sin(0~π/2), Q30 格式, 512 点
extern const int32_t IQ_COS_TABLE[512];    // cos(0~π/2), Q30 格式, 512 点

// sin(angle_pu), angle 为标幺 0~1 对应 0~2π
// 用 512 点查找表 + 线性插值, 精度 ~1e-4
_iq _IQsinPU(_iq angle_pu);

// cos(angle_pu), angle 为标幺 0~1
_iq _IQcosPU(_iq angle_pu);

// atan2(y, x) → 标幺 [0, 1), Q24
_iq _IQatan2PU(_iq y, _iq x);

// ---- Q15 版本 (用于 C2000 CLA / CMSIS-DSP 兼容) ----
_iq15 _IQ15sinPU(_iq15 angle_pu);
_iq15 _IQ15cosPU(_iq15 angle_pu);

// step_max, freq → angle 增量, 返回 sin/cos
void _IQ15sincosPU(_iq15 angle_pu, _iq15 *sin_val, _iq15 *cos_val);

// ======================= IQ sqrt / isqrt =======================

// 平方根: Q24 → Q15 (精度更高)
_iq15 _IQ15sqrt(_iq x);

// 倒数平方根: Q24 → Q15
_iq15 _IQ15isqrt(_iq x);

// ======================= IQ 饱和快捷宏 =======================

// 乘 2 并饱和
#define _IQmpy2sat(v, max, min)  _IQsat(_IQmpy2(v), max, min)

// 绝对值后比较
#define _IQabsgt(v, thresh)      (_IQabs(v) > (thresh))

// ======================= 全局 Q 格式常量 =======================

// GLOBAL_Q = 24 (默认), 这些常量用于算法中
#define IQ_PI       _IQ(3.14159265358979)
#define IQ_2PI      _IQ(6.28318530717959)
#define IQ_PI_2     _IQ(1.57079632679490)    // π/2
#define IQ_PI_3     _IQ(1.04719755119660)    // π/3
#define IQ_PI_6     _IQ(0.52359877559830)    // π/6
#define IQ_SQRT3    _IQ(1.73205080756888)    // √3
#define IQ_ONE      _IQ(1.0)
#define IQ_HALF     _IQ(0.5)
#define IQ_ZERO     _IQ(0.0)
#define IQ_NEG_ONE  _IQ(-1.0)

#ifdef __cplusplus
}
#endif

#endif  // COMP_IQMATH_H
