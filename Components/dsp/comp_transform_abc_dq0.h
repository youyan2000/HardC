// 坐标变换库扩展 — abc ↔ dq0 正序/负序组合变换
//
// 来源: TI controlSUITE solar/v1.2/float (ABC_DQ0_POS_F, ABC_DQ0_NEG_F,
//   DQ0_ABC_F — power_invariant 三相 abc ↔ dq0 变换, 含零序)
// 翻译为 C-OOP 纯C float inline 版本
//
// 变换矩阵 (幅值不变形式):
//   dq0(+) = T(θ) · abc,   其中 T(θ) = 2/3 *
//     [  cosθ     cos(θ-120°)   cos(θ+120°) ]
//     [ -sinθ    -sin(θ-120°)  -sin(θ+120°) ]
//     [  1/2        1/2           1/2        ]
//
// 逆变换:
//   abc = T⁻¹(θ) · dq0,   其中 T⁻¹(θ) =
//     [  cosθ    -sinθ    1 ]
//     [ cos(θ-120°)  -sin(θ-120°)  1 ]
//     [ cos(θ+120°)  -sin(θ+120°)  1 ]

#ifndef COMP_TRANSFORM_ABC_DQ0_H
#define COMP_TRANSFORM_ABC_DQ0_H

#include <math.h>

// 120° 弧度常数
#ifndef RAD_120
#define RAD_120  2.094395102f    // 2π/3
#endif

// ======================== AbcDq0Pos (abc → dq0 正序) ========================

// 三相静止 → 旋转正序变换 (幅值不变形式)
// 正序: 逆时针旋转 (a→b→c)
typedef struct {
  float a;                // 输入: A 相
  float b;                // 输入: B 相
  float c;                // 输入: C 相
  float sin;              // 输入: sin(θ), 正序角
  float cos;              // 输入: cos(θ)
  float d;                // 输出: d 轴 (有功分量)
  float q;                // 输出: q 轴 (无功分量)
  float z;                // 输出: 零序分量 (a+b+c)/3
} AbcDq0Pos;

#define ABC_DQ0_POS_DEFAULTS { 0, 0, 0, 0, 1, 0, 0, 0 }

// abc → dq0 正序变换
// D = 2/3 (a*cosθ + b*cos(θ-120) + c*cos(θ+120))
// Q = 2/3 (a*sinθ + b*sin(θ-120) + c*sin(θ+120))
// Z = (a + b + c) / 3
static inline void abc_dq0_pos_run(AbcDq0Pos *me) {
  float s120 = sinf(RAD_120);   // sin(120°) = √3/2
  float c120 = cosf(RAD_120);   // cos(120°) = -1/2

  // cos(θ-120°) = cosθ·cos120 + sinθ·sin120 = cosθ·(-1/2) + sinθ·(√3/2)
  float cos_m120 = me->cos * c120 + me->sin * s120;
  // sin(θ-120°) = sinθ·cos120 - cosθ·sin120 = sinθ·(-1/2) - cosθ·(√3/2)
  float sin_m120 = me->sin * c120 - me->cos * s120;

  // cos(θ+120°) = cosθ·cos120 - sinθ·sin120 = cosθ·(-1/2) - sinθ·(√3/2)
  float cos_p120 = me->cos * c120 - me->sin * s120;
  // sin(θ+120°) = sinθ·cos120 + cosθ·sin120 = sinθ·(-1/2) + cosθ·(√3/2)
  float sin_p120 = me->sin * c120 + me->cos * s120;

  me->d = (2.0f / 3.0f) * (me->a * me->cos + me->b * cos_m120 + me->c * cos_p120);
  me->q = (2.0f / 3.0f) * (me->a * me->sin + me->b * sin_m120 + me->c * sin_p120);
  me->z = (me->a + me->b + me->c) / 3.0f;
}

// ======================== AbcDq0Neg (abc → dq0 负序) ========================

// 三相静止 → 旋转负序变换 (幅值不变形式)
// 负序: 顺时针旋转 (a→c→b), 用 -θ 代入正序公式
typedef struct {
  float a, b, c;          // 输入
  float sin;              // 输入: sin(θ)
  float cos;              // 输入: cos(θ)
  float d;                // 输出: d 轴 (负序有功分量)
  float q;                // 输出: q 轴 (负序无功分量)
  float z;                // 输出: 零序分量
} AbcDq0Neg;

#define ABC_DQ0_NEG_DEFAULTS { 0, 0, 0, 0, 1, 0, 0, 0 }

// abc → dq0 负序变换
// 用 -θ: cos(-θ)=cosθ, sin(-θ)=-sinθ, cos(-θ±120)=cos(θ∓120), sin(-θ±120)=-sin(θ∓120)
static inline void abc_dq0_neg_run(AbcDq0Neg *me) {
  float s120 = sqrtf(3.0f) * 0.5f;  // √3/2
  float c120 = -0.5f;               // cos(120°)

  // θ-120° 的 sin/cos
  float cos_m120 = me->cos * c120 + me->sin * s120;
  float sin_m120 = me->sin * c120 - me->cos * s120;

  // θ+120° 的 sin/cos
  float cos_p120 = me->cos * c120 - me->sin * s120;
  float sin_p120 = me->sin * c120 + me->cos * s120;

  // 负序: sin → -sin (相位反转)
  me->d = (2.0f / 3.0f) * (me->a * me->cos + me->b * cos_p120 + me->c * cos_m120);
  me->q = (2.0f / 3.0f) * (me->a * (-me->sin) + me->b * (-sin_p120) + me->c * (-sin_m120));
  me->z = (me->a + me->b + me->c) / 3.0f;
}

// ======================== Dq0Abc (dq0 → abc 逆变换) ========================

// dq0 逆变换 → abc (用于电压参考从 dq 转回三相 PWM 比较值)
typedef struct {
  float d;                // 输入: d 轴参考
  float q;                // 输入: q 轴参考
  float z;                // 输入: 零序分量
  float sin;              // 输入: sin(θ)
  float cos;              // 输入: cos(θ)
  float a;                // 输出: A 相
  float b;                // 输出: B 相
  float c;                // 输出: C 相
} Dq0Abc;

#define DQ0_ABC_DEFAULTS { 0, 0, 0, 0, 1, 0, 0, 0 }

// dq0 → abc 逆变换
// a = d*cosθ - q*sinθ + z
// b = d*cos(θ-120°) - q*sin(θ-120°) + z
// c = d*cos(θ+120°) - q*sin(θ+120°) + z
static inline void dq0_abc_run(Dq0Abc *me) {
  float s120 = sqrtf(3.0f) * 0.5f;
  float c120 = -0.5f;

  float cos_m120 = me->cos * c120 + me->sin * s120;
  float sin_m120 = me->sin * c120 - me->cos * s120;
  float cos_p120 = me->cos * c120 - me->sin * s120;
  float sin_p120 = me->sin * c120 + me->cos * s120;

  me->a = me->d * me->cos  - me->q * me->sin      + me->z;
  me->b = me->d * cos_m120 - me->q * sin_m120     + me->z;
  me->c = me->d * cos_p120 - me->q * sin_p120     + me->z;
}

#endif  // COMP_TRANSFORM_ABC_DQ0_H
