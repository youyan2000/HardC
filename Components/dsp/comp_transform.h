// 坐标变换库 — Clarke / Park / iPark
//
// 来源: TI controlSUITE motor_control/math_blocks (clarke.h, park.h, ipark.h)
// 翻译为 HardC 纯C float inline 版本 (去掉 _iq 定点依赖)
//
// 三后端策略:
//   默认 float — 直接浮点乘法 (适用于 M4F/M7/FPU C2000)
//   将来可扩展 IQ / CLA 后端 (通过 BSP/bsp_dsp.h 平台检测)
//
// static inline 函数适合 ISR 热路径, 零调用开销

#ifndef COMP_TRANSFORM_H
#define COMP_TRANSFORM_H

// 1/sqrt(3) = 0.57735026918963 (Clarke 变换常数)
#ifndef ONE_OVER_SQRT3
#define ONE_OVER_SQRT3  0.57735026918963f
#endif

// sqrt(3)/2 = 0.86602540378444 (SVPWM 扇区判断常数)
#ifndef SQRT3_OVER_2
#define SQRT3_OVER_2     0.86602540378444f
#endif

// 2/sqrt(3) = 1.15470053837925 (Clarke 幅值不变形式常数)
#ifndef TWO_OVER_SQRT3
#define TWO_OVER_SQRT3   1.15470053837925f
#endif

// ======================== Clarke 变换 (ABC → αβ) ========================

// Clarke 变换结构体 — 三相静止 → 两相静止
typedef struct {
  float as;       // 输入: A 相
  float bs;       // 输入: B 相
  float cs;       // 输入: C 相 (三电流形式), 不使用则为 0
  float alpha;    // 输出: α 轴 (与 A 相同轴)
  float beta;     // 输出: β 轴 (超前 α 90°)
} Clarke;

// 默认初始化
#define CLARKE_DEFAULTS { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }

// Clarke 变换 — 双电流形式 (假定 ia+ib+ic=0)
// Alpha = ia,  Beta = (ia + 2*ib) / sqrt(3)
static inline void clarke_run_2curr(Clarke *v) {
  v->alpha = v->as;
  v->beta  = (v->as + 2.0f * v->bs) * ONE_OVER_SQRT3;
}

// Clarke 变换 — 三电流形式 (iα = ia, iβ = (ib - ic) / sqrt(3))
static inline void clarke_run_3curr(Clarke *v) {
  v->alpha = v->as;
  v->beta  = (v->bs - v->cs) * ONE_OVER_SQRT3;
}

// ======================== Park 变换 (αβ → dq) ========================

// Park 变换结构体 — 两相静止 → 两相旋转
// 使用者先算 sin(θ) / cos(θ) 填入, 再调 park_run
typedef struct {
  float alpha;    // 输入: α 轴分量
  float beta;     // 输入: β 轴分量
  float sine;     // 输入: sin(θ), 旋转角正弦
  float cosine;   // 输入: cos(θ), 旋转角余弦
  float ds;       // 输出: d 轴分量 (与旋转磁场同相)
  float qs;       // 输出: q 轴分量 (超前 d 轴 90°)
} Park;

#define PARK_DEFAULTS { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }

// Park 变换: Ds = α·cosθ + β·sinθ,  Qs = β·cosθ - α·sinθ
static inline void park_run(Park *v) {
  v->ds = v->alpha * v->cosine + v->beta * v->sine;
  v->qs = v->beta  * v->cosine - v->alpha * v->sine;
}

// ====================== 反Park 变换 (dq → αβ) ======================

// 反Park 变换结构体 — 两相旋转 → 两相静止
typedef struct {
  float ds;       // 输入: d 轴参考电压
  float qs;       // 输入: q 轴参考电压
  float sine;     // 输入: sin(θ)
  float cosine;   // 输入: cos(θ)
  float alpha;    // 输出: α 轴参考电压
  float beta;     // 输出: β 轴参考电压
} ParkInv;

#define PARK_INV_DEFAULTS { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }

// 反Park 变换: α = Ds·cosθ - Qs·sinθ,  β = Qs·cosθ + Ds·sinθ
static inline void park_inv_run(ParkInv *v) {
  v->alpha = v->ds * v->cosine - v->qs * v->sine;
  v->beta  = v->qs * v->cosine + v->ds * v->sine;
}

#endif  // COMP_TRANSFORM_H
