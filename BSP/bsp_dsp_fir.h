// BSP DSP 扩展 — FIR 滤波器硬件加速抽象
//
// 接续 BSP/bsp_dsp.h (§5), 添加 FIR 滤波器平台抽象:
//   fir_f32 — 单级 FIR 滤波 (CMSIS-DSP arm_fir_f32 / C2000 FIR_FP / 纯C)
//
// 使用方式:
//   #include "bsp_dsp.h"
//   #include "bsp_dsp_fir.h"
//   bsp_fir_init(&fir, coeffs, state, num_taps);
//   float y = bsp_fir_apply(&fir, x);

#ifndef BSP_DSP_FIR_H
#define BSP_DSP_FIR_H

// Q15 架构标识 — 手动覆盖 BSP_DSP_ARCH (需在 #include "bsp_dsp.h" 之前 #define BSP_DSP_ARCH BSP_DSP_ARCH_Q15)
#define BSP_DSP_ARCH_Q15  4

#include "bsp_dsp.h"

// ======== FIR 实例结构体 ========

// 最大 FIR 阶数 (纯C回退用)
#define BSP_FIR_MAX_TAPS  128

typedef struct {
  float *coeffs;          // FIR 系数数组 (长度 = num_taps)
  float *state;           // 延迟线 (长度 = num_taps + block_size - 1)
  int   num_taps;         // 抽头数 (滤波器阶数 + 1)

#if BSP_DSP_ARCH == 2 || BSP_DSP_ARCH == 1
  arm_fir_instance_f32  cmsis_inst;   // CMSIS-DSP 实例
  float coeffs_buf[BSP_FIR_MAX_TAPS];  // 系数副本 (CMSIS-DSP 要求)
  float state_buf[BSP_FIR_MAX_TAPS * 2]; // 延迟线缓冲
#elif BSP_DSP_ARCH == BSP_DSP_ARCH_Q15
  // FixedPointLib Q15 FIR16 路径
  int16_t  coeffs_q15[BSP_FIR_MAX_TAPS];   // 系数副本 Q15
  int16_t  state_q15[BSP_FIR_MAX_TAPS];    // 延迟线 (循环缓冲)
  uint16_t delay_idx;                       // 循环缓冲写入索引
  uint8_t  post_shift;                      // 后缩放右移位数
#else
  // 纯C/C2000: 直接使用 coeffs/state 指针
  float coeffs_buf[BSP_FIR_MAX_TAPS];
  float state_buf[BSP_FIR_MAX_TAPS];
#endif
} BspFirInst;

// ======== FIR 初始化 ========

static inline void bsp_fir_init(BspFirInst *me, const float *coeffs,
                                 int num_taps) {
  me->num_taps = num_taps;

#if BSP_DSP_ARCH == 2 || BSP_DSP_ARCH == 1
  // CMSIS-DSP: 复制系数到内部缓冲
  for (int i = 0; i < num_taps && i < BSP_FIR_MAX_TAPS; i++) {
    me->coeffs_buf[i] = coeffs[i];
  }
  for (int i = 0; i < num_taps * 2 && i < BSP_FIR_MAX_TAPS * 2; i++) {
    me->state_buf[i] = 0.0f;
  }
  arm_fir_init_f32(&me->cmsis_inst, (uint16_t)num_taps,
                   me->coeffs_buf, me->state_buf, 1);
  me->coeffs = me->coeffs_buf;
  me->state = me->state_buf;
#elif BSP_DSP_ARCH == BSP_DSP_ARCH_Q15
  // FixedPointLib Q15: 系数以 float* 传入 (实际为 int16_t[] Q15), 延迟线清零
  {
    const int16_t *q15c = (const int16_t *)coeffs;
    for (int i = 0; i < num_taps && i < BSP_FIR_MAX_TAPS; i++) {
      me->coeffs_q15[i] = q15c[i];
      me->state_q15[i] = 0;
    }
    me->delay_idx = 0;
    me->post_shift = 0;   // 默认不移位, 用户根据系数缩放调整
  }
#else
  // 纯C/C2000: 拷贝系数, 清零延迟线
  for (int i = 0; i < num_taps && i < BSP_FIR_MAX_TAPS; i++) {
    me->coeffs_buf[i] = coeffs[i];
    me->state_buf[i] = 0.0f;
  }
  me->coeffs = me->coeffs_buf;
  me->state = me->state_buf;
#endif
}

// ======== FIR 单步滤波 ========

// 输入 x, 输出 y
static inline float bsp_fir_apply(BspFirInst *me, float x) {
  float y;

#if BSP_DSP_ARCH == 2 || BSP_DSP_ARCH == 1
  arm_fir_f32(&me->cmsis_inst, &x, &y, 1);
#elif BSP_DSP_ARCH == BSP_DSP_ARCH_Q15
  // FixedPointLib Q15 FIR16: 循环缓冲 + 乘累加 + 后缩放
  {
    int n = me->num_taps;
    int16_t *c = me->coeffs_q15;
    int16_t *s = me->state_q15;

    // float→Q15 截断输入
    int16_t x_q15 = (int16_t)(x * 32767.0f);

    // 循环缓冲写入
    s[me->delay_idx] = x_q15;
    me->delay_idx = (me->delay_idx + 1) % n;

    // 卷积: 从最旧样本开始乘累加 (int64 防 128 抽头溢出)
    int64_t acc = 0;
    uint16_t idx = me->delay_idx;
    for (int i = 0; i < n; i++) {
      acc += (int64_t)c[i] * (int64_t)s[idx];
      idx = (idx + 1) % n;
    }

    // 后缩放 + 饱和
    int64_t y_q15 = acc >> me->post_shift;
    if (y_q15 > 32767)  { y_q15 = 32767; }
    if (y_q15 < -32768) { y_q15 = -32768; }

    y = (float)y_q15 / 32768.0f;  // Q15→float 返回
  }
#elif BSP_DSP_ARCH == 3
  // C2000: 使用 FPU FIR (如果有 DSP.h)
  // 简化回退: 纯C实现
  {
    int n = me->num_taps;
    float *c = me->coeffs;
    float *s = me->state;

    // 移位延迟线
    for (int i = n - 1; i > 0; i--) {
      s[i] = s[i - 1];
    }
    s[0] = x;

    // 卷积
    y = 0.0f;
    for (int i = 0; i < n; i++) {
      y += c[i] * s[i];
    }
  }
#else
  // 纯C: 移位延迟线 + 卷积
  {
    int n = me->num_taps;
    float *c = me->coeffs;
    float *s = me->state;

    for (int i = n - 1; i > 0; i--) {
      s[i] = s[i - 1];
    }
    s[0] = x;

    y = 0.0f;
    for (int i = 0; i < n; i++) {
      y += c[i] * s[i];
    }
  }
#endif

  return y;
}

// ======== FIR Block 滤波 ========

static inline void bsp_fir_apply_block(BspFirInst *me, const float *src,
                                        float *dst, int block_size) {
  // 统一走逐样本路径: CMSIS-DSP 在 init 时 blockSize=1,
  // arm_fir_f32 必须与 init 的 blockSize 匹配, 否则结果错误
  for (int i = 0; i < block_size; i++) {
    dst[i] = bsp_fir_apply(me, src[i]);
  }
}

// ======== Q15 定点 FIR (v1.2 扩展 — FixedPointLib) ========

// 来源: TI controlSUITE FixedPointLib/v1_20/fir.h
//
// Q15 定点 FIR, 适用于 C2000 或需要 Q15 格式的平台
// 系数缩放: 1.0 = 0x7FFF (32767)
// 乘法累加后右移归一化, 输出饱和到 [-32768, 32767]
//
// 使用方式 A — 统一 BspFirInst 接口 (float 入/出, 内部 Q15):
//   #define BSP_DSP_ARCH  BSP_DSP_ARCH_Q15   // 在 #include "bsp_dsp.h" 之前
//   #include "bsp_dsp.h"
//   #include "bsp_dsp_fir.h"
//   bsp_fir_init(&fir, (const float *)q15_coeffs, num_taps);
//   float y = bsp_fir_apply(&fir, x);          // float→Q15→滤波→float
//
// 使用方式 B — 独立 Q15 接口 (零 float 转换, ISR 高效):
//   #include "bsp_dsp_fir.h"
//   BspFirQ15Cfg cfg = { coeffs, taps, shift };
//   int16_t buf[taps];
//   BspFirQ15State st = { buf, 0 };
//   bsp_fir_q15_init(&st, &cfg);
//   int16_t y = bsp_fir_q15_apply(&st, &cfg, x);  // 纯 Q15, 零 float 开销

// Q15 FIR 系数配置 (可共享, 只读)
typedef struct {
  const int16_t *coeffs;  // FIR 系数数组 Q15 (长度 = taps)
  uint16_t       taps;    // 抽头数
  uint8_t        shift;   // 后缩放右移位数 (0-15)
} BspFirQ15Cfg;

// Q15 FIR 延迟线状态 (每实例独立 — 调用者提供 buffer 存储)
typedef struct {
  int16_t  *buffer;  // 延迟线缓冲 (长度 = taps, 调用者分配)
  uint16_t  index;   // 循环缓冲写入位置
} BspFirQ15State;

// 初始化 Q15 FIR 状态 — 清零延迟线, 重置循环索引
//   cfg:  FIR 配置 (系数 + 抽头数 + 移位数)
static inline void bsp_fir_q15_init(BspFirQ15State *me, const BspFirQ15Cfg *cfg) {
  me->index = 0;
  for (uint16_t i = 0; i < cfg->taps; i++) {
    me->buffer[i] = 0;
  }
}

// Q15 FIR 单步滤波 — 循环缓冲 + 乘累加 + 后缩放 + 饱和
//   cfg:  FIR 配置
//   x:    当前输入 Q15
//   返回: 滤波输出 Q15, 饱和在 [-32768, 32767]
//   int64 累加器保证 128 抽头乘积累加不溢出
static inline int16_t bsp_fir_q15_apply(BspFirQ15State *me, const BspFirQ15Cfg *cfg,
                                         int16_t x) {
  // 循环缓冲写入
  me->buffer[me->index] = x;
  me->index = (me->index + 1) % cfg->taps;

  // 卷积: 从最旧样本开始乘累加
  int64_t acc = 0;
  uint16_t idx = me->index;
  for (uint16_t i = 0; i < cfg->taps; i++) {
    acc += (int64_t)cfg->coeffs[i] * (int64_t)me->buffer[idx];
    idx = (idx + 1) % cfg->taps;
  }

  // 后缩放 + 饱和
  int64_t y = acc >> cfg->shift;
  if (y > 32767)  { y = 32767; }
  if (y < -32768) { y = -32768; }
  return (int16_t)y;
}

#endif  // BSP_DSP_FIR_H
