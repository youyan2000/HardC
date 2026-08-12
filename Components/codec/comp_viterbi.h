// Viterbi 卷积码解码器 — K=7 R=1/2 硬判决+软判决, 滑动窗口回溯
//
// 来源: TI controlSUITE vcu/Viterbi (vcu_viterbi.h)
// 参考: Phil Karn's KA9Q Viterbi (industry-standard embedded C implementation)
//
// 编码器: 约束长度 K=7, 码率 R=1/2, 生成多项式 G0=0x6D(133₈) G1=0x4F(171₈)
//   — NASA/CCSDS 标准卷积码, 用于卫星通信 + Qi 无线充电 ASK/FSK 链路
//
// 维特比算法: ACS (Add-Compare-Select) 蝶形运算 + 滑动窗口回溯
//   每个状态有 2 条入边 (input=0 和 input=1), 选择路径度量更小的那条
//   幸存路径存储为 LSB of previous state (允许反向回溯)
//   归一化: 每 RENORM_INTERVAL 步减去最小度量 (防 uint16_t 溢出)
//
// 使用方式:
//   ViterbiState vt;
//   viterbi_init(&vt);
//   // 每收到一对编码符号 (ASK/FSK 解调后):
//   viterbi_update_hard(&vt, rx_bit_a, rx_bit_b);
//   // 或软判决:
//   viterbi_update_soft(&vt, llr_a, llr_b);
//   // 回溯解码:
//   int n = viterbi_traceback(&vt, decoded, sizeof(decoded));

#ifndef COMP_VITERBI_H
#define COMP_VITERBI_H

#include <stdint.h>
#include <string.h>

// 约束长度 K=7 → 状态数 = 2^(K-1) = 64
#define VITERBI_K                 7
#define VITERBI_NUM_STATES        (1 << (VITERBI_K - 1))  // 64
#define VITERBI_TRACEBACK_DEPTH   (5 * VITERBI_K)         // 35
#define VITERBI_RENORM_INTERVAL   64

// ========== 预计算编码器输出表 ==========
// viterbi_output[state][input_bit] — 128 条目
// 索引: index = (state << 1) | input_bit
// 输出: bits[1:0] = { encoder_a, encoder_b }
// 多项式: G0=0x6D(133₈), G1=0x4F(171₈)
// 生成: parity((input_bit << 6 | state) & poly) 对 64×2 组合

static const uint8_t viterbi_output_table[128] = {
  0x0, 0x3, 0x3, 0x0, 0x1, 0x2, 0x2, 0x1, 0x3, 0x0, 0x0, 0x3, 0x2, 0x1, 0x1, 0x2,
  0x3, 0x0, 0x0, 0x3, 0x2, 0x1, 0x1, 0x2, 0x0, 0x3, 0x3, 0x0, 0x1, 0x2, 0x2, 0x1,
  0x0, 0x3, 0x3, 0x0, 0x1, 0x2, 0x2, 0x1, 0x3, 0x0, 0x0, 0x3, 0x2, 0x1, 0x1, 0x2,
  0x3, 0x0, 0x0, 0x3, 0x2, 0x1, 0x1, 0x2, 0x0, 0x3, 0x3, 0x0, 0x1, 0x2, 0x2, 0x1,
  0x2, 0x1, 0x1, 0x2, 0x3, 0x0, 0x0, 0x3, 0x1, 0x2, 0x2, 0x1, 0x0, 0x3, 0x3, 0x0,
  0x1, 0x2, 0x2, 0x1, 0x0, 0x3, 0x3, 0x0, 0x2, 0x1, 0x1, 0x2, 0x3, 0x0, 0x0, 0x3,
  0x2, 0x1, 0x1, 0x2, 0x3, 0x0, 0x0, 0x3, 0x1, 0x2, 0x2, 0x1, 0x0, 0x3, 0x3, 0x0,
  0x1, 0x2, 0x2, 0x1, 0x0, 0x3, 0x3, 0x0, 0x2, 0x1, 0x1, 0x2, 0x3, 0x0, 0x0, 0x3
};

// ======================= ViterbiState (维特比解码器运行状态) =======================

typedef struct {
  uint16_t path_metrics[VITERBI_NUM_STATES];                        // 64 状态的路径度量
  uint8_t  traceback[VITERBI_NUM_STATES * VITERBI_TRACEBACK_DEPTH]; // 幸存路径历史: [pos][state]
  int      tb_pos;                                                   // 回溯位置 [0, TRACEBACK_DEPTH-1]
  int      step_count;                                               // 累计步数 (用于归一化, 每64步重置)
  uint32_t total_steps;                                              // 单调递增总步数 (用于回溯深度判断)
} ViterbiState;

// 初始化 — 清零路径度量和回溯矩阵, 状态 0 度量=0, 其余=max
// 在通信链建立前调用, 确保解码器从零状态开始
static inline void viterbi_init(ViterbiState *me) {
  memset(me, 0, sizeof(ViterbiState));
  // 状态 0 度量=0, 其余状态=最大 (确保从零状态开始解码)
  for (int i = 1; i < VITERBI_NUM_STATES; i++) {
    me->path_metrics[i] = 0xFFFF;
  }
}

// ====================== ACS 蝶形运算核心 (内部辅助) ======================

// 硬判决蝶形运算 — 输入 2 个编码 bit (0 或 1), 更新路径度量和幸存路径
// sym_a, sym_b: 接收到的编码符号 (0 或 1)
//
// ACS 过程:
//   对每个当前状态 s (0..63):
//     输入 0 分支: next0 = s >> 1, expected = viterbi_output_table[(s<<1)|0]
//     输入 1 分支: next1 = (s >> 1) | 0x20, expected = viterbi_output_table[(s<<1)|1]
//     分支度量 = Hamming 距离 (2-bit XOR + popcount)
//     新度量 = 旧度量 + 分支度量
//     比较 → 选最小 → 存幸存者 (LSB of prev state)
static inline void viterbi_update_hard(ViterbiState *me, uint8_t sym_a, uint8_t sym_b) {
  uint16_t new_metrics[VITERBI_NUM_STATES];
  uint8_t  new_survivors[VITERBI_NUM_STATES] = {0};
  int pos = me->tb_pos;

  // 将接收符号打包: rx = (sym_a << 1) | sym_b (bits: [a, b])
  uint8_t rx = ((sym_a & 1) << 1) | (sym_b & 1);

  // 初始化新度量为最大
  for (int i = 0; i < VITERBI_NUM_STATES; i++) {
    new_metrics[i] = 0xFFFF;
  }

  // ACS 蝶形运算: 遍历所有 64 个当前状态
  for (int s = 0; s < VITERBI_NUM_STATES; s++) {
    uint16_t old_metric = me->path_metrics[s];
    if (old_metric >= 0xFFF0) continue;  // 死状态跳过, 防溢出

    // --- 输入 bit = 0 ---
    uint8_t out0 = viterbi_output_table[(s << 1) | 0];  // 期望输出
    uint8_t diff = out0 ^ rx;                             // XOR = 不同位
    uint16_t bm0 = (diff >> 1) + (diff & 1);              // popcount of 2-bit XOR = Hamming 距离
    uint16_t new0 = old_metric + bm0;
    int next0 = s >> 1;                                   // 下一状态 (input 0)
    if (new0 < new_metrics[next0]) {
      new_metrics[next0] = new0;
      new_survivors[next0] = s & 1;                       // 存 prev state 的 LSB
    }

    // --- 输入 bit = 1 ---
    uint8_t out1 = viterbi_output_table[(s << 1) | 1];
    diff = out1 ^ rx;
    uint16_t bm1 = (diff >> 1) + (diff & 1);
    uint16_t new1 = old_metric + bm1;
    int next1 = (s >> 1) | 0x20;                          // 下一状态 (input 1)
    if (new1 < new_metrics[next1]) {
      new_metrics[next1] = new1;
      new_survivors[next1] = s & 1;
    }
  }

  // 更新路径度量
  for (int i = 0; i < VITERBI_NUM_STATES; i++) {
    me->path_metrics[i] = new_metrics[i];
  }

  // 存幸存路径 (按列: pos × 64 + state)
  for (int i = 0; i < VITERBI_NUM_STATES; i++) {
    me->traceback[pos * VITERBI_NUM_STATES + i] = new_survivors[i];
  }

  // 推进回溯位置 (循环缓冲)
  me->tb_pos = (pos + 1) % VITERBI_TRACEBACK_DEPTH;
  me->step_count++;
  me->total_steps++;

  // 归一化: 每 RENORM_INTERVAL 步, 所有度量减最小值 (防 uint16_t 溢出)
  if (me->step_count >= VITERBI_RENORM_INTERVAL) {
    me->step_count = 0;
    uint16_t min_metric = 0xFFFF;
    for (int i = 0; i < VITERBI_NUM_STATES; i++) {
      if (me->path_metrics[i] < min_metric) {
        min_metric = me->path_metrics[i];
      }
    }
    if (min_metric > 0) {
      for (int i = 0; i < VITERBI_NUM_STATES; i++) {
        if (me->path_metrics[i] < 0xFFFF) {
          me->path_metrics[i] -= min_metric;
        }
      }
    }
  }
}

// 软判决蝶形运算 — 输入 2 个量化软值 llr ∈ [0, 255]
// llr_a, llr_b: 接收符号软值 (0=strong 0, 255=strong 1)
// 分支度量用 Manhattan 距离 (高效近似, 无乘法):
//   bm = |llr - expected*255| = expected ? (255 - llr) : llr
static inline void viterbi_update_soft(ViterbiState *me, uint8_t llr_a, uint8_t llr_b) {
  uint16_t new_metrics[VITERBI_NUM_STATES];
  uint8_t  new_survivors[VITERBI_NUM_STATES] = {0};
  int pos = me->tb_pos;

  // 初始化新度量为最大
  for (int i = 0; i < VITERBI_NUM_STATES; i++) {
    new_metrics[i] = 0xFFFF;
  }

  // ACS 蝶形运算: 遍历所有 64 个当前状态
  for (int s = 0; s < VITERBI_NUM_STATES; s++) {
    uint16_t old_metric = me->path_metrics[s];
    if (old_metric >= 0xFFF0) continue;

    // --- 输入 bit = 0 ---
    uint8_t out0 = viterbi_output_table[(s << 1) | 0];
    uint16_t bm0;
    {
      uint8_t exp_a = (out0 >> 1) & 1;  // 期望 bit a (0 或 1)
      uint8_t exp_b = out0 & 1;         // 期望 bit b (0 或 1)
      // Manhattan 距离: |llr - expected*255|
      uint16_t da = exp_a ? (uint16_t)(255 - llr_a) : (uint16_t)llr_a;
      uint16_t db = exp_b ? (uint16_t)(255 - llr_b) : (uint16_t)llr_b;
      bm0 = da + db;
    }
    uint16_t new0 = old_metric + bm0;
    int next0 = s >> 1;
    if (new0 < new_metrics[next0]) {
      new_metrics[next0] = new0;
      new_survivors[next0] = s & 1;
    }

    // --- 输入 bit = 1 ---
    uint8_t out1 = viterbi_output_table[(s << 1) | 1];
    uint16_t bm1;
    {
      uint8_t exp_a = (out1 >> 1) & 1;
      uint8_t exp_b = out1 & 1;
      uint16_t da = exp_a ? (uint16_t)(255 - llr_a) : (uint16_t)llr_a;
      uint16_t db = exp_b ? (uint16_t)(255 - llr_b) : (uint16_t)llr_b;
      bm1 = da + db;
    }
    uint16_t new1 = old_metric + bm1;
    int next1 = (s >> 1) | 0x20;
    if (new1 < new_metrics[next1]) {
      new_metrics[next1] = new1;
      new_survivors[next1] = s & 1;
    }
  }

  // 更新路径度量
  for (int i = 0; i < VITERBI_NUM_STATES; i++) {
    me->path_metrics[i] = new_metrics[i];
  }

  // 存幸存路径
  for (int i = 0; i < VITERBI_NUM_STATES; i++) {
    me->traceback[pos * VITERBI_NUM_STATES + i] = new_survivors[i];
  }

  // 推进位置
  me->tb_pos = (pos + 1) % VITERBI_TRACEBACK_DEPTH;
  me->step_count++;
  me->total_steps++;

  // 归一化
  if (me->step_count >= VITERBI_RENORM_INTERVAL) {
    me->step_count = 0;
    uint16_t min_metric = 0xFFFF;
    for (int i = 0; i < VITERBI_NUM_STATES; i++) {
      if (me->path_metrics[i] < min_metric) {
        min_metric = me->path_metrics[i];
      }
    }
    if (min_metric > 0) {
      for (int i = 0; i < VITERBI_NUM_STATES; i++) {
        if (me->path_metrics[i] < 0xFFFF) {
          me->path_metrics[i] -= min_metric;
        }
      }
    }
  }
}

// ====================== 回溯解码 ======================

// 滑动窗口回溯 — 从最小度量状态追溯 TRACEBACK_DEPTH 步, 输出正向解码 bits
//   me:     解码器状态
//   out:    输出缓冲
//   max_len: 输出缓冲最大字节数 (每字节 8 个 bit)
//   返回:   解码出的 bit 数
//
// 回溯原理:
//   state_curr = argmin(path_metrics)
//   往前回溯 i 步:
//     bit = state_curr >> 5               ← MSB 是输入 bit
//     prev = ((state_curr << 1) | survivor[state_curr]) & 0x3F
//     state_curr = prev
//   解码 bits 是逆序的 → 最后翻转输出
static inline int viterbi_traceback(ViterbiState *me, uint8_t *out, int max_len) {
  int max_bits = max_len * 8;

  // 找最小度量状态
  int state = 0;
  uint16_t min_metric = 0xFFFF;
  for (int s = 0; s < VITERBI_NUM_STATES; s++) {
    if (me->path_metrics[s] < min_metric) {
      min_metric = me->path_metrics[s];
      state = s;
    }
  }

  // 从当前位置倒退一步 (tb_pos 指向下一个空闲位置)
  int pos = (me->tb_pos - 1 + VITERBI_TRACEBACK_DEPTH) % VITERBI_TRACEBACK_DEPTH;

  // 收集逆序 bits
  uint8_t bits[VITERBI_TRACEBACK_DEPTH];
  int nbits = 0;

  for (int i = 0; i < VITERBI_TRACEBACK_DEPTH && nbits < VITERBI_TRACEBACK_DEPTH; i++) {
    uint8_t bit = (state >> 5) & 1;  // 解码 bit = MSB of current state
    bits[nbits++] = bit;

    // 重建前一状态: prev = ((state << 1) | survivor_bit) & 0x3F
    uint8_t survivor = me->traceback[pos * VITERBI_NUM_STATES + state];
    state = ((state << 1) | survivor) & (VITERBI_NUM_STATES - 1);

    // 倒退到上一时间步
    pos = (pos - 1 + VITERBI_TRACEBACK_DEPTH) % VITERBI_TRACEBACK_DEPTH;
  }

  // 翻转 bits 到正向, 写入输出缓冲
  int out_bits = (nbits < max_bits) ? nbits : max_bits;
  for (int i = 0; i < out_bits; i++) {
    int byte_idx = i / 8;
    int bit_idx = i % 8;
    uint8_t bit_val = bits[nbits - 1 - i];  // 逆序读取 = 正向
    if (bit_idx == 0) {
      out[byte_idx] = 0;
    }
    out[byte_idx] |= (bit_val << bit_idx);
  }

  return out_bits;
}

// 简化版 — 解码单个 bit (返回最近解码的 bit, -1 表示回溯深度不足)
static inline int viterbi_decode_bit(ViterbiState *me) {
  if (me->total_steps < VITERBI_TRACEBACK_DEPTH) return -1;
  uint8_t bit;
  if (viterbi_traceback(me, &bit, 1) >= 1) {
    return bit & 1;
  }
  return -1;
}

// 重置 — 重新初始化 (用于链路重建)
static inline void viterbi_reset(ViterbiState *me) {
  viterbi_init(me);
}

#endif  // COMP_VITERBI_H
