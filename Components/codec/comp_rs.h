// Reed-Solomon 编解码器 — RS(N=255, K=239, T=8), 系统码, Berlekamp-Massey 译码
//
// 来源: TI controlSUITE VCU/v2_00/reedsolomon_encoder.h + vcu2_reedsolomon_decoder.h
//
// RS(N,K) 块纠错码, N=255 字节码字, K=255-2T 字节消息, 可纠正 T 字节错误
// 工作在 GF(2^8), 本原多项式: x^8 + x^4 + x^3 + x^2 + 1 (0x11D)
// 系统编码: 消息字节在前, 2T 字节校验在后
// Berlekamp-Massey 迭代 → Chien 搜索 → Forney 算法 (全查表, ISR 安全)
//
// 应用场景: DVB 数字电视, 电力线通信, Qi 无线充电, 深空通信
//
// 使用方式:
//   RsCfg cfg;
//   rs_init(&cfg, 8);  // RS(255, 239) — 最多纠正 8 字节错误
//   uint8_t codeword[255];
//   rs_encode(&cfg, msg_239_bytes, codeword);
//   // ... 传输 ...
//   RsState st;
//   int nerr = rs_decode(&st, &cfg, received_255_bytes, corrected_255_bytes);
//   if (nerr < 0) { /* 不可纠正 */ }

#ifndef COMP_RS_H
#define COMP_RS_H

#include <stdint.h>
#include <string.h>

// 最大可配置参数: T ∈ [1, 16], 码长 N = 255 固定
#define RS_MAX_T  16
#define RS_N      255
#define RS_MAX_2T (2 * RS_MAX_T)       // 32 — 最大校验字节数
#define RS_MAX_POLY (RS_MAX_2T + 1)    // 33 — 最大多项式系数数

// ====================== GF(2^8) exp/log 查找表 ======================
// 本原多项式 0x11D = x^8 + x^4 + x^3 + x^2 + 1
// exp[i] = α^i  (i = 0..255, α^255 = α^0 = 1)
// exp 表扩展为 512 条目, 乘法时无需取模: gf_mul(a,b) = exp[log[a] + log[b]]
// 生成算法:
//   α = 1; for i in 0..254: exp[i]=α; log[α]=i; α=α<<1; if α&0x100: α^=0x11D;
//   exp[255]=1; for i in 0..255: exp[256+i]=exp[i]

static const uint8_t gf256_exp[512] = {
  0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1D, 0x3A, 0x74, 0xE8, 0xCD, 0x87, 0x13, 0x26,
  0x4C, 0x98, 0x2D, 0x5A, 0xB4, 0x75, 0xEA, 0xC9, 0x8F, 0x03, 0x06, 0x0C, 0x18, 0x30, 0x60, 0xC0,
  0x9D, 0x27, 0x4E, 0x9C, 0x25, 0x4A, 0x94, 0x35, 0x6A, 0xD4, 0xB5, 0x77, 0xEE, 0xC1, 0x9F, 0x23,
  0x46, 0x8C, 0x05, 0x0A, 0x14, 0x28, 0x50, 0xA0, 0x5D, 0xBA, 0x69, 0xD2, 0xB9, 0x6F, 0xDE, 0xA1,
  0x5F, 0xBE, 0x61, 0xC2, 0x99, 0x2F, 0x5E, 0xBC, 0x65, 0xCA, 0x89, 0x0F, 0x1E, 0x3C, 0x78, 0xF0,
  0xFD, 0xE7, 0xD3, 0xBB, 0x6B, 0xD6, 0xB1, 0x7F, 0xFE, 0xE1, 0xDF, 0xA3, 0x5B, 0xB6, 0x71, 0xE2,
  0xD9, 0xAF, 0x43, 0x86, 0x11, 0x22, 0x44, 0x88, 0x0D, 0x1A, 0x34, 0x68, 0xD0, 0xBD, 0x67, 0xCE,
  0x81, 0x1F, 0x3E, 0x7C, 0xF8, 0xED, 0xC7, 0x93, 0x3B, 0x76, 0xEC, 0xC5, 0x97, 0x33, 0x66, 0xCC,
  0x85, 0x17, 0x2E, 0x5C, 0xB8, 0x6D, 0xDA, 0xA9, 0x4F, 0x9E, 0x21, 0x42, 0x84, 0x15, 0x2A, 0x54,
  0xA8, 0x4D, 0x9A, 0x29, 0x52, 0xA4, 0x55, 0xAA, 0x49, 0x92, 0x39, 0x72, 0xE4, 0xD5, 0xB7, 0x73,
  0xE6, 0xD1, 0xBF, 0x63, 0xC6, 0x91, 0x3F, 0x7E, 0xFC, 0xE5, 0xD7, 0xB3, 0x7B, 0xF6, 0xF1, 0xFF,
  0xE3, 0xDB, 0xAB, 0x4B, 0x96, 0x31, 0x62, 0xC4, 0x95, 0x37, 0x6E, 0xDC, 0xA5, 0x57, 0xAE, 0x41,
  0x82, 0x19, 0x32, 0x64, 0xC8, 0x8D, 0x07, 0x0E, 0x1C, 0x38, 0x70, 0xE0, 0xDD, 0xA7, 0x53, 0xA6,
  0x51, 0xA2, 0x59, 0xB2, 0x79, 0xF2, 0xF9, 0xEF, 0xC3, 0x9B, 0x2B, 0x56, 0xAC, 0x45, 0x8A, 0x09,
  0x12, 0x24, 0x48, 0x90, 0x3D, 0x7A, 0xF4, 0xF5, 0xF7, 0xF3, 0xFB, 0xEB, 0xCB, 0x8B, 0x0B, 0x16,
  0x2C, 0x58, 0xB0, 0x7D, 0xFA, 0xE9, 0xCF, 0x83, 0x1B, 0x36, 0x6C, 0xD8, 0xAD, 0x47, 0x8E, 0x01,
  0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1D, 0x3A, 0x74, 0xE8, 0xCD, 0x87, 0x13, 0x26,
  0x4C, 0x98, 0x2D, 0x5A, 0xB4, 0x75, 0xEA, 0xC9, 0x8F, 0x03, 0x06, 0x0C, 0x18, 0x30, 0x60, 0xC0,
  0x9D, 0x27, 0x4E, 0x9C, 0x25, 0x4A, 0x94, 0x35, 0x6A, 0xD4, 0xB5, 0x77, 0xEE, 0xC1, 0x9F, 0x23,
  0x46, 0x8C, 0x05, 0x0A, 0x14, 0x28, 0x50, 0xA0, 0x5D, 0xBA, 0x69, 0xD2, 0xB9, 0x6F, 0xDE, 0xA1,
  0x5F, 0xBE, 0x61, 0xC2, 0x99, 0x2F, 0x5E, 0xBC, 0x65, 0xCA, 0x89, 0x0F, 0x1E, 0x3C, 0x78, 0xF0,
  0xFD, 0xE7, 0xD3, 0xBB, 0x6B, 0xD6, 0xB1, 0x7F, 0xFE, 0xE1, 0xDF, 0xA3, 0x5B, 0xB6, 0x71, 0xE2,
  0xD9, 0xAF, 0x43, 0x86, 0x11, 0x22, 0x44, 0x88, 0x0D, 0x1A, 0x34, 0x68, 0xD0, 0xBD, 0x67, 0xCE,
  0x81, 0x1F, 0x3E, 0x7C, 0xF8, 0xED, 0xC7, 0x93, 0x3B, 0x76, 0xEC, 0xC5, 0x97, 0x33, 0x66, 0xCC,
  0x85, 0x17, 0x2E, 0x5C, 0xB8, 0x6D, 0xDA, 0xA9, 0x4F, 0x9E, 0x21, 0x42, 0x84, 0x15, 0x2A, 0x54,
  0xA8, 0x4D, 0x9A, 0x29, 0x52, 0xA4, 0x55, 0xAA, 0x49, 0x92, 0x39, 0x72, 0xE4, 0xD5, 0xB7, 0x73,
  0xE6, 0xD1, 0xBF, 0x63, 0xC6, 0x91, 0x3F, 0x7E, 0xFC, 0xE5, 0xD7, 0xB3, 0x7B, 0xF6, 0xF1, 0xFF,
  0xE3, 0xDB, 0xAB, 0x4B, 0x96, 0x31, 0x62, 0xC4, 0x95, 0x37, 0x6E, 0xDC, 0xA5, 0x57, 0xAE, 0x41,
  0x82, 0x19, 0x32, 0x64, 0xC8, 0x8D, 0x07, 0x0E, 0x1C, 0x38, 0x70, 0xE0, 0xDD, 0xA7, 0x53, 0xA6,
  0x51, 0xA2, 0x59, 0xB2, 0x79, 0xF2, 0xF9, 0xEF, 0xC3, 0x9B, 0x2B, 0x56, 0xAC, 0x45, 0x8A, 0x09,
  0x12, 0x24, 0x48, 0x90, 0x3D, 0x7A, 0xF4, 0xF5, 0xF7, 0xF3, 0xFB, 0xEB, 0xCB, 0x8B, 0x0B, 0x16,
  0x2C, 0x58, 0xB0, 0x7D, 0xFA, 0xE9, 0xCF, 0x83, 0x1B, 0x36, 0x6C, 0xD8, 0xAD, 0x47, 0x8E, 0x01
};

static const uint8_t gf256_log[256] = {
  0x00, 0x00, 0x01, 0x19, 0x02, 0x32, 0x1A, 0xC6, 0x03, 0xDF, 0x33, 0xEE, 0x1B, 0x68, 0xC7, 0x4B,
  0x04, 0x64, 0xE0, 0x0E, 0x34, 0x8D, 0xEF, 0x81, 0x1C, 0xC1, 0x69, 0xF8, 0xC8, 0x08, 0x4C, 0x71,
  0x05, 0x8A, 0x65, 0x2F, 0xE1, 0x24, 0x0F, 0x21, 0x35, 0x93, 0x8E, 0xDA, 0xF0, 0x12, 0x82, 0x45,
  0x1D, 0xB5, 0xC2, 0x7D, 0x6A, 0x27, 0xF9, 0xB9, 0xC9, 0x9A, 0x09, 0x78, 0x4D, 0xE4, 0x72, 0xA6,
  0x06, 0xBF, 0x8B, 0x62, 0x66, 0xDD, 0x30, 0xFD, 0xE2, 0x98, 0x25, 0xB3, 0x10, 0x91, 0x22, 0x88,
  0x36, 0xD0, 0x94, 0xCE, 0x8F, 0x96, 0xDB, 0xBD, 0xF1, 0xD2, 0x13, 0x5C, 0x83, 0x38, 0x46, 0x40,
  0x1E, 0x42, 0xB6, 0xA3, 0xC3, 0x48, 0x7E, 0x6E, 0x6B, 0x3A, 0x28, 0x54, 0xFA, 0x85, 0xBA, 0x3D,
  0xCA, 0x5E, 0x9B, 0x9F, 0x0A, 0x15, 0x79, 0x2B, 0x4E, 0xD4, 0xE5, 0xAC, 0x73, 0xF3, 0xA7, 0x57,
  0x07, 0x70, 0xC0, 0xF7, 0x8C, 0x80, 0x63, 0x0D, 0x67, 0x4A, 0xDE, 0xED, 0x31, 0xC5, 0xFE, 0x18,
  0xE3, 0xA5, 0x99, 0x77, 0x26, 0xB8, 0xB4, 0x7C, 0x11, 0x44, 0x92, 0xD9, 0x23, 0x20, 0x89, 0x2E,
  0x37, 0x3F, 0xD1, 0x5B, 0x95, 0xBC, 0xCF, 0xCD, 0x90, 0x87, 0x97, 0xB2, 0xDC, 0xFC, 0xBE, 0x61,
  0xF2, 0x56, 0xD3, 0xAB, 0x14, 0x2A, 0x5D, 0x9E, 0x84, 0x3C, 0x39, 0x53, 0x47, 0x6D, 0x41, 0xA2,
  0x1F, 0x2D, 0x43, 0xD8, 0xB7, 0x7B, 0xA4, 0x76, 0xC4, 0x17, 0x49, 0xEC, 0x7F, 0x0C, 0x6F, 0xF6,
  0x6C, 0xA1, 0x3B, 0x52, 0x29, 0x9D, 0x55, 0xAA, 0xFB, 0x60, 0x86, 0xB1, 0xBB, 0xCC, 0x3E, 0x5A,
  0xCB, 0x59, 0x5F, 0xB0, 0x9C, 0xA9, 0xA0, 0x51, 0x0B, 0xF5, 0x16, 0xEB, 0x7A, 0x75, 0x2C, 0xD7,
  0x4F, 0xAE, 0xD5, 0xE9, 0xE6, 0xE7, 0xAD, 0xE8, 0x74, 0xD6, 0xF4, 0xEA, 0xA8, 0x50, 0x58, 0xAF
};

// ====================== GF(2^8) 基本运算 ======================

// GF(2^8) 乘法 — log-exp 查表法, a≠0 且 b≠0 时 O(1)
// a=0 或 b=0 返回 0 (零元)
static inline uint8_t gf256_mul(uint8_t a, uint8_t b) {
  if (a == 0 || b == 0) return 0;
  int sum = gf256_log[a] + gf256_log[b];
  return gf256_exp[sum];  // exp 表 512 条目, 无需 %255
}

// GF(2^8) 求逆 — a^(-1) = α^(255 - log_α(a))
// 返回值: 0 若 a=0 (未定义)
static inline uint8_t gf256_inv(uint8_t a) {
  if (a == 0) return 0;
  return gf256_exp[255 - gf256_log[a]];
}

// GF(2^8) 除法 — a / b = a * b^(-1)
static inline uint8_t gf256_div(uint8_t a, uint8_t b) {
  if (b == 0) return 0;
  return gf256_mul(a, gf256_inv(b));
}

// ====================== RsCfg — RS 配置 (编译时常量) ======================

typedef struct {
  int error_cap;                   // 纠错能力: 可纠正 error_cap 个符号错误 (1..16)
  int block_len;                   // 码长: 固定 255
  int msg_len;                     // 消息长度: block_len - 2*error_cap
  uint8_t gen_poly[RS_MAX_POLY];   // 生成多项式 g(x), gen_poly[i]=x^i 系数, gen_poly[2T]=1
} RsCfg;

// ====================== RsState — RS 解码器运行状态 ======================

typedef struct {
  uint8_t syndromes[RS_MAX_2T];    // 伴随式 S[0..2T-1]
  uint8_t lambda[RS_MAX_POLY];     // 错误位置多项式 Λ(x), lambda[0]=1
  uint8_t omega[RS_MAX_2T];        // 错误值多项式 Ω(x)
  uint8_t err_pos[RS_MAX_T];       // 错误位置 (码字字节索引 0..254)
  uint8_t err_mag[RS_MAX_T];        // 错误值 (纠错幅度)
  int nerr;                         // 找到的错误数
} RsState;

// ====================== rs_init — 初始化配置 + 构建生成多项式 ======================

// 初始化 RS 配置: 设置 T/N/K, 构建生成多项式 g(x)
//   T: 纠错能力 (1..16), 典型值 8 → RS(255, 239)
//
// 构建 g(x) = Π_{i=0}^{2T-1} (x - α^i)
//   从 g(x) = 1 开始, 逐次乘以 (x - α^i)
//   乘法在 GF(256) 中: g_new(x) = g(x)*x + α^i * g(x)
static inline void rs_init(RsCfg *cfg, int T) {
  cfg->error_cap = T;
  cfg->block_len = RS_N;
  cfg->msg_len = RS_N - 2 * T;

  // 清零生成多项式
  memset(cfg->gen_poly, 0, sizeof(cfg->gen_poly));

  // 初始: g(x) = 1 (常数多项式)
  cfg->gen_poly[0] = 1;
  int deg = 0;  // 当前度数

  // 逐次乘以 (x - α^root), root = 0..2T-1
  for (int root = 0; root < 2 * T; root++) {
    uint8_t root_pow = gf256_exp[root]; // α^root

    // 暂存旧系数
    uint8_t old_g[RS_MAX_POLY];
    for (int i = 0; i <= deg; i++) {
      old_g[i] = cfg->gen_poly[i];
    }

    // g(x) = g(x) * x (系数右移一位)
    for (int i = deg + 1; i > 0; i--) {
      cfg->gen_poly[i] = old_g[i - 1];
    }
    cfg->gen_poly[0] = 0;

    // g(x) = g(x) + α^root * old_g(x)
    for (int i = 0; i <= deg; i++) {
      cfg->gen_poly[i] ^= gf256_mul(root_pow, old_g[i]);
    }

    deg++;
  }
  // 此时 deg = 2T, cfg->gen_poly[2T] = 1
}

// ====================== rs_encode — RS 系统编码器 ======================

// 系统编码: 消息字节在前, 2T 校验字节在后
//   msg[0..K-1] → 输入消息
//   codeword[0..K-1] = msg[0..K-1] (原样复制)
//   codeword[K..N-1] = 校验字节 (LFSR 除法余数)
//
// 算法: 线性反馈移位寄存器 (LFSR) 实现多项式除法
//   余数 R(x) = (msg(x) * x^(2T)) mod g(x)
//   每次输入一个消息字节, 反馈 = msg[i] ^ parity[2T-1]
//   移位寄存器右移, 各抽头乘 gen_poly 系数 XOR 反馈
static inline void rs_encode(const RsCfg *cfg, const uint8_t *msg, uint8_t *codeword) {
  int K = cfg->msg_len;
  int T = cfg->error_cap;
  uint8_t parity[RS_MAX_2T];
  memset(parity, 0, sizeof(parity));

  // 复制消息到码字 (系统码: 消息在前)
  for (int i = 0; i < K; i++) {
    codeword[i] = msg[i];
  }

  // LFSR 多项式除法: 计算校验字节
  for (int i = 0; i < K; i++) {
    uint8_t feedback = msg[i] ^ parity[2 * T - 1];

    // 移位寄存器右移: parity[j] ← parity[j-1] ^ gen_poly[j] * feedback
    for (int j = 2 * T - 1; j > 0; j--) {
      parity[j] = parity[j - 1] ^ gf256_mul(cfg->gen_poly[j], feedback);
    }
    parity[0] = gf256_mul(cfg->gen_poly[0], feedback);
  }

  // 追加校验字节 (高位在前: parity[2T-1] 为 x^(2T-1) 系数)
  for (int i = 0; i < 2 * T; i++) {
    codeword[K + i] = parity[2 * T - 1 - i];
  }
}

// ====================== rs_calc_syndromes — 伴随式计算 ======================

// 计算 2T 个伴随式 S_i = r(α^i), i = 0..2T-1
//   r(x) = received[0]*x^(N-1) + received[1]*x^(N-2) + ... + received[N-1]
//   用 Horner 算法: S_i = ((...(received[0])*α^i + received[1])*α^i + ... + received[N-1])
//
// 返回: 非零 表示检测到错误; 0 表示无错 (所有 S_i = 0)
static inline int rs_calc_syndromes(RsState *st, const RsCfg *cfg, const uint8_t *received) {
  int T = cfg->error_cap;
  int N = cfg->block_len;
  int has_error = 0;

  for (int i = 0; i < 2 * T; i++) {
    uint8_t alpha_i = gf256_exp[i];  // α^i
    uint8_t val = received[0];

    // Horner 累加: val = val * α^i + received[j]
    for (int j = 1; j < N; j++) {
      val = gf256_mul(val, alpha_i) ^ received[j];
    }

    st->syndromes[i] = val;
    if (val != 0) has_error = 1;
  }

  return has_error;
}

// ====================== rs_berlekamp_massey — BM 迭代求 Λ(x) ======================

// Berlekamp-Massey 算法: 由伴随式求错误位置多项式 Λ(x)
//   输入: st->syndromes[0..2T-1]
//   输出: st->lambda[0..L] — 错误位置多项式, lambda[0] = 1
//
// 迭代过程:
//   Λ(x) = 1, B(x) = 1, L = 0
//   for r = 0 to 2T-1:
//     计算偏差 δ = Σ_{i=0}^{L} Λ_i * S_{r-i}
//     若 δ ≠ 0:
//       Λ_new(x) = Λ(x) - δ * B(x)  (GF 加法 = XOR)
//       若 2L ≤ r: L = r+1-L, B(x) = Λ_old(x) / δ
//     将 B(x) 乘以 x (移位) 进入下一轮
//
// 返回: L = Λ(x) 的度数; 若 L > T 则错误不可纠正
static inline int rs_berlekamp_massey(RsState *st, const RsCfg *cfg) {
  int T = cfg->error_cap;
  uint8_t *S = st->syndromes;
  uint8_t *lambda_deg = st->lambda;
  uint8_t B[RS_MAX_POLY];

  // 初始化: Λ(x) = 1, B(x) = 1
  memset(lambda_deg, 0, RS_MAX_POLY * sizeof(uint8_t));
  memset(B, 0, sizeof(B));
  lambda_deg[0] = 1;
  B[0] = 1;
  int L = 0;  // Λ(x) 的当前度数

  for (int r = 0; r < 2 * T; r++) {
    // 计算偏差 δ = S_r + Σ_{i=1}^{L} Λ_i * S_{r-i}
    uint8_t delta = S[r];
    for (int i = 1; i <= L; i++) {
      delta ^= gf256_mul(lambda_deg[i], S[r - i]);
    }

    if (delta != 0) {
      // 保存旧 Λ
      uint8_t old_lambda[RS_MAX_POLY];
      memcpy(old_lambda, lambda_deg, sizeof(old_lambda));

      // Λ(x) = Λ(x) + δ * x * B(x)  (GF 加法 = XOR)
      // 乘以 x 后, x^i 系数 = B[i-1]; Λ[0] 不受影响 (保持 =1)
      for (int i = RS_MAX_POLY - 1; i > 0; i--) {
        lambda_deg[i] ^= gf256_mul(delta, B[i - 1]);
      }

      if (2 * L <= r) {
        int new_L = r + 1 - L;
        // B(x) = old_Λ(x) / δ
        uint8_t delta_inv = gf256_inv(delta);
        for (int i = 0; i < RS_MAX_POLY; i++) {
          B[i] = gf256_mul(old_lambda[i], delta_inv);
        }
        L = new_L;
      }
    }

    // B(x) 乘以 x: B_i 后移一位, B_0 = 0
    // 从高索引向低索引移位, 防止覆盖
    for (int i = RS_MAX_POLY - 1; i > 0; i--) {
      B[i] = B[i - 1];
    }
    B[0] = 0;
  }

  return L;  // Λ 的度数; 若 > T 则不可纠正
}

// ====================== rs_chien_search — Chien 搜索求错误位置 ======================

// Chien 搜索: 求 Λ(x) 的根 → 错误位置
//   对 j = 0..N-1, 计算 Λ(α^(-j))
//   若 = 0, 则位置 j 有错误 (pos = N-1-j 为码字中的字节索引)
//
// 增量计算避免重复求幂:
//   eval_k = Λ_k, 每步 eval_k *= α^(-k)
//   Λ(α^(-j)) = Σ eval_k
//
// 返回: 找到的错误数; 若与 Λ 度数不符, 则不可纠正 (返回 -1)
static inline int rs_chien_search(RsState *st, const RsCfg *cfg, int deg_lambda) {
  int N = cfg->block_len;
  int nerr = 0;

  // 预计算 α^(-k) (k = 0..deg_lambda)
  // α^(-k) = α^(255 - k) for k > 0, α^0 = 1
  uint8_t alpha_minus_k[RS_MAX_POLY];
  alpha_minus_k[0] = 1;
  for (int k = 1; k <= deg_lambda; k++) {
    alpha_minus_k[k] = gf256_exp[255 - (k % 255)];
  }

  // 初始 eval[k] = λ_k
  uint8_t eval_terms[RS_MAX_POLY];
  for (int k = 0; k <= deg_lambda; k++) {
    eval_terms[k] = st->lambda[k];
  }

  // 遍历 j = 0..N-1
  for (int j = 0; j < N && nerr < deg_lambda; j++) {
    // 计算 Λ(α^(-j)) = Σ eval_terms[k]
    uint8_t sum = 0;
    for (int k = 0; k <= deg_lambda; k++) {
      sum ^= eval_terms[k];
    }

    if (sum == 0) {
      // 根 α^(-j) → 错误位置 = N-1-j (码字字节索引)
      if (nerr < RS_MAX_T) {
        st->err_pos[nerr] = (uint8_t)(N - 1 - j);
        nerr++;
      }
    }

    // 更新 eval_terms[k] *= α^(-k) 准备下一轮
    for (int k = 0; k <= deg_lambda; k++) {
      eval_terms[k] = gf256_mul(eval_terms[k], alpha_minus_k[k]);
    }
  }

  // 找到的根数必须等于 Λ 的度数, 否则不可纠正
  if (nerr != deg_lambda) {
    return -1;
  }

  return nerr;
}

// ====================== rs_forney — Forney 算法求错误值 ======================

// Forney 算法: 由 Ω(x) 和 Λ'(x) 计算每个错误位置上的错误幅度
//
//   1. 计算 Ω(x) = S(x) * Λ(x) mod x^(2T)
//   2. 对每个错误位置 pos (错误定位子 X = α^(N-1-pos)):
//      error_mag = X * Ω(X^(-1)) / Λ'(X^(-1))
//      Λ'(x) 为形式导数 (GF(2) 中仅奇次项保留)
//
// 返回: 0 成功; -1 失败 (除法中遇到零除数)
static inline int rs_forney(RsState *st, const RsCfg *cfg, int deg_lambda, int nerr) {
  int T = cfg->error_cap;
  int N = cfg->block_len;
  uint8_t *S = st->syndromes;
  uint8_t *lambda_deg = st->lambda;

  // ---- 第1步: 计算 Ω(x) = S(x) * Λ(x) mod x^(2T) ----
  memset(st->omega, 0, sizeof(st->omega));
  for (int i = 0; i < 2 * T; i++) {
    for (int j = 0; j <= deg_lambda && (i + j) < 2 * T; j++) {
      st->omega[i + j] ^= gf256_mul(S[i], lambda_deg[j]);
    }
  }

  // ---- 第2步: 对每个错误位置计算幅度 ----
  for (int err = 0; err < nerr; err++) {
    uint8_t pos = st->err_pos[err];
    // X = α^(N-1-pos) — 错误定位子
    int exp_val = (N - 1 - pos) % 255;
    uint8_t X = gf256_exp[exp_val];
    // X_inv = X^(-1) = α^(-(N-1-pos))
    uint8_t X_inv = gf256_exp[(255 - exp_val) % 255];

    // 用 Horner 计算 Ω(X^(-1))
    uint8_t omega_val = 0;
    for (int k = 2 * T - 1; k >= 0; k--) {
      omega_val = gf256_mul(omega_val, X_inv) ^ st->omega[k];
    }

    // 用 Horner 计算 Λ'(X^(-1)) — 仅奇次项
    // Λ'(x) = λ_1 + λ_3*x^2 + λ_5*x^4 + ...
    // Horner: val = (((...(λ_odd_max)*X_inv^2 + λ_odd_prev)*X_inv^2 ... )
    uint8_t lambda_deriv = 0;
    for (int k = deg_lambda; k >= 1; k--) {
      if (k & 1) {
        // 奇次项: 加入 λ_k, 然后乘 X_inv^(k-1) 的剩余部分
        lambda_deriv = gf256_mul(lambda_deriv, X_inv) ^ lambda_deg[k];
      } else {
        // 偶次项: 仅乘 X_inv, 不加 λ_k (导数为 0)
        lambda_deriv = gf256_mul(lambda_deriv, X_inv);
      }
    }
    // 最后一步乘 X_inv (k=0 时仅乘法)
    // 实际上 deg_lambda downto 1 已经正确了

    // 检查除数是否为零
    if (lambda_deriv == 0) {
      return -1;  // 不可纠正
    }

    // 错误幅度 = X * Ω(X^(-1)) / Λ'(X^(-1))
    uint8_t numer = gf256_mul(X, omega_val);
    st->err_mag[err] = gf256_div(numer, lambda_deriv);
  }

  return 0;
}

// ====================== rs_decode — 完整 RS 解码流水线 ======================

// 完整解码流程:
//   1. 伴随式计算 — 检测是否有错
//   2. Berlekamp-Massey — 求 Λ(x)
//   3. Chien 搜索 — 求错误位置
//   4. Forney 算法 — 求错误幅度
//   5. 纠错: corrected[i] = received[i] ^ error_mag[pos=i]
//
// 返回:
//   >= 0 — 成功纠正的错误数 (0 = 无错)
//   -1  — 不可纠正 (错误数 > T 或 根数不匹配)
static inline int rs_decode(RsState *st, const RsCfg *cfg, const uint8_t *received, uint8_t *corrected) {
  int T = cfg->error_cap;
  int N = cfg->block_len;

  // 初始化状态
  memset(st, 0, sizeof(RsState));
  st->nerr = -1;

  // 复制接收数据到输出 (默认无错)
  memcpy(corrected, received, N);

  // ---- 第1步: 伴随式计算 ----
  int has_error = rs_calc_syndromes(st, cfg, received);
  if (!has_error) {
    st->nerr = 0;
    return 0;  // 无错误
  }

  // ---- 第2步: Berlekamp-Massey 求 Λ(x) ----
  int deg_lambda = rs_berlekamp_massey(st, cfg);
  if (deg_lambda > T || deg_lambda <= 0) {
    return -1;  // 不可纠正
  }

  // ---- 第3步: Chien 搜索求错误位置 ----
  int nerr = rs_chien_search(st, cfg, deg_lambda);
  if (nerr < 0 || nerr > T) {
    return -1;  // 根数不匹配 或 超出纠错能力
  }

  // ---- 第4步: Forney 算法求错误幅度 ----
  if (rs_forney(st, cfg, deg_lambda, nerr) < 0) {
    return -1;  // Forney 失败
  }

  // ---- 第5步: 纠错 (XOR) ----
  for (int i = 0; i < nerr; i++) {
    uint8_t pos = st->err_pos[i];
    corrected[pos] = received[pos] ^ st->err_mag[i];
  }

  st->nerr = nerr;
  return nerr;
}

// ====================== rs_get_errors — 查询纠正的错误数 ======================

// 返回最近一次 rs_decode() 纠正的符号数
//   0 = 无错, -1 = 未执行 或 不可纠正
static inline int rs_get_errors(const RsState *st) {
  return st->nerr;
}

#endif  // COMP_RS_H
