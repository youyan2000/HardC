// 光伏电弧检测 — FFT 频带能量 + 阈值判定 (TI tida_010231 电弧检测算法)
//
// 来源: TI C2000Ware Digital Power SDK solutions/tida_010231/source/arc/source/ArcDetect.c
//       (arc_detect() 频带能量计算) + arc_isr.c (阈值判定)
// 翻译为 C-OOP 纯C float 版本, 与 FFT 后端解耦 (输入取幅值平方谱)
//
// 直流电弧特征: 频谱在宽频带内能量大幅抬升 (宽带噪声), 而正常负载/开关噪声集中在窄带 (开关谐波). 因此电弧检测核心 = 分析频带内宽带能量 → dB,越限即判电弧.
//
// 算法 (arc_detect):
//   1. 频带 [bin_min, bin_max) 等分低频带/高频带, 各找最小值 minval
//   2. 单频干扰滤除: 重复 FilterBins 次, 把每半带最强 bin 替换为本半带最小值
//      (滤掉开关谐波等窄带干扰, FilterBins = NumBins·D·0.5)
//   3. 频带能量: BandSum = F·Σ(低频带) + Σ(高频带)  (F 为低频带加权)
//   4. 转 dB: dB = 10·log10(BandSum) + 2.129 − 90.31 + AD_correction
//      (2.129 = Hanning 窗幅值修正, 90.31 = 满刻度后处理偏置)
//   5. 判定: dB > 阈值 T → 电弧 (is_arc)
//
// 输入: 幅值平方谱 mag_sq[k] = Re²+Im² (由用户 FFT 后端计算, 如 bsp_dsp_fft)
// 注意: 谱分辨率 bin 带宽 = 采样率/FFT点数, 由用户按采样配置换算 bin 范围

#ifndef COMP_ARC_DETECT_H
#define COMP_ARC_DETECT_H

#include <math.h>
#include <stdint.h>

// 分析频带最大宽度 (bin 数) — 用户可调, 需 ≥ bin_max−bin_min
#define ARC_BAND_MAX  256u

// ======================= 电弧检测器 =======================

typedef struct {
  // 参数
  uint16_t bin_min;       // 参数: 分析频带起始 bin
  uint16_t bin_max;       // 参数: 分析频带结束 bin (不含)
  float band_weight;      // 参数: F 低频带加权系数 (乘在低频带能量上)
  float bin_discard;      // 参数: D 单频干扰滤除比例 (0~1, 默认 0 关闭)
  float threshold;        // 参数: T 电弧判定阈值 (dB 域)
  float post_scale;       // 参数: 后处理偏置 (默认 90.31, Hanning 满刻度补偿)
  float ad_correction;    // 参数: AD 标定修正值 (默认 0)

  // 内部
  uint16_t bin_mid;       // 频带中点 (低频带/高频带分界)
  uint16_t filter_bins;   // 单频干扰滤除次数 = NumBins·D·0.5
  float scratch[ARC_BAND_MAX];  // 滤除工作区 (频带副本)

  // 输出
  float band_power_db;    // 输出: 频带能量 (dB)
  int   is_arc;           // 输出: 电弧判定 (1=电弧, 0=正常)
} ArcDetect;

// 初始化
//   bin_min/bin_max — 分析频带 (典型覆盖 20kHz~80kHz 电弧特征频带)
//   band_weight     — 低频带加权 F (默认 64)
//   bin_discard     — 单频干扰滤除比例 D (默认 0 = 关闭)
//   threshold       — 电弧判定阈值 T (dB 域, 默认 220)
static inline void arc_detect_init(ArcDetect *me, uint16_t bin_min,
                                   uint16_t bin_max, float band_weight,
                                   float bin_discard, float threshold) {
  me->bin_min = bin_min;
  me->bin_max = bin_max;
  me->band_weight = band_weight;
  me->bin_discard = bin_discard;
  me->threshold = threshold;
  me->post_scale = 90.31f;    // 20·log10(32768) — AD 满刻度偏置
  me->ad_correction = 0.0f;

  uint16_t num_bins = (uint16_t)(bin_max - bin_min);
  me->bin_mid = (uint16_t)(bin_min + num_bins / 2u);
  me->filter_bins = (uint16_t)((float)num_bins * bin_discard * 0.5f + 0.5f);

  me->band_power_db = 0.0f;
  me->is_arc = 0;
}

// 单次检测 — FFT 完成后调用
//   mag_sq — 幅值平方谱 (长度 ≥ bin_max), mag_sq[k] = Re²+Im²
//   返回: 频带能量 (dB)
static inline float arc_detect_run(ArcDetect *me, const float *mag_sq) {
  uint16_t i, j, k;
  float min_b0, min_b1, max_val, band_sum, db;
  uint16_t max_loc;

  // ---- 1. 频带副本 → 工作区 (不破坏调用者谱数据) ----
  for (i = me->bin_min; i < me->bin_max; i++) {
    me->scratch[i - me->bin_min] = mag_sq[i];
  }

  // ---- 2. 各半带最小值 (预加载保证有效) ----
  min_b0 = me->scratch[0];                          // 低频带 [0, bin_mid)
  for (i = 1u; i < (uint16_t)(me->bin_mid - me->bin_min); i++) {
    if (me->scratch[i] < min_b0) min_b0 = me->scratch[i];
  }
  min_b1 = me->scratch[me->bin_mid - me->bin_min];  // 高频带 [bin_mid, bin_max)
  for (i = (uint16_t)(me->bin_mid - me->bin_min); i < (uint16_t)(me->bin_max - me->bin_min); i++) {
    if (me->scratch[i] < min_b1) min_b1 = me->scratch[i];
  }

  // ---- 3. 单频干扰滤除: 重复滤掉每半带最强 bin, 替换为本半带最小值 ----
  for (i = 0u; i < me->filter_bins; i++) {
    // 低频带最强 bin
    max_val = 0.0f;
    max_loc = 0u;
    for (j = 0u; j < (uint16_t)(me->bin_mid - me->bin_min); j++) {
      if (me->scratch[j] > max_val) {
        max_val = me->scratch[j];
        max_loc = j;
      }
    }
    me->scratch[max_loc] = min_b0;
    // 高频带最强 bin (max_loc 复位到高频带起始, 防止全零时误写低频带)
    max_val = 0.0f;
    max_loc = (uint16_t)(me->bin_mid - me->bin_min);
    for (k = (uint16_t)(me->bin_mid - me->bin_min); k < (uint16_t)(me->bin_max - me->bin_min); k++) {
      if (me->scratch[k] > max_val) {
        max_val = me->scratch[k];
        max_loc = k;
      }
    }
    me->scratch[max_loc] = min_b1;
  }

  // ---- 4. 频带能量: F·Σ(低频带) + Σ(高频带) ----
  band_sum = 0.0f;
  for (i = 0u; i < (uint16_t)(me->bin_mid - me->bin_min); i++) {
    band_sum += me->scratch[i];
  }
  band_sum *= me->band_weight;
  for (; i < (uint16_t)(me->bin_max - me->bin_min); i++) {
    band_sum += me->scratch[i];
  }

  if (band_sum <= 0.0f) {
    band_sum = 1.0e-20f;
  }

  // ---- 5. 转 dB: 10·log10(BandSum) + Hanning 修正 − 满刻度偏置 + AD 标定 ----
  db = (10.0f / 2.30258509299f) * logf(band_sum);   // 10/ln(10) = 4.3429
  db += 2.129f;                                     // Hanning 窗幅值修正
  db -= me->post_scale;
  db += me->ad_correction;

  me->band_power_db = db;

  // ---- 6. 阈值判定 ----
  me->is_arc = (db > me->threshold) ? 1 : 0;

  return db;
}

// 读取判定结果 (arc_detect_run 后)
static inline int arc_detect_check(const ArcDetect *me) {
  return me->is_arc;
}

#endif  // COMP_ARC_DETECT_H
