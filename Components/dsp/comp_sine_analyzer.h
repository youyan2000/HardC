// 正弦波分析器 — 过零检测 + RMS/均值/频率计算
//
// 来源: TI controlSUITE SineAnalyzer (digital_power / solar)
// 翻译为 C-OOP 纯C float 版本
//
// 算法:
//   每周期: 检测过零 (阈值比较) → 累加样本 → 半周期后计算:
//     Vavg = sum(V) / N              (均值)
//     Vrms = sqrt(sum(V²) / N)       (有效值)
//     Freq = Fsamp / (2*N)           (频率, 因子2因为半周期)
//
// 调用方式:
//   sine_analyzer_run(&sa, v_sample);  // ISR 中每控制周期/每ADC采样调用
//   float vrms = sa.vrms;              // 每半周期更新一次
//
// 注: 所有累加器在 struct 内 (不像 TI 原版用全局变量), 支持多实例

#ifndef COMP_SINE_ANALYZER_H
#define COMP_SINE_ANALYZER_H

#include <math.h>
#include <stdint.h>

// ======================= SineAnalyzer (单路电压分析) =======================

typedef struct {
  // 输入
  float vin;              // 输入: 当前采样值
  float sample_freq;      // 参数: 采样频率 (Hz)
  float threshold;        // 参数: 过零检测阈值 (如 1.65V)

  // 输出 (每半周期更新)
  float vrms;             // 输出: RMS 有效值
  float vavg;             // 输出: 平均值
  float sig_freq;         // 输出: 信号频率 (Hz)

  // 过零检测
  int   zcd;              // 输出: 过零标志 (0=无, 1=正向过零)
  int   positive_cycle;   // 输出: 正半周标志 (0=负半周, 1=正半周)

  // 内部累加器 (每个实例独立)
  float vacc_avg;         // 累加: ΣVin
  float vacc_rms;         // 累加: ΣVin²
  int   prev_sign;        // 上一拍符号 (0=负, 1=正)
  int   curr_sign;        // 当前符号
  int   nsamples;         // 半周期内采样数

  float curr_norm;        // 归一化当前采样
} SineAnalyzer;

#define SINE_ANALYZER_DEFAULTS { 0, 0, 1.65f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }

// 初始化
static inline void sine_analyzer_init(SineAnalyzer *me, float sample_freq,
                                       float threshold) {
  me->sample_freq = sample_freq;
  me->threshold = threshold;
  me->vrms = 0.0f;
  me->vavg = 0.0f;
  me->sig_freq = 0.0f;
  me->zcd = 0;
  me->positive_cycle = 0;

  me->vacc_avg = 0.0f;
  me->vacc_rms = 0.0f;
  me->prev_sign = 1;
  me->curr_sign = 1;
  me->nsamples = 0;
  me->curr_norm = 0.0f;
}

// 单步运行 (每个采样周期调用)
static inline void sine_analyzer_run(SineAnalyzer *me, float sample) {
  me->vin = sample;

  // 步骤 1: 阈值比较 → 判定符号
  if (sample > me->threshold) {
    me->curr_norm = sample;          // 也可以用 sample - threshold
    me->curr_sign = 1;
  } else {
    me->curr_norm = sample;
    me->curr_sign = 0;
  }
  me->positive_cycle = me->curr_sign;

  // 步骤 2: 过零检测 (负→正跳变)
  if ((me->prev_sign != me->curr_sign) && (me->curr_sign == 1)) {
    // ---- 过零! 计算本半周期结果 ----
    me->zcd = 1;

    if (me->nsamples > 0) {
      float inv_n = 1.0f / (float)me->nsamples;

      // 均值
      me->vavg = me->vacc_avg * inv_n;

      // 有效值 = sqrt( Σ(V²) / N )
      float mean_sq = me->vacc_rms * inv_n;
      if (mean_sq > 0.0f) {
        me->vrms = sqrtf(mean_sq);
      } else {
        me->vrms = 0.0f;
      }

      // 频率 = Fsamp / (2 × N)  (半周期 → 全周期)
      me->sig_freq = me->sample_freq * inv_n * 0.5f;
    }

    // 重置累加器
    me->vacc_avg = 0.0f;
    me->vacc_rms = 0.0f;
    me->nsamples = 0;

  } else {
    // ---- 正常累加 ----
    me->zcd = 0;
    me->nsamples++;

    me->vacc_avg += me->curr_norm;
    me->vacc_rms += me->curr_norm * me->curr_norm;
  }

  me->prev_sign = me->curr_sign;
}

// ======================= SineAnalyzer2 (双路 V+I 分析, 含功率) =======================

// 扩展版: 同时分析电压和电流, 计算功率
// 来源: TI SineAnalyzer_diff_wPower (含 IRMS, PRMS)
typedef struct {
  // 输入
  float vin;              // 电压采样
  float iin;              // 电流采样
  float sample_freq;
  float threshold;

  // 输出
  float vrms, irms;       // 电压/电流 RMS
  float prms;             // 有功功率 = avg(V×I)
  float vavg;             // 电压均值
  float sig_freq;         // 信号频率
  int   zcd;
  int   positive_cycle;

  // 内部
  float vacc_avg, vacc_rms;    // 电压累加
  float iacc_rms;              // 电流平方累加
  float pacc_rms;              // 瞬时功率累加 Σ(V×I)
  int   prev_sign, curr_sign;
  int   nsamples;
  float curr_vin_norm, curr_iin_norm;

  // 功率平滑 (100 周期移动平均)
  float sum_pacc_mul;          // Σ(Pavg_n)
  int   slew_counter;          // 平滑计数器 0~100
} SineAnalyzer2;

#define SINE_ANALYZER2_DEFAULTS { 0,0,0,1.65f, 0,0,0,0,0, 0,0, 0,0,0, 0,0,0, 0,0, 0,0 }

// 初始化
static inline void sine_analyzer2_init(SineAnalyzer2 *me, float sample_freq,
                                        float threshold) {
  me->sample_freq = sample_freq;
  me->threshold = threshold;
  me->vrms = 0.0f; me->irms = 0.0f; me->prms = 0.0f;
  me->vavg = 0.0f; me->sig_freq = 0.0f;
  me->zcd = 0; me->positive_cycle = 0;

  me->vacc_avg = 0.0f; me->vacc_rms = 0.0f;
  me->iacc_rms = 0.0f; me->pacc_rms = 0.0f;
  me->prev_sign = 1; me->curr_sign = 1;
  me->nsamples = 0;
  me->curr_vin_norm = 0.0f; me->curr_iin_norm = 0.0f;
  me->sum_pacc_mul = 0.0f; me->slew_counter = 0;
}

// 双路单步运行
static inline void sine_analyzer2_run(SineAnalyzer2 *me, float v_sample,
                                       float i_sample) {
  me->vin = v_sample;
  me->iin = i_sample;

  // 阈值比较 (只用电压做过零检测)
  if (v_sample > me->threshold) {
    me->curr_vin_norm = v_sample;
    me->curr_iin_norm = i_sample;
    me->curr_sign = 1;
  } else {
    me->curr_vin_norm = v_sample;
    me->curr_iin_norm = i_sample;
    me->curr_sign = 0;
  }
  me->positive_cycle = me->curr_sign;

  // 过零检测
  if ((me->prev_sign != me->curr_sign) && (me->curr_sign == 1)) {
    me->zcd = 1;

    if (me->nsamples > 0) {
      float inv_n = 1.0f / (float)me->nsamples;

      me->vavg = me->vacc_avg * inv_n;

      // Vrms
      float mean_vsq = me->vacc_rms * inv_n;
      me->vrms = (mean_vsq > 0.0f) ? sqrtf(mean_vsq) : 0.0f;

      // Irms
      float mean_isq = me->iacc_rms * inv_n;
      me->irms = (mean_isq > 0.0f) ? sqrtf(mean_isq) : 0.0f;

      // 频率
      me->sig_freq = me->sample_freq * inv_n * 0.5f;

      // 功率平滑 (100 周期滚动平均)
      float pavg = me->pacc_rms * inv_n;
      me->slew_counter++;
      if (me->slew_counter >= 101) {
        me->slew_counter = 0;
        me->prms = me->sum_pacc_mul * 0.01f;
        me->sum_pacc_mul = 0.0f;
      } else {
        me->sum_pacc_mul += pavg;
      }
    }

    // 重置
    me->vacc_avg = 0.0f;
    me->vacc_rms = 0.0f;
    me->iacc_rms = 0.0f;
    me->pacc_rms = 0.0f;
    me->nsamples = 0;

  } else {
    me->zcd = 0;
    me->nsamples++;

    me->vacc_avg += me->curr_vin_norm;
    me->vacc_rms += me->curr_vin_norm * me->curr_vin_norm;
    me->iacc_rms += me->curr_iin_norm * me->curr_iin_norm;
    me->pacc_rms += me->curr_vin_norm * me->curr_iin_norm;
  }

  me->prev_sign = me->curr_sign;
}

#endif  // COMP_SINE_ANALYZER_H
