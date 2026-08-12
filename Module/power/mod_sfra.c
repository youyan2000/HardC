// SFRA 软件频率响应分析仪 — 在线 Bode 图测量实现
//
// 来源: TI controlSUITE SFRA/v1.20/Float (SFRA_F_Include.h)
// 翻译为 C-OOP 纯C float 版本
//
// 算法细节:
//   1. DDS 正弦注入: inject_out = A * sin(phase_dds), phase_dds += ω*dt
//      相位连续跨频点, 避免频点切换时的相位跳变
//   2. DFT 提取 (测量窗口内):
//      inj_re  = Σ inject[n]  * cos(ωnT),  inj_im  = Σ inject[n]  * (-sin)(ωnT)
//      resp_re = Σ response[n] * cos(ωnT),  resp_im = Σ response[n] * (-sin)(ωnT)
//   3. 传递函数: |H| = mag(resp) / mag(inj),  ∠H = ∠resp - ∠inj
//      gain_db = 20*log10(|H|),  phase_deg = atan2(resp_im,resp_re) - atan2(inj_im,inj_re)
//   4. 对数扫频: freq[k] = f_start * 10^(k * decade_step),  decade_step = log10(f_end/f_start)/(total-1)
//
// 数值考虑:
//   - DFT 相位在 collect() 末尾递增, 保证 inject 和 collect 同一 ISR tick 共享同一相位
//   - 相位使用 atan2f 保证全象限正确, 结果归一化到 [-180°, 180°]
//   - 幅值比用 DB_MIN 保护, 防止 log10(0) 产生 -inf

#include "mod_sfra.h"
#include <math.h>

#define SFRA_PI     3.14159265f
#define SFRA_2PI    6.28318531f
#define SFRA_DB_MIN 1e-12f       // 最小幅值保护 (防止 log10(0) → -inf)

// ======== 初始化 & 重置 ========

void mod_sfra_init(Sfra *me, float dt, const SfraCfg *cfg) {
  me->cfg = *cfg;
  me->dt = dt;
  me->on_point_done = NULL;
  me->user_data = NULL;
  mod_sfra_reset(me);
}

void mod_sfra_reset(Sfra *me) {
  // 仅重置运行时状态, 保留 cfg / dt / 回调
  me->current_freq  = me->cfg.f_start;
  me->current_omega = SFRA_2PI * me->current_freq;
  me->freq_index    = 0;
  me->total_freqs   = 0;
  me->cycle_counter = 0;
  me->settling      = true;
  me->sweep_done    = false;
  me->running       = false;

  me->inject_phase = 0.0f;
  me->inject_out   = 0.0f;
  me->dft_phase    = 0.0f;

  me->inj_re  = 0.0f;
  me->inj_im  = 0.0f;
  me->resp_re = 0.0f;
  me->resp_im = 0.0f;

  me->gain_db      = 0.0f;
  me->phase_deg    = 0.0f;
  me->sample_count = 0;
}

// ======== 扫频控制 ========

// 计算总频点数 (对数分布, 含两端点)
// total = points_per_decade * log10(f_end / f_start) + 1
static int sfra_calc_total_freqs(const SfraCfg *cfg) {
  if (cfg->f_start <= 0.0f || cfg->f_end <= cfg->f_start) {
    return 1;  // 无效配置, 至少出一个点
  }
  float decades = log10f(cfg->f_end / cfg->f_start);
  int total = (int)(cfg->points_per_decade * decades) + 1;
  return (total < 1) ? 1 : total;
}

// 根据频点序号计算频率 (对数步进)
// freq[k] = f_start * 10^(k * decade_per_step)
static float sfra_freq_by_index(const SfraCfg *cfg, int index, int total) {
  if (total <= 1) return cfg->f_start;
  float decades = log10f(cfg->f_end / cfg->f_start);
  float step = decades / (float)(total - 1);
  float freq = cfg->f_start * powf(10.0f, step * (float)index);
  return freq;
}

void mod_sfra_start(Sfra *me) {
  // 计算频点列表
  me->total_freqs = sfra_calc_total_freqs(&me->cfg);

  // 从起始频率开始
  me->freq_index    = 0;
  me->current_freq  = me->cfg.f_start;
  me->current_omega = SFRA_2PI * me->current_freq;

  // 初始化状态
  me->cycle_counter = 0;
  me->settling      = true;
  me->sweep_done    = false;
  me->running       = true;

  // DDS 相位从 0 开始, 减少初始瞬态
  me->inject_phase  = 0.0f;
  me->inject_out    = 0.0f;
}

void mod_sfra_stop(Sfra *me) {
  me->running    = false;
  me->sweep_done = false;
  // 不清零 inject_out: 由下一周期 inject() 处理
}

// ======== ISR 扰动注入 ========

void mod_sfra_inject(Sfra *me) {
  if (!me->running) {
    me->inject_out = 0.0f;
    return;
  }

  // 每个 ISR tick 计数 (稳定+测量都计数)
  me->cycle_counter++;

  // DDS 正弦波: inject_out = A * sin(phase_dds)
  me->inject_out = me->cfg.inject_amp * sinf(me->inject_phase);

  // DDS 相位累加 (连续运行, 独立于 DFT)
  me->inject_phase += me->current_omega * me->dt;

  // 相位折叠到 [-PI, PI], 防止浮点精度累积
  if (me->inject_phase > SFRA_2PI) {
    me->inject_phase -= SFRA_2PI;
  }

  // DFT 累加 — 仅在测量阶段
  if (me->settling) return;

  // 注入通道 DFT: X_inj = Σ inject[n] * (cos(ωnT) - j*sin(ωnT))
  float cos_val  = cosf(me->dft_phase);
  float nsin_val = -sinf(me->dft_phase);  // DFT 的 -j*sin 项

  me->inj_re += me->inject_out * cos_val;
  me->inj_im += me->inject_out * nsin_val;
}

// ======== ISR 响应采集 ========

void mod_sfra_collect(Sfra *me, float response) {
  if (!me->running || me->settling) return;

  // 响应通道 DFT (使用与 inject 相同的 dft_phase)
  float cos_val  = cosf(me->dft_phase);
  float nsin_val = -sinf(me->dft_phase);

  me->resp_re += response * cos_val;
  me->resp_im += response * nsin_val;

  me->sample_count++;

  // DFT 相位累加 — 放在 collect 末尾, 保证 inject 和 collect 同一 tick 共享同一相位
  me->dft_phase += me->current_omega * me->dt;
  if (me->dft_phase > SFRA_2PI) {
    me->dft_phase -= SFRA_2PI;
  }
}

// ======== 后台处理 ========

// 角度归一化到 [-180°, 180°]
static float sfra_norm_phase_deg(float deg) {
  while (deg > 180.0f)  deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

void mod_sfra_background(Sfra *me) {
  if (!me->running) return;

  // ---- 稳定阶段: 等待 settle_cycles 个 ISR 周期 ----
  if (me->settling) {
    if (me->cycle_counter >= me->cfg.settle_cycles) {
      // 稳定完成, 进入测量阶段, 清零 DFT 累加器
      me->settling      = false;
      me->cycle_counter = 0;
      me->sample_count  = 0;
      me->dft_phase     = 0.0f;
      me->inj_re        = 0.0f;
      me->inj_im        = 0.0f;
      me->resp_re       = 0.0f;
      me->resp_im       = 0.0f;
    }
    return;
  }

  // ---- 测量阶段: 等待 measure_cycles 个 ISR 周期 ----
  if (me->cycle_counter >= me->cfg.measure_cycles) {
    // === DFT 结算 ===

    // 注入通道幅值
    float inj_mag = sqrtf(me->inj_re * me->inj_re + me->inj_im * me->inj_im);
    // 响应通道幅值
    float resp_mag = sqrtf(me->resp_re * me->resp_re + me->resp_im * me->resp_im);

    // 增益: 20*log10(|H|) = 20*log10(mag_resp / mag_inj)
    if (inj_mag < SFRA_DB_MIN) {
      me->gain_db = 0.0f;  // 注入信号检测不到, 结果不可信
    } else {
      float ratio = resp_mag / inj_mag;
      if (ratio < SFRA_DB_MIN) ratio = SFRA_DB_MIN;
      me->gain_db = 20.0f * log10f(ratio);
    }

    // 相位: ∠H = ∠resp - ∠inj
    // atan2f(y, x) 返回 [-π, π]
    // DFT 中 Im = -Σ x*sin, 所以相位 = atan2(Im, Re)
    float inj_phase  = atan2f(me->inj_im, me->inj_re);
    float resp_phase = atan2f(me->resp_im, me->resp_re);
    float phase_diff = resp_phase - inj_phase;      // (rad)
    me->phase_deg = sfra_norm_phase_deg(phase_diff * 180.0f / SFRA_PI);

    // === 回调通知 ===
    if (me->on_point_done) {
      me->on_point_done(me->user_data, me->freq_index, me->total_freqs,
                         me->current_freq, me->gain_db, me->phase_deg);
    }

    // === 下一个频点 ===
    me->freq_index++;
    if (me->freq_index >= me->total_freqs) {
      // 扫频完成
      me->sweep_done = true;
      me->running    = false;
      return;
    }

    // 计算新频率 (对数步进)
    me->current_freq  = sfra_freq_by_index(&me->cfg, me->freq_index, me->total_freqs);
    me->current_omega = SFRA_2PI * me->current_freq;

    // 进入新频点的稳定阶段
    me->settling      = true;
    me->cycle_counter = 0;
    me->sample_count  = 0;
  }
}
