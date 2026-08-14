/**
 * @file    adc_ac_sampler.c
 * @brief   三相交流采样器实现 —— AdcBase 子类
 * @note    参考 drv_adc_sampler.c
 *
 * 实现 AdcOps 虚函数表:
 *   .start_dma → start_dma_impl (启动 ADC DMA 循环扫描)
 *   .read_ch   → read_ch_impl   (读取通道 i 原始值)
 *   .process   → process_impl   (工程量转换 + 三相重构 + RMS + Vdc)
 *
 * 通道数灵活配置:
 *   - num_v 路电压 (差分, ≤4):  v_val[i] = diff_to_eng(raw[v_ch[i]], v_gain[i])
 *   - num_i 路电流 (差分, ≤8):  i_val[i] = diff_to_eng(raw[i_ch[i]], i_gain[i])
 *   - 1 路参考 (单端):          vref = se_to_voltage(raw[vref_ch])
 *
 * 三相重构 (process / fast_fetch 中调用):
 *   - 电压: Vab, Vbc → Va, Vb, Vc
 *   - 电流: Ia, Ic  → Ia, Ib, Ic  (KCL: Ib = -(Ia+Ic))
 *
 * ISR 安全: fast_fetch 仅标量浮点运算 (无 arm_rms_f32 / sqrtf)
 * 参考: RM0440 §21.4.6 差分模式 DIFSEL 寄存器
 */

#include "adc_ac_sampler.h"
#include "container_of.h"
#include "arm_math.h"   // arm_rms_f32
#include <math.h>        // sqrtf
#include <string.h>      // memset
#include <assert.h>

// ====== 校准常量 (static const — 类型安全) =====================================
static const float adc_vref = 3.30f;     // ADC 参考电压 (V)
static const float adc_res  = 4096.0f;   // 12-bit 分辨率
static const float vsensor_gain_default  = 15.15f;  // 电压传感器分压比
static const float isensor_gain_default  = 9.60f;   // 电流传感器增益 (A/V)

// RMS 缓冲
enum { RMS_BUF_SIZE = 800 };  // 50Hz 一个周期 = 20ms @ 40kHz
static float vrms_buf[RMS_BUF_SIZE];
static float irms_buf[RMS_BUF_SIZE];
static uint32_t rms_idx = 0;

// ====== 工具函数 =============================================================

// 差分通道: int16_t → 工程量
// raw = ADC 差分結果 (IN_P - IN_N), -4095 ~ +4095 對應 ±3.3V
static float diff_to_eng(int16_t raw, float gain) {
  float v = (float)raw * adc_vref / adc_res;  // 有符號電壓 (±3.3V)
  return v * gain;                              // → 工程量 (±A 或 ±V)
}

// 单端通道: 原始值 → 电压
static float se_to_voltage(int16_t raw) {
  return (float)((uint16_t)raw) * adc_vref / adc_res;
}

// ====== ops 实现 (static — 封装) =============================================

// 启动 ADC DMA 循环扫描
static void start_dma_impl(AdcBase *base) {
  AdcAcSampler *me = container_of(base, AdcAcSampler, base);
  HAL_ADC_Start_DMA(me->hadc, (uint32_t *)base->raw, me->num_ch);
}

// 读取通道 i 的原始 ADC 值
static uint16_t read_ch_impl(AdcBase *base, int i) {
  if (i < 0 || i >= base->raw_cap) return 0;
  return base->raw[i];
}

// === 三相重构辅助函数 ========================================================

// 线电压 → 相电压 (三相平衡系统)
// Va =  (2*Vab + Vbc) / 3
// Vb =  (-Vab + Vbc) / 3
// Vc =  (-Vab - 2*Vbc) / 3
void adc_ac_sampler_reconstruct_v(AdcAcSampler *me,
                                  uint8_t idx_vab, uint8_t idx_vbc) {
  float vab = me->v_val[idx_vab];
  float vbc = me->v_val[idx_vbc];
  me->va =  (2.0f * vab + vbc) / 3.0f;
  me->vb =  (-vab + vbc) / 3.0f;
  me->vc =  (-vab - 2.0f * vbc) / 3.0f;
}

// 两相电流 → 三相电流 (KCL: Ia + Ib + Ic = 0)
void adc_ac_sampler_reconstruct_i(AdcAcSampler *me,
                                  uint8_t idx_ia, uint8_t idx_ic,
                                  float *ia, float *ib, float *ic) {
  *ia = me->i_val[idx_ia];
  *ic = me->i_val[idx_ic];
  *ib = -(*ia + *ic);
}

// === 传感器数据处理 (主循环调用, 含重型 DSP) ================================

static void process_impl(AdcBase *base) {
  AdcAcSampler *me = container_of(base, AdcAcSampler, base);

  // ---- 第1步: 原始 ADC → 工程量 (所有通道) --------------------------------
  // 电压通道 (差分)
  for (uint8_t i = 0; i < me->num_v; i++) {
    me->v_val[i] = diff_to_eng(base->raw[me->v_ch[i]], me->v_gain[i]);
  }

  // 电流通道 (差分)
  for (uint8_t i = 0; i < me->num_i; i++) {
    me->i_val[i] = diff_to_eng(base->raw[me->i_ch[i]], me->i_gain[i]);
  }

  // 参考电压 (单端)
  me->vref_measured = se_to_voltage(base->raw[me->vref_ch]);

  // ---- 第2步: 三相电压重构 (默认: Vab=v_val[0], Vbc=v_val[1]) -----------
  if (me->num_v >= 2) {
    adc_ac_sampler_reconstruct_v(me, 0, 1);
  }

  // ---- 第3步: 三相电流重构 (逆变侧: Ia=i_val[0], Ic=i_val[1]) ------------
  if (me->num_i >= 2) {
    adc_ac_sampler_reconstruct_i(me, 0, 1,
                                 &me->ia1, &me->ib1, &me->ic1);
  }

  // 整流侧 (若有 ≥4 路电流: Ia=i_val[2], Ic=i_val[3])
  if (me->num_i >= 4) {
    adc_ac_sampler_reconstruct_i(me, 2, 3,
                                 &me->ia2, &me->ib2, &me->ic2);
  }

  // ---- 第4步: DC 母线电压估算 (线电压峰值低通滤波) -----------------------
  if (me->num_v >= 2) {
    float vab = me->v_val[0];
    float vbc = me->v_val[1];
    float vline_peak = sqrtf(vab * vab + vbc * vbc) * 0.8165f; // ≈ sqrt(2/3)
    // 一阶低通: alpha = 2*PI*fc*Ts = 2*PI*10Hz*25us ≈ 0.00157
    me->vdc += 0.00157f * (vline_peak * 1.414f - me->vdc);
  }

  // ---- 第5步: RMS 累加 (800 样本 = 20ms @ 40kHz) -------------------------
  if (me->num_v >= 2) {
    float v_rms_inst = sqrtf((me->va * me->va + me->vb * me->vb +
                              me->vc * me->vc) / 3.0f);
    vrms_buf[rms_idx] = v_rms_inst;
  }

  if (me->num_i >= 2) {
    float i_rms_inst = sqrtf((me->ia1 * me->ia1 + me->ib1 * me->ib1 +
                              me->ic1 * me->ic1) / 3.0f);
    irms_buf[rms_idx] = i_rms_inst;
  }

  rms_idx = (rms_idx + 1) % RMS_BUF_SIZE;

  // CMSIS-DSP 计算 800 点 RMS
  float rms;
  arm_rms_f32(vrms_buf, RMS_BUF_SIZE, &rms);
  // 相电压 RMS → 线电压 RMS = sqrt(3) * 相电压 RMS
  me->vrms = rms * 1.732f;

  arm_rms_f32(irms_buf, RMS_BUF_SIZE, &rms);
  me->irms1 = rms;

  me->data_ready = true;
}

// ====== ADC 虚函数表 =========================================================

static const AdcOps sampler_ops = {
  .start_dma  = start_dma_impl,
  .read_ch    = read_ch_impl,
  .process    = process_impl,
  .get_sum2   = NULL,
  .get_ch_bin = NULL,
};

// ====== 构造器 ===============================================================

void adc_ac_sampler_init(AdcAcSampler *me, IoCompletion completion, ADC_HandleTypeDef *hadc,
                         DMA_HandleTypeDef *hdma,
                         uint8_t num_ch, uint8_t num_v, uint8_t num_i,
                         const uint8_t *i_ch, const uint8_t *v_ch,
                         uint8_t vref_ch) {
  assert(num_ch >= 1 && num_ch <= ADC_AC_MAX_CH);
  assert(num_v <= ADC_AC_MAX_V);
  assert(num_i <= ADC_AC_MAX_I);
  assert(num_ch >= num_v + num_i + 1);  // 总通道 ≥ V+I+参考

  // 调用基类构造器
  adc_base_init(&me->base);
  me->base.name    = AdcAcSamplerSensor;
  me->base.raw     = me->raw_buf;
  me->base.raw_cap = ADC_AC_MAX_CH;
  me->base.ops     = &sampler_ops;

  // 绑定 HAL 句柄
  me->hadc   = hadc;
  me->hdma   = hdma;
  me->num_ch = num_ch;
  me->num_v  = num_v;
  me->num_i  = num_i;

  // 完成契约: 转换完成→置 data_ready 标志, 消费侧轮询 — 只支持 IO_ASYNC_FLAG (声明其他值 = 违约)
  me->completion = completion;
  assert(me->completion == IO_ASYNC_FLAG);

  // 通道索引 (用户传入, NULL 则自动分配: 先电流再电压最后参考)
  if (i_ch) {
    memcpy(me->i_ch, i_ch, num_i);
  } else {
    for (uint8_t i = 0; i < num_i; i++) me->i_ch[i] = i;
  }

  if (v_ch) {
    memcpy(me->v_ch, v_ch, num_v);
  } else {
    for (uint8_t i = 0; i < num_v; i++) me->v_ch[i] = num_i + i;
  }

  me->vref_ch = vref_ch;

  // 默认校准参数 (init 后可手动覆盖各通道)
  for (uint8_t i = 0; i < num_v; i++) {
    me->v_gain[i]   = vsensor_gain_default;
    me->v_offset[i] = 0.0f;
    me->v_val[i]    = 0.0f;
  }
  for (uint8_t i = num_v; i < ADC_AC_MAX_V; i++) {
    me->v_gain[i]   = 0.0f;
    me->v_offset[i] = 0.0f;
    me->v_val[i]    = 0.0f;
  }

  for (uint8_t i = 0; i < num_i; i++) {
    me->i_gain[i]   = isensor_gain_default;
    me->i_offset[i] = 0.0f;
    me->i_val[i]    = 0.0f;
  }
  for (uint8_t i = num_i; i < ADC_AC_MAX_I; i++) {
    me->i_gain[i]   = 0.0f;
    me->i_offset[i] = 0.0f;
    me->i_val[i]    = 0.0f;
  }

  // 初始化处理后数据
  me->va   = 0; me->vb   = 0; me->vc   = 0;
  me->ia1  = 0; me->ib1  = 0; me->ic1  = 0;
  me->ia2  = 0; me->ib2  = 0; me->ic2  = 0;
  me->vdc  = 48.0f;   // 初始假设 48V 母线
  me->vrms = 0;
  me->irms1 = 0;
  me->vref_measured = 0.0f;
  me->data_ready = false;
}

// ====== 数据获取 =============================================================

// ADC 转换完成回调 — 在 HAL_ADC_ConvCpltCallback 中调用
void adc_ac_sampler_fetch(AdcAcSampler *me) {
  // 契约: 生产者置 data_ready 标志 (IO_ASYNC_FLAG) — init 声明不符即配置错误不可静默
  assert(me->completion == IO_ASYNC_FLAG);
  adc_process(&me->base);
}

// ISR 安全瞬时读取 — 仅标量浮点, 无 arm_rms_f32/sqrtf
void adc_ac_sampler_fast_fetch(AdcAcSampler *me) {
  AdcBase *base = &me->base;

  // 工程量转换 (所有通道, 标量浮点)
  for (uint8_t i = 0; i < me->num_v; i++) {
    me->v_val[i] = diff_to_eng(base->raw[me->v_ch[i]], me->v_gain[i]);
  }
  for (uint8_t i = 0; i < me->num_i; i++) {
    me->i_val[i] = diff_to_eng(base->raw[me->i_ch[i]], me->i_gain[i]);
  }

  // 三相电压重构 (Vab=v_val[0], Vbc=v_val[1])
  if (me->num_v >= 2) {
    float vab = me->v_val[0];
    float vbc = me->v_val[1];
    me->va = ( 2.0f * vab + vbc) / 3.0f;
    me->vb = (-vab + vbc) / 3.0f;
    me->vc = (-vab - 2.0f * vbc) / 3.0f;
  }

  // 三相电流重构 (逆变侧: i_val[0]=Ia, i_val[1]=Ic)
  if (me->num_i >= 2) {
    float ia = me->i_val[0];
    float ic = me->i_val[1];
    me->ia1 =  ia;
    me->ib1 = -(ia + ic);
    me->ic1 =  ic;
  }

  // 整流侧 (i_val[2]=Ia, i_val[3]=Ic)
  if (me->num_i >= 4) {
    float ia = me->i_val[2];
    float ic = me->i_val[3];
    me->ia2 =  ia;
    me->ib2 = -(ia + ic);
    me->ic2 =  ic;
  }
}

// ====== 查询接口 =============================================================

// 反初始化: 停止 DMA、清空 HAL 句柄、清空 ops
void adc_ac_sampler_deinit(AdcAcSampler *me) {
  if (me->hadc != NULL) {
    HAL_ADC_Stop_DMA(me->hadc);
  }
  me->hadc = NULL;
  me->hdma = NULL;
  adc_base_deinit(&me->base);
}

float adc_ac_sampler_get_vrms(const AdcAcSampler *me) { return me->vrms; }
float adc_ac_sampler_get_irms(const AdcAcSampler *me) { return me->irms1; }
float adc_ac_sampler_get_vdc(const AdcAcSampler *me)  { return me->vdc; }
float adc_ac_sampler_get_freq(const AdcAcSampler *me) { (void)me; return 50.0f; }
float adc_ac_sampler_get_vref(const AdcAcSampler *me) { return me->vref_measured; }

float adc_ac_sampler_get_v(const AdcAcSampler *me, uint8_t idx) {
  if (idx >= me->num_v) return 0.0f;
  return me->v_val[idx];
}

float adc_ac_sampler_get_i(const AdcAcSampler *me, uint8_t idx) {
  if (idx >= me->num_i) return 0.0f;
  return me->i_val[idx];
}
