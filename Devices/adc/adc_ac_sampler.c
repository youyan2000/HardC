// 三相交流采样器实现 —— AdcBase 子类
//
// 参考 drv_adc_sampler.c
//
// 实现 AdcOps 虚函数表:
//   .start_dma → start_dma_impl (启动 ADC DMA 循环扫描)
//   .read_ch   → read_ch_impl   (读取通道 i 原始值)
//   .process   → process_impl   (工程量转换 + 三相重构 + RMS + Vdc)
//
// 通道数灵活配置:
//   - num_v 路电压 (差分, ≤4):  v_val[i] = diff_to_eng(raw[v_ch[i]], v_gain[i])
//   - num_i 路电流 (差分, ≤8):  i_val[i] = diff_to_eng(raw[i_ch[i]], i_gain[i])
//   - 1 路参考 (单端):          vref = se_to_voltage(raw[vref_ch])
//
// 三相重构 (process / fast_fetch 中调用):
//   - 电压: Vab, Vbc → Va, Vb, Vc
//   - 电流: Ia, Ic  → Ia, Ib, Ic  (KCL: Ib = -(Ia+Ic))
//
// 滑动窗块 RMS (adc_ac_rms_sample / adc_ac_rms_commit):
//   替代原 static 全局 vrms_buf/irms_buf + arm_rms_f32:
//     - 缓冲移入 AdcAcSampler 实例 (多实例/ISR 安全)
//     - 数学 = sqrt(Σ(最新 N 个样本²) / N), 与 arm_rms_f32 等价
//     - 纯 C (MATH_SQRT), 去 arm_math.h, 跨平台
//
// ISR 安全: fast_fetch 仅标量浮点运算 (无 RMS 聚合 / 长循环)
// 参考: RM0440 §21.4.6 差分模式 DIFSEL 寄存器

#include "adc_ac_sampler.h"
#include "container_of.h"
#include <string.h>      // memset
#include <assert.h>
#include "comp_math.h"   // MATH_SQRT — 平方/开方 (替代 arm_math.h)

// ====== 校准常量 (static const — 类型安全) =====================================
static const float adc_vref = 3.30f;     // ADC 参考电压 (V)
static const float adc_res  = 4096.0f;   // 12-bit 分辨率
static const float vsensor_gain_default  = 15.15f;  // 电压传感器分压比
static const float isensor_gain_default  = 9.60f;   // 电流传感器增益 (A/V)

// ====== 工具函数 =============================================================

// 差分通道: int16_t → 工程量
// raw = ADC 差分结果 (IN_P - IN_N), -4095 ~ +4095 对应 ±3.3V
static float diff_to_eng(int16_t raw, float gain) {
  float v = (float)raw * adc_vref / adc_res;  // 有符号电压 (±3.3V)
  return v * gain;                              // → 工程量 (±A 或 ±V)
}

// 单端通道: 原始值 → 电压
static float se_to_voltage(int16_t raw) {
  return (float)((uint16_t)raw) * adc_vref / adc_res;
}

// ====== 滑动窗块 RMS (纯 C, per-ring window; 替代 arm_rms_f32 + static 缓冲) ======
// 每调 sample 一次喂入一个新样本 (瞬时 RMS)。window 由调用方传入 (指向 me->v_rms 或 me->i_rms)，
// 电压/电流各自独立窗口 — 互不污染。等价原实现 arm_rms_f32 = sqrt(Σx²/N)。

// 喂入一个瞬时值 (样本平方), 维护窗口内 Σ平方 与环形缓冲, 返回当前窗口 RMS
static float adc_ac_rms_sample(RmsWindow *w, float inst, uint16_t win) {
  float sq = inst * inst;
  // 覆盖最旧: 若窗口已满, 减去最旧平方; w->ring[w->idx] 存 (写入前) 的最旧平方
  if (w->count >= win) {
    w->sum_sq -= w->ring[w->idx];
  }
  w->ring[w->idx] = sq;
  w->sum_sq      += sq;
  w->idx = (w->idx + 1u) % win;
  if (w->count < win) {
    w->count++;
  }
  // RMS = sqrt(Σx² / N)
  if (w->count > 0u) {
    return MATH_SQRT(w->sum_sq / (float)w->count);
  }
  return 0.0f;
}

// ====== ops 实现 (static — 封装) =============================================

// 启动 ADC DMA 循环扫描
static void start_dma_impl(AdcBase *base) {
  AdcAcSampler *me = container_of(base, AdcAcSampler, base);
  // PingPong: 从非活动块起步 (DMA 总写非活动块, FAST 只读活动块; init 后 active=0 → 写 buf[1])
  // 统一走 BSP 抽象 (STM32: HAL_ADC_Start_DMA; C2000: 触发源 + 缓冲注册) — 跨平台
  bsp_adc_start_dma(me->hadc, me->hdma, (uint16_t *) double_buffer_pending(&me->dbuf), me->num_ch);
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

// === 传感器数据处理 (主循环调用, 含 RMS 聚合) ================================

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
    // 三相平衡: 母线电压 ≈ 线电压瞬时合成幅值 (两相 120° 差)
    //   |Vab + Vbc∠120°| = sqrt(Vab² + Vbc² + Vab·Vbc) — 正确三相合成, 非 90° 假设
    float vline_inst = MATH_SQRT(vab * vab + vbc * vbc + vab * vbc);
    // 一阶低通平滑: alpha = 2π·fc·Ts (fc=10Hz, Ts=1/FAST_FREQ)
    const float vdc_lp_alpha = 0.00157f;  // 10Hz 低通 @ ~10kHz 采样
    me->vdc += vdc_lp_alpha * (vline_inst - me->vdc);
  }

  // ---- 第5步: 滑动窗 RMS (window = ADC_AC_RMS_WIN) -----------------------
  // 线电压 RMS (逐点喂入 → 窗口 RMS); 相→线 sqrt(3); V/I 各用独立 RmsWindow
  if (me->num_v >= 2) {
    float v_rms_inst = MATH_SQRT((me->va * me->va + me->vb * me->vb +
                                  me->vc * me->vc) / 3.0f);
    me->vrms = adc_ac_rms_sample(&me->v_rms, v_rms_inst, ADC_AC_RMS_WIN) * 1.732f;
  }

  // 逆变侧线电流 RMS (窗口)
  if (me->num_i >= 2) {
    float i_rms_inst = MATH_SQRT((me->ia1 * me->ia1 + me->ib1 * me->ib1 +
                                  me->ic1 * me->ic1) / 3.0f);
    me->irms1 = adc_ac_rms_sample(&me->i_rms, i_rms_inst, ADC_AC_RMS_WIN);
  }

  // A11: 屏障 — 确保上述 raw 派生写入在 data_ready 置位前完成 (编译器不得重排到置位之后)
  ADC_AC_MEM_BARRIER();
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

void adc_ac_sampler_init(AdcAcSampler *me, IoCompletion completion, BspAdcHandle *hadc,
                         BspAdcHandle *hdma,
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
  // PingPong 双缓冲 (A7): 双倍 raw_buf 对半切分, 两块各 num_ch 个 16bit 采样.
  // base.raw 绑活动块 (FAST/fetch 读); DMA 写非活动块, 完成回调标 pending 后由 fetch 切换
  double_buffer_init(&me->dbuf, me->raw_buf, (uint16_t) (2u * ADC_AC_MAX_CH * sizeof(uint16_t)));
  me->base.raw     = (uint16_t *) double_buffer_active(&me->dbuf);
  me->base.raw_cap = ADC_AC_MAX_CH;
  me->base.ops     = &sampler_ops;

  // 绑定 BSP 句柄
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

  // 默认校准参数 (init 后可手动覆盖各通道; 工程可用 YmaC 注入覆盖 v_gain/i_gain)
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
  me->freq_hz = 0.0f;
  me->freq_timebase_hz = 10000.0f;  // fast_fetch 调用频率缺省 10kHz (工程按实际控制频率覆盖)
  me->freq_prev_va = 0.0f;
  me->freq_period_ticks = 0u;
  me->freq_last_period = 0u;
  me->freq_cross_cnt = 0u;
  me->data_ready = false;

  // 初始化滑动窗 RMS 状态 (per-ring: V/I 各自独立, 零初始化)
  me->v_rms.sum_sq = 0.0f;
  me->v_rms.idx    = 0u;
  me->v_rms.count  = 0u;
  me->i_rms.sum_sq = 0.0f;
  me->i_rms.idx    = 0u;
  me->i_rms.count  = 0u;
  memset(me->v_rms.ring, 0, sizeof(me->v_rms.ring));
  memset(me->i_rms.ring, 0, sizeof(me->i_rms.ring));
}

// ====== 数据获取 =============================================================

// ADC 转换完成回调 — 在 HAL_ADC_ConvCpltCallback (STM32 DMA 完成) / ePWM ISR (C2000) 中调用
// PingPong 快照交接 (A7, 与 AdcDcSampler 同款语义): DMA 刚写满非活动块 →
//   ① 重装目标在 enable 之前捕获 (完成时活动块 = 下轮写目标; 防 STM32 上 FAST 抢占
//      切快照后重装到刚写满/正在读的块 — A4 同款原子性) ② 标 pending → 切快照
//   ③ 重装 DMA 到捕获块 ④ process (工程量转换 + 三相重构 + RMS)
void adc_ac_sampler_fetch(AdcAcSampler *me) {
  // 契约: 生产者置 data_ready 标志 (IO_ASYNC_FLAG) — init 声明不符即配置错误不可静默
  assert(me->completion == IO_ASYNC_FLAG);
  uint8_t *next = double_buffer_active(&me->dbuf);
  double_buffer_enable_pending(&me->dbuf);
  double_buffer_switch(&me->dbuf);
  me->base.raw = (uint16_t *) double_buffer_active(&me->dbuf);
  bsp_adc_restart_dma(me->hadc, me->hdma, (uint16_t *) next, me->num_ch);
  adc_process(&me->base);
}

// ISR 安全瞬时读取 — 仅标量浮点, 无 RMS 聚合/长循环
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

  // 频率过零检测 (真测): Va 正过零计数, 2 次正过零 = 1 完整周期 → freq = ticks/秒
  //   带死区 (±ADC_AC_FREQ_HYST_V) 抑制 0 附近噪声抖动产生伪过零; 频率未知时返回 0 (不造假数据)
  if (me->num_v >= 2) {
    me->freq_period_ticks++;
    if (me->freq_prev_va <= ADC_AC_FREQ_HYST_V && me->va > ADC_AC_FREQ_HYST_V) {
      me->freq_cross_cnt++;
      if (me->freq_cross_cnt >= 2u) {
        // 两个连续正过零 = 一个完整周期; freq = 1 / (period_ticks × dt)
        if (me->freq_last_period > 0u) {
          float dt = 1.0f / (me->freq_timebase_hz > 0.0f ? me->freq_timebase_hz : 10000.0f);  // fast_fetch 调用频率 (init 注入)
          me->freq_hz = 1.0f / ((float) me->freq_last_period * dt);
        }
        me->freq_last_period = me->freq_period_ticks;
        me->freq_cross_cnt = 0u;
      }
      me->freq_period_ticks = 0u;
    }
    me->freq_prev_va = me->va;
  }
}

// ====== 查询接口 =============================================================

// 反初始化: 停止 DMA、清空 BSP 句柄、清空 ops
void adc_ac_sampler_deinit(AdcAcSampler *me) {
  if (me->hadc != NULL) {
    bsp_adc_stop_dma(me->hadc, me->hdma);
  }
  me->hadc = NULL;
  me->hdma = NULL;
  adc_base_deinit(&me->base);
}

float adc_ac_sampler_get_vrms(const AdcAcSampler *me) { return me->vrms; }
float adc_ac_sampler_get_irms(const AdcAcSampler *me) { return me->irms1; }
float adc_ac_sampler_get_vdc(const AdcAcSampler *me)  { return me->vdc; }
float adc_ac_sampler_get_freq(const AdcAcSampler *me) { return me->freq_hz; }
float adc_ac_sampler_get_vref(const AdcAcSampler *me) { return me->vref_measured; }

float adc_ac_sampler_get_v(const AdcAcSampler *me, uint8_t idx) {
  if (idx >= me->num_v) return 0.0f;
  return me->v_val[idx];
}

float adc_ac_sampler_get_i(const AdcAcSampler *me, uint8_t idx) {
  if (idx >= me->num_i) return 0.0f;
  return me->i_val[idx];
}
