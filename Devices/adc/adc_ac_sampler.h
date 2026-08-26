// 三相交流采样驱动 —— AdcBase 的子类
//
// 参考 STM32G474 配置: ADC1, TIM1_TRGO 触发, DMA 循环
//
// 继承关系:
//   AdcBase  <—  AdcAcSampler (本文件)
//
// 通道配置 (灵活映射, 不写死):
//   电压通道 (v_ch[], 差分): 最多 4 路 — 典型: Vab, Vbc, Vab_rec, Vbc_rec
//   电流通道 (i_ch[], 差分): 最多 8 路 — 典型: Ia_inv,Ib_inv,Ic_inv, Ia_rec,Ib_rec,Ic_rec + 2 spare
//   参考通道 (vref_ch, 单端): 1 路  — 典型: Vref_1V65
//
//   典型 7ch 配置:
//     i_ch[0..3] = {0,1,2,3}  → Ia_inv, Ic_inv, Ia_rec, Ic_rec
//     v_ch[0..1] = {4,5}      → Vab, Vbc
//     vref_ch    = 6          → Vref_1V65
//
//   扩展 13ch 配置 (双逆变器+双整流器全监测):
//     i_ch[0..7] = {0..7}     → 8 路电流 (两组三相+两路预留)
//     v_ch[0..3] = {8..11}    → 4 路电压
//     vref_ch    = 12         → Vref_1V65
//
// 差分模式关键:
//   - ADC 结果为有符号 int16_t: -4095 ~ +4095 对应 ±3.3V
//   - 不用减 1.65V 偏置！差分对自动抵消共模
//
// 数据处理 (process):
//   1. 原始 ADC → 工程量: v_val[i] = diff_to_eng(raw[v_ch[i]], v_gain[i], v_offset[i])
//                         i_val[i] = diff_to_eng(raw[i_ch[i]], i_gain[i], i_offset[i])
//                         差分电压 × gain + offset (差分共模已抵消, 默认 offset=0; G6 参数化)
//   2. 三相电压重构: Vab,Vbc → Va,Vb,Vc (线电压→相电压)
//   3. 三相电流重构: Ia,Ic → Ia,Ib,Ic (KCL: Ib=-(Ia+Ic))
//   4. RMS 计算 (纯 C 滑动窗块 RMS, 见 adc_ac_rms; 原 arm_rms_f32 已替换, 跨平台)
//   5. DC 母线电压估算 (线电压峰值低通滤波)
//
// 跨平台: hadc/hdma 为 BSP 不透明句柄 (BspAdcHandle), 不直接依赖 STM32 HAL/DMA。
// 适用场景: 电力电子三相逆变器/整流器交流电压电流采样
// 直流采样请使用 AdcDcSampler (adc_dc_sampler.h)

#ifndef ADC_AC_SAMPLER_H
#define ADC_AC_SAMPLER_H

#include "comp_adc.h"
#include "comp_io.h"          // 运行时契约: I/O 完成方式 (转换完成置 data_ready = IO_ASYNC_FLAG)
#include "bsp_adc.h"          // BspAdcHandle 不透明句柄 — 跨平台 (STM32/C2000), 去除 HAL 硬依赖
#include "comp_double_buffer.h"  // 五原语之 PingPong: DMA→FAST 采样快照 (撕裂读消除, A7)
#include "comp_math.h"        // MATH_SQRT — 纯 C 块 RMS 回退 (替代 arm_math.h arm_rms_f32)
#include <stdbool.h>

#define ADC_AC_MAX_CH  16  // 交流采样最大通道数 (STM32 ADC1 上限)
#define ADC_AC_MAX_V   4   // 最多 4 路电压 (差分)
#define ADC_AC_MAX_I   8   // 最多 8 路电流 (差分)

// 滑动窗 RMS 窗口长度 (样本数)。原实现 arm_rms_f32 固定 800 (约 20ms @ 40kHz)。
// 移入实例后窗口可配置, 默认保持 800 以对齐原行为。每路各占 ADC_AC_RMS_WIN 个 float 环形缓冲。
#define ADC_AC_RMS_WIN 800

// 过零检测死区 (V) — Va 须越过 ±此值才计一次正过零, 抑制 0 附近噪声抖动产生伪过零
// (工程可按实际噪声覆盖; 缺省 0.1V — 对电网级相电压噪声免疫, 小信号采样可调小)
#ifndef ADC_AC_FREQ_HYST_V
#define ADC_AC_FREQ_HYST_V 0.1f
#endif

// ======== 编译器内存屏障 (A11) ========
// data_ready 是跨上下文标志 (ADC 完成回调置位, 消费侧轮询). C 语言 volatile 不保证
// 非 volatile 访问 (raw 写/读) 不被重排到 volatile 访问之后 — 需显式屏障保证:
//   生产侧: raw/工程量写入在 data_ready=1 之前完成 (process_impl 末尾调用)
//   消费侧: 看到 data_ready=1 后再读 raw (见下方 data_ready 字段注释)
//   GCC/Clang: 扩展 asm 内存屏障; TI CGT (C28x): asm 语句是调度屏障 (不跨语句重排)
#if defined(__GNUC__) || defined(__clang__)
#define ADC_AC_MEM_BARRIER() __asm volatile("" ::: "memory")
#elif defined(__TI_COMPILER_VERSION__)
#define ADC_AC_MEM_BARRIER() __asm(" nop")
#else
#define ADC_AC_MEM_BARRIER() ((void) 0)
#endif


// 单路滑动窗 RMS 状态 (per-ring 独立 — V / I 各一份, 互不污染)
typedef struct {
  float    sum_sq;               // 累计平方和 (窗口内)
  float    ring[ADC_AC_RMS_WIN]; // 最近 WIN 个样本平方的环形缓冲
  uint32_t idx;                  // 环形写指针
  uint16_t count;                // 已填充样本数 (≤ window)
} RmsWindow;

typedef struct {
  AdcBase            base;          // [首成员!] 基类
  uint16_t           raw_buf[2 * ADC_AC_MAX_CH]; // [PingPong] 双倍缓冲: 对半切分为两个快照块 (A7)
  DoubleBuffer       dbuf;          // PingPong 双缓冲状态 (active/pending 块翻转, 撕裂读消除)
  BspAdcHandle      *hadc;         // BSP ADC 句柄 (STM32: &hadc1; C2000: ADC 基址)
  BspAdcHandle      *hdma;         // BSP DMA/触发句柄 (STM32: &hdma_adc1; C2000: 触发源)
  IoCompletion       completion;    // 完成契约: 发起时声明完成方式 (本设备固定 IO_ASYNC_FLAG)

  // ---- 通道配置 (init 后可按 PCB 布局手动修改) ----
  uint8_t num_ch;                   // ADC 扫描通道总数 (如 7, 13)
  uint8_t num_v;                    // 实际电压通道数 (≤ ADC_AC_MAX_V)
  uint8_t num_i;                    // 实际电流通道数 (≤ ADC_AC_MAX_I)
  uint8_t i_ch[ADC_AC_MAX_I];       // 电流通道在 raw 中的索引
  uint8_t v_ch[ADC_AC_MAX_V];       // 电压通道在 raw 中的索引
  uint8_t vref_ch;                  // 参考电压通道在 raw 中的索引

  // ---- 校准参数 (每通道独立增益/偏置, init 后可改) ----
  float  v_gain[ADC_AC_MAX_V];      // 电压增益 (V/V)
  float  v_offset[ADC_AC_MAX_V];    // 电压偏置 (差分=0)
  float  i_gain[ADC_AC_MAX_I];      // 电流增益 (A/V)
  float  i_offset[ADC_AC_MAX_I];    // 电流偏置 (差分=0)

  // ---- 处理后的工程量 (process 每次更新) ----
  float  v_val[ADC_AC_MAX_V];       // 电压工程量 (V)
  float  i_val[ADC_AC_MAX_I];       // 电流工程量 (A)

  // ---- 三相重构结果 (从 v_val/i_val 子集推算) ----
  float  va, vb, vc;                // 逆变侧三相相电压 (V)
  float  ia1, ib1, ic1;             // 逆变侧三相电流 (A)
  float  ia2, ib2, ic2;             // 整流侧三相电流 (A)
  float  vdc;                       // DC 母线电压估算 (V)
  float  vrms;                      // 线电压 RMS (V)
  float  irms1;                     // 逆变侧线电流 RMS (A)

  // ---- 滑动窗 RMS 状态 (per-instance + per-ring, V/I 独立, 防互相污染) ----
  RmsWindow v_rms;   // 电压 RMS 窗口
  RmsWindow i_rms;   // 电流 RMS 窗口

  // ---- 诊断 ----
  float  vref_measured;             // 参考电压实测值 (V)

  // ---- 频率测量 (过零检测, fast_fetch 每周期推进; 真测, 不硬编码) ----
  float  freq_hz;                // 当前测得基波频率 (Hz, 0=尚未测得)
  float  freq_timebase_hz;       // fast_fetch 调用频率 (Hz) — 频率测量时间基准; init 缺省 10000, 工程按实际控制频率覆盖
  float  freq_prev_va;           // 上次 Va (过零检测用)
  uint32_t freq_period_ticks;    // 距上次正过零的 tick 数 (2 次正过零 = 1 完整周期)
  uint32_t freq_last_period;     // 上个完整周期的 tick 数 (freq 时间基准)
  uint8_t  freq_cross_cnt;       // 过零计数 (2 次=1 周期)

  // ---- 数据就绪标志 (ADC 完成回调置位) ----
  // 消费侧读取模式: if (me->data_ready) { ADC_AC_MEM_BARRIER(); ...读 raw... }
  // 生产侧: process_impl 末尾 ADC_AC_MEM_BARRIER() 后置 true (A11, 见 .c)
  volatile bool data_ready;
} AdcAcSampler;

// === API =====================================================================

// 初始化三相交流采样器
// completion: 完成契约 — 转换完成回调置 data_ready 标志 (IO_ASYNC_FLAG), 消费者轮询; 传其他值即契约违约
// hadc:   BSP ADC 句柄 (STM32: &hadc1, C2000: ADC 基址)
// hdma:   BSP DMA/触发句柄 (STM32: &hdma_adc1, C2000: NULL — 由 ePWM 触发)
// num_ch: ADC 扫描总通道数 (如 7 或 13)
// num_v:  电压通道数 (≤4)
// num_i:  电流通道数 (≤8)
// i_ch:   电流通道在 raw 中的索引 [num_i] (传 NULL 则默认 0,1,2...)
// v_ch:   电压通道在 raw 中的索引 [num_v] (传 NULL 则默认 num_i, num_i+1...)
// vref_ch: 参考电压通道索引 (如 6 或 12)
// v_gain / v_offset: 电压通道校准 [num_v] — 工程量 = 差分电压 × v_gain + v_offset (传 NULL 用默认: 15.15/0.0)
// i_gain / i_offset: 电流通道校准 [num_i] — 工程量 = 差分电压 × i_gain + i_offset (传 NULL 用默认: 9.60/0.0)
//   对齐 DC/Follower 与 YmaC ProjectConfig 注入 (G6 参数化); init 后仍可手工覆盖字段
void adc_ac_sampler_init(AdcAcSampler *me, IoCompletion completion, BspAdcHandle *hadc,
                         BspAdcHandle *hdma,
                         uint8_t num_ch, uint8_t num_v, uint8_t num_i,
                         const uint8_t *i_ch, const uint8_t *v_ch,
                         uint8_t vref_ch,
                         const float *v_gain, const float *v_offset,
                         const float *i_gain, const float *i_offset);

// 反初始化: 停止 DMA、清空 ops
void adc_ac_sampler_deinit(AdcAcSampler *me);

// ADC 转换完成回调 — 在 HAL_ADC_ConvCpltCallback (STM32 DMA 完成) / ePWM ISR (C2000) 中调用
// 契约: 生产者置 data_ready 标志 (IO_ASYNC_FLAG) — 与 init 声明不符即配置错误, 不可静默 (assert)
// DMA 刚写满非活动块 → 切快照 (base.raw 指向新活动块) + 重装 DMA 到完成时活动块, 再执行
// 工程量转换 + 三相重构 + RMS (process). FAST 的 fast_fetch 只读活动块 → DMA 从不写 FAST
// 正在读的块 (A7: 撕裂读消除, 与 AdcDcSampler 同款 PingPong 语义)
void adc_ac_sampler_fetch(AdcAcSampler *me);

// ISR 安全瞬时读取 — 仅 va/vb/vc + ia1/ib1/ic1 + 频率过零推进 (freq_hz 真测), 无 RMS/VDC
// 不做滑动窗 RMS 聚合 (ISR 中用 FPU/长循环 → 可能 HardFault)
void adc_ac_sampler_fast_fetch(AdcAcSampler *me);

// 三相电压重构: 从 v_val[idx_vab], v_val[idx_vbc] → va, vb, vc
// 调用者指定哪两路电压是线电压 (Vab, Vbc)
void adc_ac_sampler_reconstruct_v(AdcAcSampler *me,
                                  uint8_t idx_vab, uint8_t idx_vbc);

// 三相电流重构: 从 i_val[idx_ia], i_val[idx_ic] → ia, ib, ic (KCL)
// 调用者指定哪两路电流是 Ia, Ic
void adc_ac_sampler_reconstruct_i(AdcAcSampler *me,
                                  uint8_t idx_ia, uint8_t idx_ic,
                                  float *ia, float *ib, float *ic);

// 获取处理后的数据 (供 HMI/UART 调试使用)
float adc_ac_sampler_get_vrms(const AdcAcSampler *me);
float adc_ac_sampler_get_irms(const AdcAcSampler *me);
float adc_ac_sampler_get_vdc(const AdcAcSampler *me);
float adc_ac_sampler_get_freq(const AdcAcSampler *me);
float adc_ac_sampler_get_vref(const AdcAcSampler *me);

// 获取指定通道的工程量
float adc_ac_sampler_get_v(const AdcAcSampler *me, uint8_t idx);
float adc_ac_sampler_get_i(const AdcAcSampler *me, uint8_t idx);

#endif  // ADC_AC_SAMPLER_H
