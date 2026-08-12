#ifndef COMP_ADC_H
#define COMP_ADC_H

// ADC 平台层 —— 抽象基类
// 子类: 红外循迹传感器 (drv_adc_follower)

#include <stdint.h>

// ======== ADC 参数类型 (纯数据 POD, 供 CarConfig 组合) ========

typedef struct {
  int16_t thr[8];
} AdcFollowerParams;

// ADC 实例名
typedef enum {
  AdcFollowerSensor,  // 8 路红外循迹传感器
  AdcDcSamplerSensor, // 通用直流采样器 (1~8ch, k*adc+b + LPF)
  AdcAcSamplerSensor, // 三相交流采样器 (6 差分 + 1 单端 + 三相重构)
} AdcName;

typedef struct AdcBase AdcBase;

// 虚函数指针类型
typedef void     (*adc_dma_fn)(AdcBase *me);          // 启动 DMA 多通道扫描
typedef uint16_t (*adc_ch_fn)(AdcBase *me, int i);    // 读取通道 i 的原始值
typedef void     (*adc_proc_fn)(AdcBase *me);         // 处理传感器 (二值化+位置)
typedef int16_t  (*adc_sum2_fn)(AdcBase *me);         // [可选] 触发传感器计数
typedef int16_t  (*adc_bin_fn)(AdcBase *me, int i);   // [可选] 通道 i 二值化结果

// 虚函数表 (ops)
// 注: get_sum2 和 get_ch_bin 僅對循跡傳感器有意義, 電壓電流採樣器可設 NULL
typedef struct {
  adc_dma_fn  start_dma;  // [必须] 启动 DMA 循环扫描
  adc_ch_fn   read_ch;    // [必须] 读取通道 i 原始值
  adc_proc_fn process;    // [必须] 传感器数据处理
  adc_sum2_fn get_sum2;   // [可选] 触发传感器计数 (popcount of ch_bin)
  adc_bin_fn  get_ch_bin; // [可选] 通道 i 二值化结果 (0/1)
} AdcOps;

// 基类结构体
// raw 和 raw_cap 由子类 init 时绑定到子类自己的 raw_buf[N]
// 通道数不再由基类限制 —— 每个子类自定
struct AdcBase {
  const AdcOps *ops;     // 指向子类实现的虚函数表
  AdcName       name;    // 实例名
  uint16_t     *raw;     // DMA 缓冲区指针 (子类 init 时绑到 raw_buf)
  uint8_t       raw_cap; // 缓冲区容量 (通道数上限, 子类自定)
  int16_t       pos;     // 位置偏差 (-7 ~ +7)
};

// 基类构造 / 析构
void adc_base_init  (AdcBase *me);
void adc_base_deinit(AdcBase *me);

// 分发函数 —— 通过 ops 调用子类实现
void     adc_start_dma(AdcBase *me);
uint16_t adc_read_ch(AdcBase *me, int i);
void     adc_process(AdcBase *me);
int16_t  adc_get_sum2(AdcBase *me);
int16_t  adc_get_ch_bin(AdcBase *me, int i);

#endif
