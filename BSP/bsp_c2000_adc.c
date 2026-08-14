// BSP ADC C2000 后端 — bsp_adc.h 的 TMS320F28004x 实现 (driverlib)
//
// 语义映射: C2000 无循环 DMA — ePWM 触发的 SOC 每周期把结果写进 ADCRESULT 寄存器.
// "DMA 缓冲"由 ADC INT ISR 承担: 转换完成 → 拷贝 ADCRESULT[0..num_ch-1] → 已注册目标块.
// start_dma/restart_dma 注册目标块 (PingPong 非活动块); 上层 ePWM ISR 在周期首部调
// adc_dc_sampler_on_dma_complete 标 pending + restart 换块 → fetch 切快照. 与 STM32 的
// HAL_ADC_ConvCpltCallback 拆分语义一致: 数据落地 ISR 只做交接 (拷贝+重臂), 状态机归上层.
//
// 前提: SysConfig board.c (或项目 setup) 已完成 ADC VREF/使能/SOC 触发配置, ePWM 已启动.
// 本层只做中断注册 + 结果拷贝 (硬件 "DMA" 等价物), 不碰 GPIO/SOC — 与 bsp_c2000_epwm.c 同约定.
// 假设 SOC0..SOC(n-1) 由同一 ePWM 触发同时开始, SOC0 优先转换, SOC(n-1) 最后完成 → 中断取最后一路.
//
// 约束: 单采样器 (单 ADC 模块). s_base/s_target/s_num_ch 为模块级单例 + 单 ISR +
// 一次性注册守卫 → 仅支持一个活动 ADC 采样器. 当前拓扑 (buck) 单 ADC 满足; 多 ADC
// (ADCA+ADCB 并行采样) 需改为按 PIE 向量索引的实例状态表再扩展.

#include "bsp_adc.h"
#include <stdbool.h>
#include "driverlib.h"

// F28004x 每 ADC 模块 16 个 SOC (SOC0..SOC15); 超过则 ADC_SOCNumber 越界
#define C2000_ADC_MAX_CH 16u

// ======== 模块级状态 ========

static uint32_t s_base = 0u;           // ADC 基址 (hadc → base, ISR 取结果基址)
static uint16_t *volatile s_target;    // 当前拷贝目标 (PingPong 非活动块), ISR 读
static volatile uint16_t s_num_ch;     // 扫描通道数
static volatile bool s_isr_registered; // Interrupt_register 只做一次

// ======== 内部辅助 ========

// ADC 基址 → 该模块 ADC INT1 向量 (F28004x: ADCA/ADCB/ADCC 各一 INT1)
static uint32_t adc_int(uint32_t base) {
  if (base == ADCA_BASE)
    return INT_ADCA1;
  if (base == ADCB_BASE)
    return INT_ADCB1;
  if (base == ADCC_BASE)
    return INT_ADCC1;
  return 0u;
}

// ADC 基址 → 结果寄存器基址 (ADCRESULT 独立地址段, ADC_readResult 用)
static uint32_t adc_result_base(uint32_t base) {
  if (base == ADCA_BASE)
    return ADCARESULT_BASE;
  if (base == ADCB_BASE)
    return ADCBRESULT_BASE;
  if (base == ADCC_BASE)
    return ADCCRESULT_BASE;
  return 0u;
}

// ======== ADC INT ISR — 硬件 "DMA": 拷贝本次扫描结果到目标块 ========
// 交接职责 (rt-execution-design.md §5): 只拷贝快照 + 清中断重臂, 不做业务.
// 上层 ePWM ISR 随后 adc_dc_sampler_on_dma_complete 标 pending + restart 换块.
__interrupt void bsp_c2000_adc_isr(void) {
  if (!s_target || s_num_ch == 0u) {
    ADC_clearInterruptStatus(s_base, ADC_INT_NUMBER1);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
    return;
  }
  uint32_t rb = adc_result_base(s_base);
  for (uint16_t i = 0u; i < s_num_ch; i++)
    s_target[i] = ADC_readResult(rb, (ADC_SOCNumber) i);
  // 重臂: 清主中断 + 溢出 (EOC 丢失 = 转换率 > ISR 处理率, 控制周期内 ISR 应远快于周期)
  ADC_clearInterruptStatus(s_base, ADC_INT_NUMBER1);
  if (ADC_getInterruptOverflowStatus(s_base, ADC_INT_NUMBER1)) {
    ADC_clearInterruptOverflowStatus(s_base, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(s_base, ADC_INT_NUMBER1);
  }
  // PIEACK[1] 必须清 (末行): Interrupt_register 直写 PIE 向量表, 无 dispatcher 代劳清 ACK;
  // 不清则 PIE 组 1 锁存, ADC INT1 只触发一次 → 采样缓冲冻结 (SDK adc_ex10 等全清末行 ACK)
  Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

// ======== BSP 接口 ========

void bsp_adc_calibrate(void *hadc, BspAdcMode mode) {
  (void) mode;  // F28004x 单端 ADC, 校准流程与模式无关
  if (!hadc)
    return;
  // F28004x 仅有偏移校准 (OTP 校准数据加载进 trim 寄存器), 无增益校准寄存器
  ADC_setOffsetTrim((uint32_t) hadc);
}

void bsp_adc_start_dma(void *hadc, void *hdma, uint16_t *buf, int num_ch) {
  (void) hdma;  // C2000 无 DMA 对象: ePWM 触发单转换源, ADC INT ISR 拷结果
  if (!hadc || !buf || num_ch <= 0 || num_ch > C2000_ADC_MAX_CH)
    return;
  uint32_t vec = adc_int((uint32_t) hadc);
  if (!vec)
    return;
  s_base = (uint32_t) hadc;
  s_target = buf;
  s_num_ch = (uint16_t) num_ch;

  if (!s_isr_registered) {
    Interrupt_register(vec, &bsp_c2000_adc_isr);
    s_isr_registered = true;
  }

  // 中断在最后一路 SOC 完成时触发 (单 ePWM 触发同时开始, SOC0 优先转换 → SOC(n-1) 最后完)
  ADC_setInterruptSource(s_base, ADC_INT_NUMBER1, (ADC_SOCNumber) (num_ch - 1));
  ADC_setInterruptPulseMode(s_base, ADC_PULSE_END_OF_CONV);
  ADC_clearInterruptStatus(s_base, ADC_INT_NUMBER1);
  ADC_clearInterruptOverflowStatus(s_base, ADC_INT_NUMBER1);
  ADC_enableInterrupt(s_base, ADC_INT_NUMBER1);
  Interrupt_enable(vec);
}

void bsp_adc_stop_dma(void *hadc, void *hdma) {
  (void) hdma;
  uint32_t base = (uint32_t) hadc;  // 以调用方 hadc 为准, 不依赖 s_base
  uint32_t vec = base ? adc_int(base) : 0u;
  if (!vec)
    return;
  ADC_disableInterrupt(base, ADC_INT_NUMBER1);
  Interrupt_disable(vec);
  ADC_clearInterruptStatus(base, ADC_INT_NUMBER1);
}

// PingPong 换块: 重注册拷贝目标 (下轮 ADC INT 写入新非活动块), 中断保持使能
void bsp_adc_restart_dma(void *hadc, void *hdma, uint16_t *buf, int num_ch) {
  (void) hdma;
  if (!hadc || !buf || num_ch <= 0 || num_ch > C2000_ADC_MAX_CH)
    return;
  uint32_t vec = adc_int((uint32_t) hadc);
  if (!vec)
    return;
  s_base = (uint32_t) hadc;
  s_target = buf;
  s_num_ch = (uint16_t) num_ch;
  ADC_clearInterruptStatus(s_base, ADC_INT_NUMBER1);
  ADC_enableInterrupt(s_base, ADC_INT_NUMBER1);
  Interrupt_enable(vec);
}
