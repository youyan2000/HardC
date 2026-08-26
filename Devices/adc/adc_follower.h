// 红外循迹传感器驱动 —— AdcBase 的子类
// 使用 ADC1 + DMA1 通道 1, 8 路扫描 (PA0-PA7)
// TIM3 TRGO 触发，DMA 循环写入 AdcBase.raw[8]

#ifndef ADC_FOLLOWER_H
#define ADC_FOLLOWER_H

#include "comp_adc.h"
#include "bsp_adc.h"               // BspAdcHandle 不透明句柄 — 跨平台 (STM32/C2000), 去除 HAL 硬依赖
#include "comp_double_buffer.h"    // 五原语之 PingPong: DMA→FAST 采样快照 (撕裂读消除, 对齐 DC/AC)
#include "comp_io.h"               // 运行时契约: I/O 完成方式 (DMA 完成 = IO_ASYNC_FLAG)

typedef struct {
  AdcBase            base;        // 基类
  uint16_t           raw_buf[2 * 8]; // [PingPong] 双倍缓冲: 对半切分为两个快照块 (8 通道)
  DoubleBuffer       dbuf;        // PingPong 双缓冲状态 (active/pending 块翻转, 撕裂读消除)
  BspAdcHandle      *hadc;        // BSP ADC 句柄 (STM32: &hadc1; C2000: ADC 基址)
  BspAdcHandle      *hdma;        // BSP DMA/触发句柄 (STM32: &hdma_adc1; C2000: 触发源)
  IoCompletion       completion;  // 完成契约: 发起时声明完成方式 (本设备固定 IO_ASYNC_FLAG)
  int16_t            threshold[8]; // 各通道门限值
  int16_t            ch_bin[8];   // 二值化结果 (0/1)
  int16_t            ch_val[8];   // 独热码值 (0 或 1<<i)
  int16_t            sum;         // 独热码之和
  int16_t            sum2;        // 超过门限的通道数
  int16_t            cal_white[8]; // 校准: 白线 ADC
  int16_t            cal_black[8]; // 校准: 黑线 ADC
  int8_t             polarity[8]; // 传感器极性: 1=黑低白高, -1=黑高白低
  uint8_t            cal_step;    // 校准步骤: 0=正常, 1=白已采, 2=黑追踪中
  uint8_t            cal_send;    // 主循环发送标志: 1=发白值, 2=发校准结果
  uint8_t            cal_pending; // ISR: EE 前缀已收到, 下一字节为校准子命令
  uint8_t            sns_pending; // ISR: FC 前缀已收到, 下一字节为传感器子命令
  uint8_t            sns_send;    // ISR→主循环: 传感器查询发送 (1=ADC,2=超声,3=MPU)
} AdcFollower;

// 初始化循迹传感器 — completion: 完成契约 (本设备 DMA 完成→置 pending, 消费者 fetch 轮询; 固定 IO_ASYNC_FLAG)
// hadc/hdma: BSP 句柄; threshold: 8 路门限数组
void adc_follower_init(AdcFollower *me, IoCompletion completion, BspAdcHandle *hadc, BspAdcHandle *hdma,
                       const int16_t *threshold);
void adc_follower_deinit(AdcFollower *me);

// ADC DMA 完成回调 — 生产侧 (PingPong 快照交接), 在 HAL_ADC_ConvCpltCallback 中转发
// 与 DC/AC 同款语义: 标 pending + 重装到完成时活动块 (下轮写目标); A4 原子性: 重装目标 enable 前捕获
void adc_follower_on_dma_complete(AdcFollower *me);

// FAST 消费侧 — 每控制周期调用: 有 pending 则切快照, 然后 process (二值化 + 位置偏差 pos)
// mod_follower 每周期读 me->adc->pos (本函数是 pos 的更新源; 未调用则 pos 不刷新)
void adc_follower_fetch(AdcFollower *me);

// 校准三步协议 (前缀 EE, 与电机 EF 隔离):
//   EE 01 → cal_white()       车放白纸上, 记录白值
//   EE 02 → cal_black_start() 推车过黑线, 自动追踪每路最远离白值的采样
//   EE 03 → cal_finish()      停止追踪, 计算门限+极性, 返回 buf 长度
void adc_follower_cal_white(AdcFollower *me);
void adc_follower_cal_black_start(AdcFollower *me);
int  adc_follower_cal_finish(AdcFollower *me, char *buf, int max_len);

// ISR 串口字节分发 (Application 层不再直接读写 cal_pending/sns_pending)
// 返回 1=字节已消费, 0=未消费 (交给 mot_rx 继续处理)
uint8_t adc_follower_handle_cal_byte(AdcFollower *me, uint8_t byte);
uint8_t adc_follower_handle_sns_byte(AdcFollower *me, uint8_t byte);

#endif
