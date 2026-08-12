#ifndef ADC_FOLLOWER_H
#define ADC_FOLLOWER_H

// 红外循迹传感器驱动 —— AdcBase 的子类
// 使用 ADC1 + DMA1 通道 1, 8 路扫描 (PA0-PA7)
// TIM3 TRGO 触发，DMA 循环写入 AdcBase.raw[8]

#include "comp_adc.h"
#include "stm32f1xx_hal.h"

typedef struct {
  AdcBase            base;        // 基类
  uint16_t           raw_buf[8];  // [基类绑定] DMA 缓冲区 (8 通道)
  ADC_HandleTypeDef *hadc;        // HAL ADC 句柄
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

void adc_follower_init(AdcFollower *me, ADC_HandleTypeDef *hadc,
                        const int16_t *threshold);
void adc_follower_deinit(AdcFollower *me);

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
