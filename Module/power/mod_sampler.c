// 采样管理状态机实现 — ADHardC Module 层
// Module 层状态机模式
//
// 状态转换图:
//   INIT ──(DMA配置OK)──→ CAL_ZERO (启动校准)
//   CAL_ZERO ──(零点采集完成)──→ CAL_SPAN
//   CAL_SPAN ──(满量程采集完成)──→ CAL_DONE
//   CAL_DONE ──(start)──→ RUN ──(溢出检测)──→ OVERFLOW
//   RUN ──(硬件错误)──→ FAULT
//   OVERFLOW ──(复位恢复)──→ INIT
//   FAULT ──(硬件恢复)──→ INIT
//
// 使用方式:
//   1. 用户创建子类结构体, ModSmpl 为第一个成员
//   2. 构造时调用 mod_smpl_init(), 再绑定 ADC 设备和状态指针
//   3. 在 tick 中实现具体采样逻辑 (DMA 启动/读取/滤波/校准计算)
//   4. 校准参数 (cal_zero/cal_span) 持久化到 Flash (用户实现)
//   5. 其他 Module 通过 data_ready 标志同步读取

#include "mod_sampler.h"
#include <string.h>

// ======== 构造 ========

void mod_smpl_init(ModSmpl *me, const ModSmpl_Param *param) {
  memset(me, 0, sizeof(*me));
  me->st = SMPL_INIT;
  if (param) {
    me->param = *param;
  }
  me->overflow_cnt = 0;
}

// ======== 状态机驱动 (每 tick 调用一次) ========

/*
  每 tick 流程:

  SMPL_INIT:
    1. 配置 DMA (循环模式, 半满/全满中断)
    2. 绑定 ADC 通道到 DMA 缓冲区
    3. 分配 raw_buf / filt_buf 内存
    4. DMA 配置完成 → 等待校准启动命令
    5. 如果跳过校准 → 直接进入 SMPL_RUN (使用默认 cal_zero/cal_span)

  SMPL_CAL_ZERO:
    1. 输入端短接 (或切换到 GND)
    2. 采集 N 次 (如 100 次), 取平均值 → cal_zero
    3. 写入 me->param.cal_zero
    4. 自动跳转 → SMPL_CAL_SPAN

  SMPL_CAL_SPAN:
    1. 输入端接已知参考电压 (如 3.3V)
    2. 采集 N 次, 取平均值 → cal_span
    3. 计算各通道增益系数: gain = ref_voltage / (cal_span - cal_zero)
    4. 写入 me->param.cal_span
    5. 自动跳转 → SMPL_CAL_DONE

  SMPL_CAL_DONE:
    1. 校准参数已就绪
    2. 等待 start() 命令进入 RUN
    3. 可选: 持久化校准参数到 Flash

  SMPL_RUN:
    1. DMA 已启动 (循环扫描)
    2. 每 tick 检查 DMA 半满/全满标志:
      - 半满中断: 处理缓冲区前半段, 设置 data_ready
      - 全满中断: 处理缓冲区后半段, 设置 data_ready
    3. 数据后处理:
      - 原始值 → 工程值: val = (raw - cal_zero) * gain
      - 滑动平均滤波 (窗口大小 smooth_window)
    4. DMA 溢出检测:
      - 如果全满中断触发时上一次数据未被消费 → overflow_cnt++
      - overflow_cnt > overflow_thresh → SMPL_OVERFLOW
    5. 硬件错误检测:
      - ADC 模块时钟异常 / 参考电压丢失 → SMPL_FAULT

  SMPL_OVERFLOW:
    1. 记录溢出事件 (通过 status_)
    2. 丢弃当前缓冲区
    3. 复位 DMA (重新 init)
    4. → SMPL_INIT

  SMPL_FAULT:
    1. 记录故障码 (通过 status_)
    2. 停止 DMA
    3. 等待硬件恢复 → SMPL_INIT
*/
void mod_smpl_tick(ModSmpl *me) {
  me->tick_cnt++;

  switch (me->st) {

  case SMPL_INIT:
    // TODO: 用户在此插入 DMA 初始化和通道绑定
    // 1. 配置 DMA (循环模式, 数据宽度, 缓冲区大小)
    //    adc_start_dma(adc_devices_[0]);  // 或批量启动所有通道
    // 2. 绑定 raw 缓冲区
    //    adc_devices_[0]->raw = me->raw_buf;
    // 3. 初始化通道校准参数
    //    for (int i = 0; i < num_channels; i++) {
    //      channels_[i].cal_offset = param.cal_zero;
    //      channels_[i].cal_gain   = param.cal_ref_voltage
    //                              / (param.cal_span - param.cal_zero);
    //    }
    //
    // 初始化完成后等待校准或直接运行
    // (如果已有校准参数, 可以直接跳到 SMPL_RUN)
    // me->st = SMPL_CAL_DONE;  // 跳过校准, 使用已有参数
    break;

  case SMPL_CAL_ZERO:
    // TODO: 零点校准
    // 步骤:
    // 1. 切换输入到 GND (通过模拟开关或要求用户短接)
    // 2. 采集 N 次:
    //    static uint16_t zero_samples = 0;
    //    static float    zero_sum = 0;
    //    zero_sum += adc_read_ch(adc_devices_[0], 0);
    //    zero_samples++;
    //    if (zero_samples >= 100) {
    //      me->param.cal_zero = zero_sum / zero_samples;
    //      zero_samples = 0;
    //      zero_sum = 0;
    //      me->st = SMPL_CAL_SPAN;
    //    }
    break;

  case SMPL_CAL_SPAN:
    // TODO: 满量程校准
    // 步骤:
    // 1. 切换输入到参考电压 (如板上 3.3V 基准)
    // 2. 采集 N 次, 取平均值
    // 3. 计算增益: gain = ref_voltage / (span_avg - cal_zero)
    // 4. 写入各通道的 cal_offset 和 cal_gain
    //    for (int i = 0; i < num_channels; i++) {
    //      channels_[i].cal_offset = me->param.cal_zero;
    //      channels_[i].cal_gain   = me->param.cal_ref_voltage
    //                              / (span_avg - me->param.cal_zero);
    //    }
    //    me->st = SMPL_CAL_DONE;
    break;

  case SMPL_CAL_DONE:
    // 校准完成, 等待 start() 命令
    // 可选: 持久化校准参数到 Flash
    // flash_write_calibration(&me->param);
    break;

  case SMPL_RUN: {
    // === 主采样循环 (用户实现) ===

    // 1. 检查 DMA 传输状态
    // if (dma_half_complete_flag) {
    //   // 处理缓冲区前半段
    //   process_half_buffer(me);
    //   me->data_ready = true;
    // } else if (dma_full_complete_flag) {
    //   // 处理缓冲区后半段
    //   process_full_buffer(me);
    //   me->data_ready = true;
    // }

    // 2. 数据后处理 (在 process_buffer 中):
    // for (int i = 0; i < num_channels; i++) {
    //   uint16_t raw = raw_buf[i];
    //   // 转换为工程值
    //   channels_[i].raw_val  = (float)raw;
    //   channels_[i].filt_val = (raw - channels_[i].cal_offset) * channels_[i].cal_gain;
    //   // 滑动平均滤波 (窗口大小 smooth_window)
    //   // TODO: 实现环形缓冲滤波
    // }

    // 3. DMA 溢出检测:
    // if (dma_overrun_flag) {
    //   me->overflow_cnt++;
    //   if (me->overflow_cnt > me->param.overflow_thresh) {
    //     me->st = SMPL_OVERFLOW;
    //     // ERROR_SET(status_->errors, ERR_DMA_OVERFLOW);
    //   }
    // } else {
    //   me->overflow_cnt = 0;
    // }

    // 4. 硬件错误检测:
    // if (adc_hw_error) {
    //   me->st = SMPL_FAULT;
    //   // ERROR_SET(status_->errors, ERR_ADC_FAULT);
    // }
    break;
  }

  case SMPL_OVERFLOW:
    // TODO: 溢出恢复
    // 1. 记录溢出事件
    // 2. 停止 DMA
    //    adc_stop_dma(adc_devices_[0]);
    // 3. 丢弃缓冲区, 重置标志
    //    me->data_ready = false;
    //    me->overflow_cnt = 0;
    // 4. 重新初始化 DMA
    //    me->st = SMPL_INIT;
    break;

  case SMPL_FAULT:
    // TODO: 故障处理
    // 1. 停止 DMA
    // 2. 记录故障码 (通过 status_)
    // 3. 等待硬件恢复 (如参考电压恢复)
    // if (adc_hw_ok) {
    //   ERROR_CLEAR(status_->errors, ERR_ADC_FAULT);
    //   me->st = SMPL_INIT;
    // }
    break;

  default:
    break;
  }
}

// ======== 控制接口 ========

void mod_smpl_start_calibration(ModSmpl *me) {
  if (me->st == SMPL_INIT || me->st == SMPL_CAL_DONE) {
    me->st = SMPL_CAL_ZERO;
    me->cal_in_progress = true;
  }
}

void mod_smpl_start(ModSmpl *me) {
  if (me->st == SMPL_CAL_DONE || me->st == SMPL_INIT) {
    // 如果还在 INIT, 跳过校准直接运行 (使用默认/已有参数)
    me->st = SMPL_RUN;
    me->data_ready = false;
  }
}

void mod_smpl_stop(ModSmpl *me) {
  if (me->st == SMPL_RUN) {
    // 停止 DMA
    // adc_stop_dma(adc_devices_[0]);
    me->st = SMPL_CAL_DONE;  // 回到就绪状态, 可重新 start
    me->data_ready = false;
  }
}

// ======== 查询接口 ========

bool mod_smpl_is_data_ready(const ModSmpl *me) {
  return me->data_ready;
}

void mod_smpl_clear_data_ready(ModSmpl *me) {
  me->data_ready = false;
}

float mod_smpl_get_filt_val(const ModSmpl *me, uint8_t ch) {
  // TODO: 用户实现 — 从 channels_[ch].filt_val 读取
  // if (ch < me->num_channels) {
  //   return me->channels_[ch].filt_val;
  // }
  (void)ch;
  return 0.0f;
}

SmplSt mod_smpl_get_state(const ModSmpl *me) {
  return me->st;
}
