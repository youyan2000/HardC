#ifndef MOD_SAMPLER_H
#define MOD_SAMPLER_H

// 采样管理状态机模板 — ADC-OOP Module 层
// Module 层状态机模式
//
// 状态: INIT → CAL_ZERO → CAL_SPAN → CAL_DONE → RUN → OVERFLOW → FAULT
//
// 设计要点:
//   - 状态机是通用手段, 不限于 Module 层 — 但采样管理天然适合 Module 层
//   - 管理多个 ADC 通道的采样调度 (单次/连续/DMA)
//   - 校准流程: 零点 → 满量程 → 线性化 (支持多点校准扩展)
//   - 采样溢出检测 (DMA 半满/全满中断)
//   - 数据就绪标记 (供其他 Module 读取, 避免数据竞争)
//
// 继承方式 (用户自建子类):
//   typedef struct {
//     ModSmpl       base;          // ← 父类必须是第一个成员
//     AdcBase      *adc_dev[N];    // 管理的 ADC 设备列表
//     Module_Status *status;       // 共享状态
//   } MySampler;

#include <stdbool.h>
#include <stdint.h>

typedef struct AdcBase AdcBase;

// ======== 采样器状态 ========
typedef enum {
  SMPL_INIT,       // 初始化: DMA 配置, 通道绑定, 内存分配
  SMPL_CAL_ZERO,   // 校准-零点: 输入端短路, 采集零点偏移
  SMPL_CAL_SPAN,   // 校准-满量程: 输入已知参考电压, 计算增益系数
  SMPL_CAL_DONE,   // 校准完成: 写入校准参数, 等待启动
  SMPL_RUN,        // 运行: 持续采样 + 滤波 + 数据就绪通知
  SMPL_OVERFLOW,   // 溢出: DMA 过载, 数据丢失, 需复位
  SMPL_FAULT,      // 故障: 硬件错误 (ADC 模块异常 / 参考电压丢失)
} SmplSt;

// ======== 采样器参数 (YAML 可配置) ========
typedef struct {
  uint16_t sample_rate_hz;    // 采样率 (Hz)
  uint16_t oversample_n;      // 过采样倍数 (1/2/4/8/16)
  float    cal_zero;          // 零点校准值 (ADC 码值)
  float    cal_span;          // 满量程校准值 (ADC 码值)
  float    cal_ref_voltage;   // 校准参考电压 (V)
  uint16_t overflow_thresh;   // 溢出阈值 (连续溢出 tick 数后报警)
  uint16_t smooth_window;     // 滑动平均窗口大小 (0=不滤波)
} ModSmpl_Param;

// ======== 采样通道描述 ========
typedef struct {
  uint8_t  ch_idx;      // ADC 通道号 (硬件)
  float    cal_offset;  // 校准偏移 (y = k * x + b 中的 b)
  float    cal_gain;    // 校准增益 (y = k * x + b 中的 k)
  float    raw_val;     // 当前原始值 (ADC 码)
  float    filt_val;    // 当前滤波值 (工程单位)
} SmplChannel;

// ======== 采样管理器 (父类模板) ========
#define MAX_CH 16

typedef struct {
  // --- 状态机 ---
  SmplSt st;

  // --- 参数 ---
  ModSmpl_Param param;

  // --- 指针注入: 用户构造时绑定 ---
  // AdcBase       *adc_devices_[N];  // 管理的 ADC 设备列表
  // Module_Status *status_;          // 共享状态 (错误码/心跳)

  // --- 通道管理 ---
  // SmplChannel   *channels_;        // 通道数组 (用户分配)
  uint8_t       num_channels;        // 实际通道数

  // --- 运行时状态 ---
  uint32_t      tick_cnt;            // tick 计数
  uint16_t      overflow_cnt;        // 溢出计数
  bool          data_ready;          // 数据就绪标志 (一轮采样完成)
  bool          cal_in_progress;     // 校准进行中标志
} ModSmpl;

// ======== API ========

// 构造: 初始化状态机和参数
void   mod_smpl_init(ModSmpl *me, const ModSmpl_Param *param);

// 每 tick 驱动状态机 (由定时器 ISR 或 App 主循环调用)
void   mod_smpl_tick(ModSmpl *me);

// 启动校准流程: IDLE → CAL_ZERO (自动走完 ZERO → SPAN → DONE)
void   mod_smpl_start_calibration(ModSmpl *me);

// 启动采样: CAL_DONE / IDLE → RUN
void   mod_smpl_start(ModSmpl *me);

// 停止采样: RUN → IDLE
void   mod_smpl_stop(ModSmpl *me);

// 查询数据是否就绪 (一轮采样完成, 滤波值已更新)
bool   mod_smpl_is_data_ready(const ModSmpl *me);

// 消费数据就绪标志 (读后清零, 等待下一轮)
void   mod_smpl_clear_data_ready(ModSmpl *me);

// 获取某通道的滤波值 (工程单位)
float  mod_smpl_get_filt_val(const ModSmpl *me, uint8_t ch);

// 获取当前状态
SmplSt mod_smpl_get_state(const ModSmpl *me);

#endif
