// SensorBase — 测量设备契约身份基类
//
// 定位: 与 CommBase (comp_comm.h) 平行的测量契约, 只承载 实例名 + 生命周期 + 测量能力.
//   数据面在子类 (PerUltrasonic/PerMpu6050, Devices/peripheral).
// 风格对齐 comp_comm.h: 契约身份基类 + 空安全分发.
//
// measure: 触发一次测量并返回 ErrorCode (ERR_OK=0 成功). 未绑定 → ERR_NOT_SUPPORT.
//   轮询型设备 (如超声波 tick/process 模型) 可不绑 measure, 由 App 直接轮询子类 API.

#ifndef COMP_SENSOR_H
#define COMP_SENSOR_H

#include <stdint.h>
#include "comp_error_code.h"

typedef struct SensorBase SensorBase;

// 测量虚函数 — 触发一次测量, 返回 ErrorCode (0=成功, 负值=错误码)
typedef int (*sensor_measure_fn)(SensorBase *me);

// 基类 — 契约身份 + 测量能力
struct SensorBase {
  const char *name;           // 实例名 (调试/诊断用)
  uint8_t inited;             // 初始化标志 (0=未初始化, 1=已初始化)
  sensor_measure_fn measure;  // [可选] 测量触发, 可空
};

// 基类构造 / 析构
void sensor_base_init(SensorBase *me, const char *name);
void sensor_base_deinit(SensorBase *me);

// 测量分发 — measure 为空 → ERR_NOT_SUPPORT
int sensor_measure(SensorBase *me);

#endif  // COMP_SENSOR_H
