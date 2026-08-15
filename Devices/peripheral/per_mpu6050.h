#ifndef PER_MPU6050_H
#define PER_MPU6050_H

// MPU6050 六轴 IMU 外设 — SensorBase 子类 + 全局传输接缝
//
// 迁移: 原 Devices/sensor/mpu6050.c + mpu6050.h (阶段2 peripheral 域收编)
// 双轨设计:
//   - 对象契约: PerMpu6050 (SensorBase 第一成员) + per_mpu6050_init/measure
//   - 全局 API: 实现 comp_mpu.h 声明的 4 个传输接缝 (转发单例 s_mpu->bus),
//     DMP 固件 (comp_mpu_dmp.c) 零改动调用
//
// 总线: Iic* (HW 或 SW 模式, 7-bit 从机地址 0x68 由 App 配总线注入)
// 改造: 原 mpu6050.c 的软件 I2C bit-bang (i2c_*) 已删除, 底层统一走 Iic 类.

#include "comp_sensor.h"
#include "com_i2c.h"

typedef struct {
  SensorBase base;         // 测量契约 (第一成员)
  Iic *bus;                // 总线 (HW 或 SW, 由 Iic 决定)
  int16_t gyro[3];         // 最新陀螺仪 (gx,gy,gz)
  int16_t accel[3];        // 最新加速度 (ax,ay,az)
  int16_t temp;            // 最新温度
  float pitch, roll, yaw;  // DMP 欧拉角 (度)
  uint8_t dmp_enabled;     // DMP 是否已启动 (per_mpu6050_dmp_start 置位)
  uint8_t valid;           // 最新读数有效
} PerMpu6050;

// 初始化: 绑总线 + 设单例 → 基础寄存器配置 (复位/唤醒/量程/采样率) → 绑 measure
int per_mpu6050_init(PerMpu6050 *me, Iic *bus);

// [可选] 启动 DMP: 加载固件 → 配置特性 → 自检 → 启动, 成功置 dmp_enabled
//   依赖 comp_mpu.h 平台层 (mpu_set_* 等) 已实现; 不调用则走寄存器回退
int per_mpu6050_dmp_start(PerMpu6050 *me);

// 触发一次测量: DMP 已启动 → mpu_dmp_read_fifo 更新欧拉角 + 陀螺仪;
//   否则寄存器回退读 accel/gyro/temp. 成功置 valid, 返回 ERR_OK
int per_mpu6050_measure(PerMpu6050 *me);

// 仅读陀螺仪 Z 轴 (2 字节, ~200us) — 转弯角度积分用, 比读 6 字节快
int16_t per_mpu6050_read_gyro_z(PerMpu6050 *me);

#endif  // PER_MPU6050_H
